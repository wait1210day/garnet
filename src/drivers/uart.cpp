// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include "stm32g4xx_ll_dma.h"

#include <algorithm>

#include "base/critical_section.h"
#include "base/panic.h"
#include "drivers/ll_helper.h"
#include "drivers/uart.h"

namespace uart
{

ErrStatus Transmitter::init(USART_TypeDef *uart, DmaChannel dma)
{
    GCHECK(uart && "invalid uart peripheral address");
    uart_ = uart;
    dma_ = dma;

    notifier_.init();
    mutex_.init();

    return ErrStatus::kOk;
}

std::optional<Transmitter::Scoped> Transmitter::acquire(TickType_t timeout)
{
    if (uart_ && mutex_.try_lock_for(timeout))
        return Scoped{this};
    return std::nullopt;
}

ErrStatus Transmitter::transmit(std::span<const uint8_t> data, XferMethod method,
                                TickType_t timeout)
{
    if (data.empty())
        return ErrStatus::kOk;
    // non-blocking transfer is impossible on asynchronous peripheral,
    // no matter if it is polling method or DMA method.
    if (timeout == 0)
        return ErrStatus::kTimeout;

    if (method != XferMethod::kPolling && method != XferMethod::kDMA)
        return ErrStatus::kInvalidArg;
    if (method == XferMethod::kDMA)
    {
        if (data.size_bytes() > 0xffff)
            return ErrStatus::kUnsupported;
        if (!dma_.instance)
            return ErrStatus::kUnsupported;
    }

    // acquire the lock
    if (method == XferMethod::kDMA)
        return transmit_dma(data, timeout);
    if (method == XferMethod::kPolling)
        return transmit_poll(data, timeout);

    panic::unreachable_code();
}

ErrStatus Transmitter::transmit_poll(std::span<const uint8_t> data, TickType_t timeout)
{
    const TickType_t start_tick = xTaskGetTickCount();
    TickType_t last_delay_tick = start_tick;

    for (uint8_t byte : data)
    {
        for (;;)
        {
            const TickType_t now = xTaskGetTickCount();
            if (timeout != portMAX_DELAY && now - start_tick >= timeout)
                return ErrStatus::kTimeout;

            // Bound continuous polling to one tick so lower-priority tasks run.
            if (now != last_delay_tick)
            {
                vTaskDelay(1);
                last_delay_tick = xTaskGetTickCount();
                continue;
            }

            if (LL_USART_IsActiveFlag_TXE_TXFNF(uart_))
                break;
            taskYIELD();
        }

        LL_USART_TransmitData8(uart_, byte);
    }

    // The USART has accepted the final byte; wait_flush() waits for the TX pin.
    return ErrStatus::kOk;
}

ErrStatus Transmitter::transmit_dma(std::span<const uint8_t> data, TickType_t timeout)
{
    // drain an ISR completion that raced with the preceding timeout
    (void) notifier_.take(0);

    dma_transfer_error_.store(false, std::memory_order_relaxed);

    // Prepare DMA transfer
    LL_DMA_DisableChannel(dma_.instance, dma_.channel);
    LL_DMA_DisableIT_HT(dma_.instance, dma_.channel);
    llhelper::dma_clear_intr_flags(dma_.instance, dma_.channel);

    const uint32_t periph_addr = LL_USART_DMA_GetRegAddr(uart_, LL_USART_DMA_REG_DATA_TRANSMIT);
    LL_DMA_SetPeriphAddress(dma_.instance, dma_.channel, periph_addr);
    LL_DMA_SetMemoryAddress(dma_.instance, dma_.channel,
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(data.data())));
    LL_DMA_SetDataLength(dma_.instance, dma_.channel, data.size_bytes());

    LL_DMA_EnableIT_TC(dma_.instance, dma_.channel);
    LL_DMA_EnableIT_TE(dma_.instance, dma_.channel);

    __DMB();

    // Go!
    LL_DMA_EnableChannel(dma_.instance, dma_.channel);
    LL_USART_EnableDMAReq_TX(uart_);

    const bool acquired = notifier_.take(timeout);

    LL_USART_DisableDMAReq_TX(uart_);
    LL_DMA_DisableChannel(dma_.instance, dma_.channel);
    LL_DMA_DisableIT_TC(dma_.instance, dma_.channel);
    LL_DMA_DisableIT_TE(dma_.instance, dma_.channel);
    llhelper::dma_clear_intr_flags(dma_.instance, dma_.channel);

    if (!acquired)
    {
        // flush the fifo to prevent spitting corrupted data in the
        // next transfer request.
        if (LL_USART_IsEnabledFIFO(uart_))
            LL_USART_RequestTxDataFlush(uart_);

        return ErrStatus::kTimeout;
    }

    std::atomic_signal_fence(std::memory_order_acquire);
    if (dma_transfer_error_.load(std::memory_order_relaxed))
    {
        if (LL_USART_IsEnabledFIFO(uart_))
            LL_USART_RequestTxDataFlush(uart_);
        return ErrStatus::kFail;
    }

    return ErrStatus::kOk;
}

ErrStatus Transmitter::wait_flush(TickType_t timeout)
{
    {
        const TaskCriticalSection critical;

        // DMA and USART completion share this semaphore under the transmitter lock.
        // A DMA IRQ may have completed just after transmit_dma() timed out.
        (void) notifier_.take(0);

        if (LL_USART_IsActiveFlag_TC(uart_))
            return ErrStatus::kOk;
        if (timeout == 0)
            return ErrStatus::kTimeout;

        LL_USART_EnableIT_TC(uart_);
    }

    (void) notifier_.take(timeout);

    const TaskCriticalSection critical;
    LL_USART_DisableIT_TC(uart_);
    // TC may have become set as the semaphore wait timed out.
    return LL_USART_IsActiveFlag_TC(uart_) ? ErrStatus::kOk : ErrStatus::kTimeout;
}

void Transmitter::on_usart_irq(uint32_t isr, uint32_t cr1, BaseType_t& wake)
{
    if (!(cr1 & USART_CR1_TCIE) || !(isr & USART_ISR_TC))
        return;

    LL_USART_DisableIT_TC(uart_);
    notifier_.give_from_isr(wake);
}

void Transmitter::on_tx_dma_irq(BaseType_t& wake)
{
    DMA_TypeDef *dma = dma_.instance;
    const uint32_t channel = dma_.channel;

    // this function "normalize" any provided channel flags to a fixed channel 1,
    // so use `DMA_ISR_xxIF1` to determine a specific flag.
    const uint32_t flags = llhelper::dma_get_intr_flags(dma, channel);
    llhelper::dma_clear_intr_flags(dma, channel);

    if ((flags & DMA_ISR_TEIF1) && LL_DMA_IsEnabledIT_TE(dma, channel))
    {
        // transfer error
        dma_transfer_error_.store(true, std::memory_order_relaxed);
    }
    else if ((flags & DMA_ISR_TCIF1) && LL_DMA_IsEnabledIT_TC(dma, channel))
    {
        // transfer completion
        dma_transfer_error_.store(false, std::memory_order_relaxed);
    }
    else
        return;

    std::atomic_signal_fence(std::memory_order_release);

    notifier_.give_from_isr(wake);
}

ErrStatus Receiver::init(USART_TypeDef *uart, DmaChannel dma)
{
    GCHECK(uart && "invalid peripheral address");

    uart_ = uart;
    dma_ = dma;
    mutex_.init();
    notifier_.init();

    // prepare DMA transfer
    LL_DMA_DisableChannel(dma.instance, dma.channel);
    LL_DMA_DisableIT_HT(dma.instance, dma.channel);
    llhelper::dma_clear_intr_flags(dma.instance, dma.channel);

    LL_DMA_SetPeriphAddress(dma.instance, dma.channel,
                            LL_USART_DMA_GetRegAddr(uart, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(dma.instance, dma.channel,
                            static_cast<uint32_t>(reinterpret_cast<uintptr_t>(dma_buffer_.data())));
    LL_DMA_SetDataLength(dma.instance, dma.channel, kDmaSize);

    LL_DMA_EnableIT_HT(dma.instance, dma.channel);
    LL_DMA_EnableIT_TC(dma.instance, dma.channel);
    LL_DMA_EnableIT_TE(dma.instance, dma.channel);

    // prepare USART peripheral
    LL_USART_RequestRxDataFlush(uart);
    LL_USART_ClearFlag_IDLE(uart);
    LL_USART_ClearFlag_ORE(uart);
    LL_USART_ClearFlag_FE(uart);
    LL_USART_ClearFlag_NE(uart);
    LL_USART_ClearFlag_PE(uart);

    __DMB();

    LL_DMA_EnableChannel(dma.instance, dma.channel);
    LL_USART_EnableDMAReq_RX(uart);
    LL_USART_EnableIT_IDLE(uart);
    LL_USART_EnableIT_ERROR(uart);
    LL_USART_EnableIT_PE(uart);

    return ErrStatus::kOk;
}

std::optional<Receiver::Scoped> Receiver::acquire(TickType_t timeout)
{
    if (uart_ && mutex_.try_lock_for(timeout))
        return Scoped{this};
    return std::nullopt;
}

Receiver::ReadResult Receiver::read_some(std::span<uint8_t> data, TickType_t timeout)
{
    if (data.empty())
        return {0, Status::kInvalidArg};

    const TickType_t start = xTaskGetTickCount();
    for (;;)
    {
        TickType_t remaining = portMAX_DELAY;
        {
            const TaskCriticalSection critical;
            if (error_ != Status::kOk)
            {
                const Status error = error_;
                error_ = Status::kOk;
                return {0, error};
            }

            if (ring_count_ > 0)
            {
                const size_t count = std::min(data.size(), ring_count_);
                const size_t first = std::min(count, kRingSize - ring_tail_);
                std::copy_n(ring_.data() + ring_tail_, first, data.data());
                if (count > first)
                    std::copy_n(ring_.data(), count - first, data.data() + first);
                ring_tail_ = (ring_tail_ + count) % kRingSize;
                ring_count_ -= count;
                return {count, Status::kOk};
            }

            // Drain a notification after consuming all previously available data.
            (void) notifier_.take(0);

            if (timeout != portMAX_DELAY)
            {
                const TickType_t elapsed = xTaskGetTickCount() - start;
                if (elapsed >= timeout)
                    return {0, Status::kTimeout};
                remaining = timeout - elapsed;
            }
        }

        (void) notifier_.take(remaining);
    }
}

void Receiver::set_error(Status error, BaseType_t& wake)
{
    if (error_ == Status::kOk)
        error_ = error;
    ring_head_ = ring_tail_ = ring_count_ = 0;
    notifier_.give_from_isr(wake);
}

void Receiver::sync_dmabuf_to_ringbuf(uint32_t flags, BaseType_t& wake)
{
    const size_t remaining = LL_DMA_GetDataLength(dma_.instance, dma_.channel);
    const size_t position = (kDmaSize - remaining) % kDmaSize;
    const size_t count = (position + kDmaSize - dma_read_pos_) % kDmaSize;

    // A pending milestone must lie between the last copied position and the
    // current DMA position. Otherwise DMA has made at least one extra lap.
    const uint32_t milestones = flags & (DMA_ISR_HTIF1 | DMA_ISR_TCIF1);
    const bool wrapped = position < dma_read_pos_;
    const bool crossed_half = wrapped
        ? (dma_read_pos_ < kDmaSize / 2 || position >= kDmaSize / 2)
        : (dma_read_pos_ < kDmaSize / 2 && position >= kDmaSize / 2);

    if ((milestones && count == 0) ||
        ((milestones & DMA_ISR_HTIF1) && !crossed_half) ||
        ((milestones & DMA_ISR_TCIF1) && !wrapped))
    {
        dma_read_pos_ = position;
        set_error(Status::kOverflow, wake);
        return;
    }

    if (count > kRingSize - ring_count_)
    {
        dma_read_pos_ = position;
        set_error(Status::kOverflow, wake);
        return;
    }

    __DMB();
    size_t copied = 0;
    size_t dma_pos = dma_read_pos_;
    while (copied < count)
    {
        const size_t chunk = std::min(count - copied,
            std::min(kDmaSize - dma_pos, kRingSize - ring_head_));
        std::copy_n(dma_buffer_.data() + dma_pos, chunk, ring_.data() + ring_head_);
        dma_pos = (dma_pos + chunk) % kDmaSize;
        ring_head_ = (ring_head_ + chunk) % kRingSize;
        copied += chunk;
    }
    ring_count_ += count;
    dma_read_pos_ = position;
    if (count != 0)
        notifier_.give_from_isr(wake);
}

void Receiver::restart_dma()
{
    LL_USART_DisableDMAReq_RX(uart_);
    LL_DMA_DisableChannel(dma_.instance, dma_.channel);
    llhelper::dma_clear_intr_flags(dma_.instance, dma_.channel);

    LL_USART_RequestRxDataFlush(uart_);
    LL_DMA_SetDataLength(dma_.instance, dma_.channel, kDmaSize);

    dma_read_pos_ = 0;

    __DMB();
    LL_DMA_EnableChannel(dma_.instance, dma_.channel);
    LL_USART_EnableDMAReq_RX(uart_);
}

void Receiver::on_usart_irq(uint32_t isr, uint32_t cr1, uint32_t cr3, BaseType_t& wake)
{
    const uint32_t errors = isr & (USART_ISR_ORE | USART_ISR_FE |
                                   USART_ISR_NE | USART_ISR_PE);
    if (errors && ((cr3 & USART_CR3_EIE) || (cr1 & USART_CR1_PEIE)))
    {
        Status error = Status::kParity;
        if (errors & USART_ISR_ORE)
            error = Status::kOverrun;
        else if (errors & USART_ISR_FE)
            error = Status::kFraming;
        else if (errors & USART_ISR_NE)
            error = Status::kNoise;

        if (errors & USART_ISR_ORE)
            LL_USART_ClearFlag_ORE(uart_);
        if (errors & USART_ISR_FE)
            LL_USART_ClearFlag_FE(uart_);
        if (errors & USART_ISR_NE)
            LL_USART_ClearFlag_NE(uart_);
        if (errors & USART_ISR_PE)
            LL_USART_ClearFlag_PE(uart_);
        if (isr & USART_ISR_IDLE)
            LL_USART_ClearFlag_IDLE(uart_);

        set_error(error, wake);
        restart_dma();
        return;
    }

    if ((isr & USART_ISR_IDLE) && (cr1 & USART_CR1_IDLEIE))
    {
        LL_USART_ClearFlag_IDLE(uart_);
        // IDLE and DMA milestones can be pending together. Consume both once.
        const uint32_t flags = llhelper::dma_get_intr_flags(dma_.instance, dma_.channel);
        llhelper::dma_clear_intr_flags(dma_.instance, dma_.channel);
        if (flags & DMA_ISR_TEIF1)
        {
            set_error(Status::kDma, wake);
            restart_dma();
        }
        else
            sync_dmabuf_to_ringbuf(flags, wake);
    }
}

void Receiver::on_rx_dma_irq(BaseType_t& wake)
{
    const uint32_t flags = llhelper::dma_get_intr_flags(dma_.instance, dma_.channel);
    llhelper::dma_clear_intr_flags(dma_.instance, dma_.channel);
    if (flags & DMA_ISR_TEIF1)
    {
        set_error(Status::kDma, wake);
        restart_dma();
    }
    else if (flags & (DMA_ISR_HTIF1 | DMA_ISR_TCIF1))
        sync_dmabuf_to_ringbuf(flags, wake);
}

ErrStatus Port::init(const Config& config)
{
    if (!config.uart || !config.rx_dma.instance)
        return ErrStatus::kInvalidArg;
    if (config.tx_dma.instance == config.rx_dma.instance &&
        config.tx_dma.channel == config.rx_dma.channel)
        return ErrStatus::kInvalidArg;

    const ErrStatus result = tx_.init(config.uart, config.tx_dma);
    if (result != ErrStatus::kOk)
        return result;

    uart_ = config.uart;
    return rx_.init(config.uart, config.rx_dma);
}

void Port::on_usart_irq()
{
    if (!uart_)
        return;

    const uint32_t isr = uart_->ISR;
    const uint32_t cr1 = uart_->CR1;
    const uint32_t cr3 = uart_->CR3;
    BaseType_t wake = pdFALSE;
    rx_.on_usart_irq(isr, cr1, cr3, wake);
    tx_.on_usart_irq(isr, cr1, wake);
    portYIELD_FROM_ISR(wake);
}

void Port::on_tx_dma_irq()
{
    if (!uart_)
        return;

    BaseType_t wake = pdFALSE;
    tx_.on_tx_dma_irq(wake);
    portYIELD_FROM_ISR(wake);
}

void Port::on_rx_dma_irq()
{
    if (!uart_)
        return;

    BaseType_t wake = pdFALSE;
    rx_.on_rx_dma_irq(wake);
    portYIELD_FROM_ISR(wake);
}

} // namespace uart

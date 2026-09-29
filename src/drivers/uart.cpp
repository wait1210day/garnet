// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include "stm32g4xx_ll_dma.h"

#include "base/critical_section.h"
#include "base/panic.h"
#include "drivers/ll_helper.h"
#include "drivers/uart.h"

namespace uart
{

ErrStatus Transmitter::init(const Config &config)
{
    if (!config.uart)
        return ErrStatus::kInvalidArg;

    config_ = config;

    notifier_.init();
    mutex_.init();

    return ErrStatus::kOk;
}

std::optional<Transmitter::Scoped> Transmitter::acquire(TickType_t timeout)
{
    if (mutex_.try_lock_for(timeout))
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
        if (!config_.dma)
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
    USART_TypeDef *uart = config_.uart;
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

            if (LL_USART_IsActiveFlag_TXE_TXFNF(uart))
                break;
            taskYIELD();
        }

        LL_USART_TransmitData8(uart, byte);
    }

    // The USART has accepted the final byte; wait_flush() waits for the TX pin.
    return ErrStatus::kOk;
}

ErrStatus Transmitter::transmit_dma(std::span<const uint8_t> data, TickType_t timeout)
{
    DMA_TypeDef *dma = config_.dma;
    USART_TypeDef *uart = config_.uart;
    const uint32_t channel = config_.dma_channel;

    // drain an ISR completion that raced with the preceding timeout
    (void) notifier_.take(0);

    dma_transfer_error_.store(false, std::memory_order_relaxed);

    // Prepare DMA transfer
    LL_DMA_DisableChannel(dma, channel);
    LL_DMA_DisableIT_HT(dma, channel);
    llhelper::dma_clear_intr_flags(dma, channel);

    const uint32_t periph_addr = LL_USART_DMA_GetRegAddr(uart, LL_USART_DMA_REG_DATA_TRANSMIT);
    LL_DMA_SetPeriphAddress(dma, channel, periph_addr);
    LL_DMA_SetMemoryAddress(dma, channel,
        reinterpret_cast<uint32_t>(data.data()));
    LL_DMA_SetDataLength(dma, channel, data.size_bytes());

    LL_DMA_EnableIT_TC(dma, channel);
    LL_DMA_EnableIT_TE(dma, channel);

    __DMB();

    // Go!
    LL_DMA_EnableChannel(dma, channel);
    LL_USART_EnableDMAReq_TX(uart);

    const bool acquired = notifier_.take(timeout);

    LL_DMA_DisableChannel(dma, channel);
    LL_DMA_DisableIT_TC(dma, channel);
    LL_DMA_DisableIT_TE(dma, channel);
    LL_USART_DisableDMAReq_TX(uart);
    llhelper::dma_clear_intr_flags(dma, channel);

    if (!acquired)
    {
        // flush the fifo to prevent spitting corrupted data in the
        // next transfer request.
        if (LL_USART_IsEnabledFIFO(uart))
            LL_USART_RequestTxDataFlush(uart);

        return ErrStatus::kTimeout;
    }

    std::atomic_signal_fence(std::memory_order_acquire);
    if (dma_transfer_error_.load(std::memory_order_relaxed))
    {
        if (LL_USART_IsEnabledFIFO(uart))
            LL_USART_RequestTxDataFlush(uart);
        return ErrStatus::kFail;
    }

    return ErrStatus::kOk;
}

ErrStatus Transmitter::wait_flush(TickType_t timeout)
{
    USART_TypeDef *uart = config_.uart;

    {
        const TaskCriticalSection critical;

        // DMA and USART completion share this semaphore under the transmitter lock.
        // A DMA IRQ may have completed just after transmit_dma() timed out.
        (void) notifier_.take(0);

        if (LL_USART_IsActiveFlag_TC(uart))
            return ErrStatus::kOk;
        if (timeout == 0)
            return ErrStatus::kTimeout;

        LL_USART_EnableIT_TC(uart);
    }

    (void) notifier_.take(timeout);

    const TaskCriticalSection critical;
    LL_USART_DisableIT_TC(uart);
    // TC may have become set as the semaphore wait timed out.
    return LL_USART_IsActiveFlag_TC(uart) ? ErrStatus::kOk : ErrStatus::kTimeout;
}

void Transmitter::on_usart_irq()
{
    USART_TypeDef *uart = config_.uart;
    if (!LL_USART_IsEnabledIT_TC(uart) || !LL_USART_IsActiveFlag_TC(uart))
        return;

    LL_USART_DisableIT_TC(uart);

    BaseType_t wake = pdFALSE;
    notifier_.give_from_isr(wake);
    portYIELD_FROM_ISR(wake);
}

void Transmitter::on_tx_dma_irq()
{
    DMA_TypeDef *dma = config_.dma;
    const uint32_t channel = config_.dma_channel;

    // this function "normalize" any provided channel flags to a fixed channel 1,
    // so use `DMA_ISR_xxIF1` to determine a specific flag.
    const uint32_t flags = llhelper::dma_get_intr_flags(dma, channel);
    llhelper::dma_clear_intr_flags(dma, channel);

    if ((flags & DMA_ISR_TCIF1) && LL_DMA_IsEnabledIT_TC(dma, channel))
    {
        // transfer completion
        dma_transfer_error_.store(false, std::memory_order_relaxed);
    }
    else if ((flags & DMA_ISR_TEIF1) && LL_DMA_IsEnabledIT_TE(dma, channel))
    {
        // transfer error
        dma_transfer_error_.store(true, std::memory_order_relaxed);
    }
    else
        return;

    std::atomic_signal_fence(std::memory_order_release);

    BaseType_t wake = pdFALSE;
    notifier_.give_from_isr(wake);

    portYIELD_FROM_ISR(wake);
}

} // namespace uart

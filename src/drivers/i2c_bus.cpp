// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <utility>

#include "FreeRTOS.h"
#include "task.h"

#include "base/critical_section.h"
#include "drivers/ll_helper.h"
#include "drivers/i2c_bus.h"

// Reference: STM32G4 HAL libary, part of DMA and I2C peripheral

namespace i2c
{
namespace
{

// DMA completion only means the data register has been serviced; I2C TC/TCR
// and STOP determine when to restart, reload and report completion.
constexpr size_t kMaxChunkSize = 255;
constexpr uint32_t kTransferInterrupts = I2C_CR1_TXIE
                                       | I2C_CR1_RXIE
                                       | I2C_CR1_TCIE
                                       | I2C_CR1_STOPIE
                                       | I2C_CR1_NACKIE
                                       | I2C_CR1_ERRIE;
constexpr uint32_t kClearFlags = I2C_ICR_STOPCF
                               | I2C_ICR_NACKCF
                               | I2C_ICR_BERRCF
                               | I2C_ICR_ARLOCF
                               | I2C_ICR_OVRCF
                               | I2C_ICR_TIMOUTCF;

void flush_tx(I2C_TypeDef *i2c)
{
    // Same sequence as HAL's I2C_Flush_TXDR: flushing TXE alone does not
    // acknowledge a pending TXIS request after a failed transfer.
    if (LL_I2C_IsActiveFlag_TXIS(i2c))
        LL_I2C_TransmitData8(i2c, 0);
    LL_I2C_ClearFlag_TXE(i2c);
}

} // namespace anonymous

const char *get_err_str(Err e)
{
    switch (e)
    {
        case Err::kOk:
            return "ok";
        case Err::kInvalidArg:
            return "invalid argument";
        case Err::kNotInitialized:
            return "not initialized";
        case Err::kInvalidContext:
            return "invalid context";
        case Err::kDmaNotConfigured:
            return "dma not configured";
        case Err::kBusy:
            return "bus is busy";
        case Err::kTimeout:
            return "timed out";
        case Err::kNack:
            return "device responded nack";
        case Err::kBus:
            return "bus error";
        case Err::kArbitrationLost:
            return "arbitration lost";
        case Err::kOverrun:
            return "overrun";
        case Err::kDma:
            return "dma error";
        case Err::kUnexpectedStop:
            return "unexpected stop";
    }
    return "unknown error";
}

ErrStatus Bus::init(const Config& config)
{
    // conflict DMA instance and channel
    if (config.tx_dma.instance &&
        config.tx_dma.instance == config.rx_dma.instance &&
        config.tx_dma.channel == config.rx_dma.channel)
        return ErrStatus::kInvalidArg;

    if (i2c_ || !LL_I2C_IsEnabled(config.i2c) || LL_I2C_IsActiveFlag_BUSY(config.i2c) ||
        (LL_I2C_ReadReg(config.i2c, CR1) & (kTransferInterrupts | I2C_CR1_ADDRIE |
                                          I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN)))
        return ErrStatus::kFail;

    for (const auto dma : {config.tx_dma, config.rx_dma})
    {
        if (dma.instance && LL_DMA_IsEnabledChannel(dma.instance, dma.channel))
            return ErrStatus::kFail;
    }

    bus_lock_.init();
    completion_.init();

    tx_dma_ = config.tx_dma;
    rx_dma_ = config.rx_dma;
    i2c_ = config.i2c;
    return ErrStatus::kOk;
}

std::optional<Bus::Scoped> Bus::acquire(TickType_t timeout)
{
    if (!i2c_ || !bus_lock_.try_lock_for(timeout))
        return std::nullopt;
    return Scoped{*this};
}

Err Bus::probe(uint16_t dev_addr, TickType_t timeout)
{
    return transmit(dev_addr, {}, XferMethod::kIRQ, timeout);
}

Err Bus::transmit(uint16_t dev_addr, std::span<const uint8_t> txbuf,
                  XferMethod method, TickType_t xfer_timeout)
{
    return transfer(dev_addr, txbuf, {}, method, xfer_timeout);
}

Err Bus::receive(uint16_t dev_addr, std::span<uint8_t> rxbuf,
                 XferMethod method, TickType_t xfer_timeout)
{
    if (rxbuf.empty())
        return Err::kInvalidArg;
    return transfer(dev_addr, {}, rxbuf, method, xfer_timeout);
}

Err Bus::transmit_receive(uint16_t dev_addr, std::span<const uint8_t> txbuf,
                                 std::span<uint8_t> rxbuf, XferMethod method,
                                 TickType_t xfer_timeout)
{
    if (txbuf.empty() || rxbuf.empty())
        return Err::kInvalidArg;
    return transfer(dev_addr, txbuf, rxbuf, method, xfer_timeout);
}

Err Bus::transfer(uint16_t dev_addr, std::span<const uint8_t> txbuf,
                         std::span<uint8_t> rxbuf, XferMethod method,
                         TickType_t xfer_timeout)
{
    if (!i2c_)
        return Err::kNotInitialized;

    if (dev_addr > 0x7f)
        return Err::kInvalidArg;

    if (method == XferMethod::kDMA &&
        ((!txbuf.empty() && !tx_dma_.instance) || (!rxbuf.empty() && !rx_dma_.instance)))
        return Err::kDmaNotConfigured;

    if (xfer_timeout == 0)
        return Err::kTimeout;

    {
        const TaskCriticalSection critical;

        if (!LL_I2C_IsEnabled(i2c_))
            return Err::kNotInitialized;
        if (LL_I2C_IsActiveFlag_BUSY(i2c_))
            return Err::kBusy;

        // Drain an ISR completion that raced with the preceding timeout.
        (void)completion_.take(0);

        LL_I2C_WriteReg(i2c_, ICR, kClearFlags);
        if (LL_I2C_IsActiveFlag_RXNE(i2c_))
            (void) LL_I2C_ReceiveData8(i2c_);
        flush_tx(i2c_);

        tx_ = txbuf;
        rx_ = rxbuf;
        address_ = dev_addr << 1;
        method_ = method;
        reading_ = txbuf.empty() && !rxbuf.empty();
        result_ = Err::kOk;
        active_ = true;

        start_chunk(reading_ ? LL_I2C_GENERATE_START_READ : LL_I2C_GENERATE_START_WRITE);
    }

    (void)completion_.take(xfer_timeout);

    const TaskCriticalSection critical;
    if (active_)
    {
        // Disable DMA before returning borrowed buffers, even if no IRQ arrived.
        quiesce(true);
        active_ = false;
        if (result_ == Err::kOk)
            result_ = Err::kTimeout;
    }
    tx_ = {};
    rx_ = {};
    return result_;
}

void Bus::start_chunk(uint32_t request)
{
    const auto size = reading_ ? rx_.size() : tx_.size();
    chunk_remaining_ = static_cast<uint16_t>(std::min(size, kMaxChunkSize));

    uint32_t end_mode;
    if (size > kMaxChunkSize)
        end_mode = LL_I2C_MODE_RELOAD;
    else if (!reading_ && !rx_.empty())
        end_mode = LL_I2C_MODE_SOFTEND;
    else
        end_mode = LL_I2C_MODE_AUTOEND;

    LL_I2C_DisableIT_TX(i2c_);
    LL_I2C_DisableIT_RX(i2c_);
    LL_I2C_EnableIT_NACK(i2c_);
    LL_I2C_EnableIT_ERR(i2c_);

    if (method_ == XferMethod::kDMA && chunk_remaining_ != 0)
    {
        const auto dma = reading_ ? rx_dma_ : tx_dma_;

        LL_I2C_DisableIT_TC(i2c_);
        LL_I2C_DisableIT_STOP(i2c_);
        LL_DMA_DisableChannel(dma.instance, dma.channel);
        LL_DMA_DisableIT_HT(dma.instance, dma.channel);

        llhelper::dma_clear_intr_flags(dma.instance, dma.channel);

        // We skip configuring the DMA by `LL_DMA_ConfigTransfer()` and
        // `LL_DMA_SetPeriphRequest()` since the board initialization code
        // (generated by cubemx) has done that.

        uint32_t periph_addr = LL_I2C_DMA_GetRegAddr(
            i2c_, reading_ ? LL_I2C_DMA_REG_DATA_RECEIVE : LL_I2C_DMA_REG_DATA_TRANSMIT);

        LL_DMA_SetPeriphAddress(dma.instance, dma.channel, periph_addr);
        LL_DMA_SetMemoryAddress(dma.instance, dma.channel,
                                (reinterpret_cast<uintptr_t>(reading_ ? rx_.data() : tx_.data())));

        LL_DMA_SetDataLength(dma.instance, dma.channel, chunk_remaining_);

        LL_DMA_EnableIT_TC(dma.instance, dma.channel);
        LL_DMA_EnableIT_TE(dma.instance, dma.channel);

        dma_active_ = true;
        __DMB();

        LL_DMA_EnableChannel(dma.instance, dma.channel);

        if (reading_)
            LL_I2C_EnableDMAReq_RX(i2c_);
        else
            LL_I2C_EnableDMAReq_TX(i2c_);
    }
    else
    {
        LL_I2C_EnableIT_TC(i2c_);
        LL_I2C_EnableIT_STOP(i2c_);
        if (chunk_remaining_ != 0)
        {
            if (reading_)
                LL_I2C_EnableIT_RX(i2c_);
            else
                LL_I2C_EnableIT_TX(i2c_);
        }
    }

    LL_I2C_HandleTransfer(i2c_, address_, LL_I2C_ADDRSLAVE_7BIT,
                          chunk_remaining_, end_mode, request);
}

void Bus::stop_dma()
{
    LL_I2C_DisableDMAReq_TX(i2c_);
    LL_I2C_DisableDMAReq_RX(i2c_);

    if (!dma_active_)
        return;

    const auto dma = reading_ ? rx_dma_ : tx_dma_;

    LL_DMA_DisableChannel(dma.instance, dma.channel);
    LL_DMA_DisableIT_TC(dma.instance, dma.channel);
    LL_DMA_DisableIT_TE(dma.instance, dma.channel);
    llhelper::dma_clear_intr_flags(dma.instance, dma.channel);
    __DSB();

    dma_active_ = false;
}

void Bus::quiesce(bool reset)
{
    LL_I2C_WriteReg(i2c_, CR1, LL_I2C_ReadReg(i2c_, CR1) & ~kTransferInterrupts);
    stop_dma();

    if (reset)
    {
        // A missing STOP (e.g. held SCL) must not leave an IRQ or DMA accessing
        // the caller's buffers. PE reset preserves timing/filter configuration.
        LL_I2C_Disable(i2c_);
        // Keep PE low for at least three APB cycles, independent of CPU clock.
        (void)LL_I2C_ReadReg(i2c_, CR1);
        (void)LL_I2C_ReadReg(i2c_, CR1);
        (void)LL_I2C_ReadReg(i2c_, CR1);
    }

    LL_I2C_WriteReg(i2c_, CR2, 0);
    LL_I2C_WriteReg(i2c_, ICR, kClearFlags);

    if (LL_I2C_IsActiveFlag_RXNE(i2c_))
        (void)LL_I2C_ReceiveData8(i2c_);

    flush_tx(i2c_);

    if (reset)
        LL_I2C_Enable(i2c_);
}

void Bus::finish_from_isr(Err error, bool reset, BaseType_t& wake)
{
    quiesce(reset);
    result_ = error;
    active_ = false;
    (void)completion_.give_from_isr(wake);
}

bool Bus::handle_error(BaseType_t& wake)
{
    if (!LL_I2C_IsEnabledIT_ERR(i2c_))
        return false;

    // Arbitration loss has precedence: never generate STOP on another master's
    // transaction, even if a bus error is reported in the same ISR snapshot.
    if (LL_I2C_IsActiveFlag_ARLO(i2c_))
        finish_from_isr(Err::kArbitrationLost, false, wake);
    else if (LL_I2C_IsActiveFlag_BERR(i2c_))
        finish_from_isr(Err::kBus, true, wake);
    else if (LL_I2C_IsActiveFlag_OVR(i2c_))
        finish_from_isr(Err::kOverrun, true, wake);
    else if (LL_I2C_IsActiveSMBusFlag_TIMEOUT(i2c_))
        finish_from_isr(Err::kTimeout, true, wake);
    else
        return false;
    return true;
}

void Bus::handle_event(BaseType_t& wake)
{
    if (handle_error(wake))
        return;

    if (LL_I2C_IsEnabledIT_NACK(i2c_) && LL_I2C_IsActiveFlag_NACK(i2c_))
    {
        LL_I2C_ClearFlag_NACK(i2c_);
        result_ = Err::kNack;
        stop_dma();
        LL_I2C_DisableIT_TX(i2c_);
        LL_I2C_DisableIT_RX(i2c_);
        LL_I2C_DisableIT_TC(i2c_);
        // Hardware automatically generates STOP on master NACK, even in RELOAD
        // or SOFTEND. Keep waiting for it before releasing the bus lock.
        LL_I2C_EnableIT_STOP(i2c_);
    }

    // RXNE may coexist with STOPF: consume the final byte before completion.
    if (result_ == Err::kOk && chunk_remaining_ != 0)
    {
        if (reading_ && LL_I2C_IsEnabledIT_RX(i2c_) && LL_I2C_IsActiveFlag_RXNE(i2c_))
        {
            rx_.front() = LL_I2C_ReceiveData8(i2c_);
            rx_ = rx_.subspan(1);
            --chunk_remaining_;
        }
        else if (!reading_ && LL_I2C_IsEnabledIT_TX(i2c_) && LL_I2C_IsActiveFlag_TXIS(i2c_))
        {
            LL_I2C_TransmitData8(i2c_, tx_.front());
            tx_ = tx_.subspan(1);
            --chunk_remaining_;
        }
    }

    if (LL_I2C_IsEnabledIT_STOP(i2c_) && LL_I2C_IsActiveFlag_STOP(i2c_))
    {
        auto error = result_;
        if (error == Err::kOk && (!tx_.empty() || !rx_.empty()))
            error = Err::kUnexpectedStop;
        finish_from_isr(error, false, wake);
        return;
    }
    if (result_ != Err::kOk || !LL_I2C_IsEnabledIT_TC(i2c_))
        return;

    if (LL_I2C_IsActiveFlag_TCR(i2c_))
    {
        if (chunk_remaining_ != 0 || (reading_ ? rx_.empty() : tx_.empty()))
            finish_from_isr(Err::kUnexpectedStop, true, wake);
        else
            start_chunk(LL_I2C_GENERATE_NOSTARTSTOP);
    }
    else if (LL_I2C_IsActiveFlag_TC(i2c_))
    {
        if (chunk_remaining_ != 0 || reading_ || !tx_.empty() || rx_.empty())
            finish_from_isr(Err::kUnexpectedStop, true, wake);
        else
        {
            reading_ = true;
            start_chunk(LL_I2C_GENERATE_RESTART_7BIT_READ);
        }
    }
}

void Bus::on_i2c_event_irq()
{
    BaseType_t wake = pdFALSE;
    {
        const IsrCriticalSection critical;
        if (i2c_ && active_)
            handle_event(wake);
    }
    portYIELD_FROM_ISR(wake);
}

void Bus::on_i2c_error_irq()
{
    BaseType_t wake = pdFALSE;
    {
        const IsrCriticalSection critical;
        if (i2c_ && active_)
            (void)handle_error(wake);
    }
    portYIELD_FROM_ISR(wake);
}

void Bus::on_rx_dma_irq()
{
    handle_dma_irq(true);
}

void Bus::on_tx_dma_irq()
{
    handle_dma_irq(false);
}

void Bus::handle_dma_irq(bool reading)
{
    BaseType_t wake = pdFALSE;
    {
        const IsrCriticalSection critical;
        if (i2c_ && active_ && dma_active_ && reading_ == reading)
        {
            const auto dma = reading ? rx_dma_ : tx_dma_;
            const uint32_t flags = llhelper::dma_get_intr_flags(dma.instance, dma.channel);

            if ((flags & DMA_ISR_TEIF1) && LL_DMA_IsEnabledIT_TE(dma.instance, dma.channel))
            {
                if (!handle_error(wake))
                    finish_from_isr(Err::kDma, true, wake);
            }
            else if ((flags & DMA_ISR_TCIF1) && LL_DMA_IsEnabledIT_TC(dma.instance, dma.channel))
            {
                stop_dma();
                if (reading_)
                    rx_ = rx_.subspan(chunk_remaining_);
                else
                    tx_ = tx_.subspan(chunk_remaining_);
                chunk_remaining_ = 0;
                // These flags are level-triggered: already pending STOP/TCR/TC
                // is serviced as soon as its interrupt is enabled.
                LL_I2C_EnableIT_TC(i2c_);
                LL_I2C_EnableIT_STOP(i2c_);
            }
        }
    }
    portYIELD_FROM_ISR(wake);
}

} // namespace i2c

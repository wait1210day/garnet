// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_UART_H
#define GARNET_UART_H

#include <span>
#include <atomic>
#include <optional>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32g4xx_ll_usart.h"

#include "base/err.h"
#include "base/defs.h"
#include "base/xfer_method.h"
#include "base/panic.h"
#include "base/static_semaphore.h"
#include "base/static_mutex.h"

namespace uart
{

class Transmitter
{
public:
    GARNET_NON_COPYABLE(Transmitter)
    GARNET_NON_MOVABLE(Transmitter)

    Transmitter() = default;
    ~Transmitter() = default;

    class Scoped
    {
    public:
        GARNET_NON_COPYABLE(Scoped)

        ~Scoped()
        {
            if (tx_)
                tx_->mutex_.unlock();
        }

        Scoped(Scoped&& other) noexcept : tx_(other.tx_)
        { other.tx_ = nullptr; }

        Scoped& operator=(Scoped&& other) noexcept
        {
            if (&other != this)
            {
                if (tx_)
                    tx_->mutex_.unlock();
                tx_ = other.tx_;
                other.tx_ = nullptr;
            }
            return *this;
        }

        // Borrows memory `data` during transmitting (accessed by DMA if DMA method
        // is used), and the ownership of `data` is given back to caller when the
        // function returns.
        // It only guarantees that all the `data` has been TRANSFERRED to the USART
        // peripheral, and does NOT guarantee all the data have been spitted out
        //  (TRANSMITTED) on the UART TX wire.
        ErrStatus transmit(std::span<const uint8_t> data, XferMethod method,
                           TickType_t timeout)
        {
            GCHECK(tx_ && "invalid operation");
            return tx_->transmit(data, method, timeout);
        }

        // To make sure all the data have been TRANSMITTED.
        ErrStatus wait_flush(TickType_t timeout)
        {
            GCHECK(tx_ && "invalid operation");
            return tx_->wait_flush(timeout);
        }

    private:
        friend class Transmitter;
        explicit Scoped(Transmitter *tx) : tx_{tx} {}

        Transmitter *tx_;
    };

    struct Config
    {
        USART_TypeDef *uart;
        DMA_TypeDef *dma;
        uint32_t dma_channel;
    };

    ErrStatus init(const Config& config);

    std::optional<Scoped> acquire(TickType_t timeout);

    void on_usart_irq();
    void on_tx_dma_irq();
#define UART_IRQ_HANDLER_IMPL(sym, instance, type) \
    extern "C" void sym() { (instance).on_##type##_irq(); }

private:
    ErrStatus transmit(std::span<const uint8_t> data, XferMethod method, TickType_t timeout);
    ErrStatus transmit_dma(std::span<const uint8_t> data, TickType_t timeout);
    ErrStatus transmit_poll(std::span<const uint8_t> data, TickType_t timeout);

    ErrStatus wait_flush(TickType_t timeout);

    Config config_{};
    StaticMutex mutex_{StaticInitOption::kDefer};
    StaticBinarySemaphore completion_{StaticInitOption::kDefer};
    std::atomic<bool> dma_transfer_error_{false};
};

class Receiver
{
};

} // namespace uart

#endif //GARNET_UART_H

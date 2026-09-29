// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_UART_H
#define GARNET_UART_H

#include <array>
#include <span>
#include <atomic>
#include <optional>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_dma.h"

#include "base/err.h"
#include "base/defs.h"
#include "base/xfer_method.h"
#include "base/panic.h"
#include "base/static_semaphore.h"
#include "base/static_mutex.h"

namespace uart
{

class Port;

struct DmaChannel
{
    DMA_TypeDef *instance{nullptr};
    uint32_t channel{LL_DMA_CHANNEL_1};
};

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
        // Call after transmit() within the same Scoped lock acquisition.
        // Releasing and reacquiring the lock between them is unsupported.
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

    std::optional<Scoped> acquire(TickType_t timeout);

private:
    friend class Port;

    ErrStatus init(USART_TypeDef *uart, DmaChannel dma);
    void on_usart_irq(uint32_t isr, uint32_t cr1, BaseType_t& wake);
    void on_tx_dma_irq(BaseType_t& wake);

    ErrStatus transmit(std::span<const uint8_t> data, XferMethod method, TickType_t timeout);
    ErrStatus transmit_dma(std::span<const uint8_t> data, TickType_t timeout);
    ErrStatus transmit_poll(std::span<const uint8_t> data, TickType_t timeout);

    ErrStatus wait_flush(TickType_t timeout);

    USART_TypeDef *uart_{nullptr};
    DmaChannel dma_{};
    StaticMutex mutex_{StaticInitOption::kDefer};
    StaticBinarySemaphore notifier_{StaticInitOption::kDefer};
    std::atomic<bool> dma_transfer_error_{false};
};

class Receiver
{
public:
    GARNET_NON_COPYABLE(Receiver)
    GARNET_NON_MOVABLE(Receiver)

    Receiver() = default;
    ~Receiver() = default;

    enum class Status : uint8_t
    {
        kOk,
        kTimeout,
        kInvalidArg,
        kOverflow,
        kOverrun,
        kFraming,
        kNoise,
        kParity,
        kDma,
    };

    struct ReadResult
    {
        size_t size;
        Status status;
    };

    class Scoped
    {
    public:
        GARNET_NON_COPYABLE(Scoped)

        ~Scoped()
        {
            if (rx_)
                rx_->mutex_.unlock();
        }

        Scoped(Scoped&& other) noexcept : rx_(other.rx_)
        { other.rx_ = nullptr; }

        Scoped& operator=(Scoped&& other) noexcept
        {
            if (this != &other)
            {
                if (rx_)
                    rx_->mutex_.unlock();
                rx_ = other.rx_;
                other.rx_ = nullptr;
            }
            return *this;
        }

        // Returns available bytes, up to data.size(). On error, data received
        // before the error is discarded; later bytes remain available.
        // A line boundary is determined by the caller, not by USART IDLE.
        ReadResult read_some(std::span<uint8_t> data, TickType_t timeout)
        {
            GCHECK(rx_ && "invalid operation");
            return rx_->read_some(data, timeout);
        }

    private:
        friend class Receiver;
        explicit Scoped(Receiver *rx) : rx_{rx} {}
        Receiver *rx_;
    };

    std::optional<Scoped> acquire(TickType_t timeout);

private:
    friend class Port;
    static constexpr size_t kDmaSize = 256;
    static constexpr size_t kRingSize = 1024;
    static_assert(kRingSize >= kDmaSize);

    ErrStatus init(USART_TypeDef *uart, DmaChannel dma);

    ReadResult read_some(std::span<uint8_t> data, TickType_t timeout);

    void on_usart_irq(uint32_t isr, uint32_t cr1, uint32_t cr3, BaseType_t& wake);
    void on_rx_dma_irq(BaseType_t& wake);

    void sync_dmabuf_to_ringbuf(uint32_t flags, BaseType_t& wake);
    void set_error(Status error, BaseType_t& wake);
    void restart_dma();

    USART_TypeDef *uart_{nullptr};
    DmaChannel dma_{};
    StaticMutex mutex_{StaticInitOption::kDefer};
    StaticBinarySemaphore notifier_{StaticInitOption::kDefer};
    std::array<uint8_t, kDmaSize> dma_buffer_{};
    std::array<uint8_t, kRingSize> ring_{};
    size_t dma_read_pos_{0};
    size_t ring_head_{0};
    size_t ring_tail_{0};
    size_t ring_count_{0};
    Status error_{Status::kOk};
};

// One owner per USART. TX and RX may operate concurrently.
class Port
{
public:
    GARNET_NON_COPYABLE(Port)
    GARNET_NON_MOVABLE(Port)

    Port() = default;
    ~Port() = default;

    struct Config
    {
        USART_TypeDef *uart{nullptr};
        DmaChannel tx_dma{};
        DmaChannel rx_dma{};
    };

    // Board code enables the USART and configures RX DMA as circular,
    // byte-wide, memory-incrementing DMA with a dedicated IRQ channel.
    // Bind one Port to each USART before concurrent use.
    ErrStatus init(const Config& config);
    Transmitter& tx() { return tx_; }
    Receiver& rx() { return rx_; }

    void on_usart_irq();
    void on_tx_dma_irq();
    void on_rx_dma_irq();

private:
    USART_TypeDef *uart_{nullptr};
    Transmitter tx_{};
    Receiver rx_{};
};

#define UART_IRQ_HANDLER_IMPL(sym, instance, type) \
    extern "C" void sym() { (instance).on_##type##_irq(); }

} // namespace uart

#endif //GARNET_UART_H

// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_I2C_BUS_H
#define GARNET_I2C_BUS_H

#include <cstddef>
#include <cstdint>
#include <span>
#include <expected>
#include <optional>

#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_i2c.h"

#include "base/defs.h"
#include "base/err.h"
#include "base/xfer_method.h"
#include "base/static_mutex.h"
#include "base/static_semaphore.h"

namespace i2c
{

enum class Err : uint8_t
{
    kOk = 0,
    kInvalidArg,
    kNotInitialized,
    kInvalidContext,
    kDmaNotConfigured,
    kBusy,
    kTimeout,
    kNack,
    kBus,
    kArbitrationLost,
    kOverrun,
    kDma,
    kUnexpectedStop,
};

const char *get_err_str(Err e);

// Synchronous, master-only, 7-bit I2C bus. No heap allocation or HAL calls.
// One owner per peripheral; do not mix HAL transfers with this driver.
//
// Device addresses are un-shifted 7-bit addresses.
// Task-side bus operations require a Scoped obtained from acquire().
class Bus
{
public:
    Bus() = default;
    // All Scoped objects must be destroyed and IRQ forwarding detached first.
    ~Bus() = default;

    GARNET_NON_COPYABLE(Bus)
    GARNET_NON_MOVABLE(Bus)

    // Owns the bus mutex until destruction; movable, but not copyable.
    // Acquire, use, move and destroy in the same task. Bus must outlive Scoped.
    // Moving transfers ownership; operations on the source triggers panic.
    class Scoped
    {
    public:
        GARNET_NON_COPYABLE(Scoped)

        Scoped(Scoped&& other) noexcept : bus_(other.bus_)
        { other.bus_ = nullptr; }

        ~Scoped() noexcept
        {
            if (bus_)
                bus_->bus_lock_.unlock();
        }

        Scoped& operator=(Scoped&& other) noexcept
        {
            if (this != &other)
            {
                if (bus_)
                    bus_->bus_lock_.unlock();

                bus_ = other.bus_;
                other.bus_ = nullptr;
            }
            return *this;
        }

        [[nodiscard]] Err probe(uint16_t dev_addr, TickType_t timeout)
        {
            GCHECK(bus_ && "invalid operation");
            return bus_->probe(dev_addr, timeout);
        }

        // Buffers are borrowed until return, including abort cleanup.
        // DMA buffers must be DMA-accessible (not the CCM core alias).
        // On failure RX may be partially updated. Empty TX probes the address;
        // empty RX is rejected. Each timeout covers only this transfer, not
        // acquisition or earlier operations. Zero never starts a transfer.
        [[nodiscard]] Err transmit(uint16_t dev_addr, std::span<const uint8_t> txbuf,
                                   XferMethod method, TickType_t xfer_timeout)
        {
            GCHECK(bus_ && "invalid operation");
            return bus_->transmit(dev_addr, txbuf, method, xfer_timeout);
        }

        [[nodiscard]] Err receive(uint16_t dev_addr, std::span<uint8_t> rxbuf,
                                  XferMethod method, TickType_t xfer_timeout)
        {
            GCHECK(bus_ && "invalid operation");
            return bus_->receive(dev_addr, rxbuf, method, xfer_timeout);
        }

        // Nonempty buffers: write + repeated START + read + STOP, one timeout.
        [[nodiscard]] Err transmit_receive(uint16_t dev_addr,
                                           std::span<const uint8_t> txbuf,
                                           std::span<uint8_t> rxbuf,
                                           XferMethod method, TickType_t xfer_timeout)
        {
            GCHECK(bus_ && "invalid operation");
            return bus_->transmit_receive(dev_addr, txbuf, rxbuf, method, xfer_timeout);
        }

    private:
        friend class Bus;
        // Adopt a mutex already acquired by Bus::acquire().
        explicit Scoped(Bus& bus) noexcept : bus_(&bus) {}

        Bus *bus_;
    };

    struct DmaChannel
    {
        DMA_TypeDef *instance{nullptr};
        uint32_t channel{LL_DMA_CHANNEL_1};
    };

    struct Config
    {
        I2C_TypeDef *i2c{nullptr};
        DmaChannel tx_dma{};
        DmaChannel rx_dma{};
    };

    // Bind once, before concurrent use. Board code configures GPIO, clocks,
    // TIMINGR, filters and enables I2C first.
    //
    // Use ordinary I2C mode with slave address matching disabled.
    // For DMA, also enable DMA/DMAMUX clocks and reserve distinct channels
    // with DMAMUX synchronization/event generation disabled.
    // Board code configures normal-mode byte transfers, memory increment,
    // fixed peripheral addresses and the matching DMAMUX requests.
    [[nodiscard]] ErrStatus init(const Config& config);

    // Task context only. Nullopt if uninitialized or the mutex wait fails.
    // Zero tries once without blocking; portMAX_DELAY waits indefinitely.
    // The mutex is nonrecursive: reuse Scoped for a sequence of operations.
    [[nodiscard]] std::optional<Scoped> acquire(TickType_t timeout);

    // Forward the peripheral's EV/ER and both configured DMA channel IRQs here.
    // FreeRTOS calls are involved in these handlers.
    // The peripheral that issued the IRQ must match, thus, the caller is responsible
    // to route the correct IRQ to us.
    void on_i2c_event_irq();
    void on_i2c_error_irq();
    void on_rx_dma_irq();
    void on_tx_dma_irq();

#define I2C_BUS_IRQ_HANDLER_IMPL(sym, instance, type) \
    extern "C" void sym() { (instance).on_##type##_irq(); }

private:
    // Called only while Scoped owns bus_lock_. IRQ handlers do not take it.
    [[nodiscard]] Err probe(uint16_t dev_addr, TickType_t timeout);
    [[nodiscard]] Err transmit(uint16_t dev_addr, std::span<const uint8_t> txbuf,
                                    XferMethod method, TickType_t xfer_timeout);

    [[nodiscard]] Err receive(uint16_t dev_addr, std::span<uint8_t> rxbuf,
                                   XferMethod method, TickType_t xfer_timeout);

    [[nodiscard]] Err transmit_receive(uint16_t dev_addr,
                                            std::span<const uint8_t> txbuf,
                                            std::span<uint8_t> rxbuf,
                                            XferMethod method, TickType_t xfer_timeout);

    Err transfer(uint16_t dev_addr, std::span<const uint8_t> txbuf,
                      std::span<uint8_t> rxbuf, XferMethod method,
                      TickType_t xfer_timeout);
    void start_chunk(uint32_t request);
    void stop_dma();
    void quiesce(bool reset);
    void finish_from_isr(Err error, bool reset, BaseType_t& wake);
    void handle_event(BaseType_t& wake);
    bool handle_error(BaseType_t& wake);
    void handle_dma_irq(bool reading);

    I2C_TypeDef *i2c_{nullptr};
    StaticMutex bus_lock_{StaticInitOption::kDefer};
    StaticBinarySemaphore completion_{StaticInitOption::kDefer};
    DmaChannel tx_dma_{};
    DmaChannel rx_dma_{};

    std::span<const uint8_t> tx_{};
    std::span<uint8_t> rx_{};
    uint16_t address_{0};
    uint16_t chunk_remaining_{0};
    XferMethod method_{XferMethod::kIRQ};
    Err result_{Err::kOk};
    bool reading_{false};
    bool active_{false};
    bool dma_active_{false};
};

} // namespace i2c

#endif //GARNET_I2C_BUS_H

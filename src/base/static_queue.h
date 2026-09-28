// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_STATIC_QUEUE_H
#define GARNET_STATIC_QUEUE_H

#include <cstdint>
#include <cstddef>

#include <type_traits>
#include <optional>

#include "FreeRTOS.h"
#include "queue.h"

#include "base/defs.h"
#include "base/panic.h"
#include "base/static_resource.h"


template<typename T, size_t QUEUE_LENGTH>
class StaticQueue
{
    static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
    static_assert(QUEUE_LENGTH > 0, "invalid queue length, must be greater than 0");

public:
    GARNET_NON_COPYABLE(StaticQueue)
    GARNET_NON_MOVABLE(StaticQueue)

    // NOLINTNEXTLINE
    explicit  StaticQueue(StaticInitOption init_option)
    {
        if (init_option == StaticInitOption::kImm)
            init();
    }

    ~StaticQueue()
    {
        if (queue_)
            vQueueDelete(queue_);
    }

    void init()
    {
        if (queue_)
            return;

        queue_ = xQueueCreateStatic(QUEUE_LENGTH, sizeof(T),
            store_, &queue_data_);
        GCHECK(queue_ && "failed to create static queue");
    }

    bool send(const T& item, TickType_t ticks)
    {
        GCHECK(queue_ && "invalid operation");
        return xQueueSend(queue_, &item, ticks) == pdTRUE;
    }

    bool receive(T& item, TickType_t ticks)
    {
        GCHECK(queue_ && "invalid operation");
        return xQueueReceive(queue_, &item, ticks) == pdTRUE;
    }

    [[nodiscard]] std::optional<T> receive(TickType_t ticks)
        requires(std::is_default_constructible_v<T>)
    {
        std::optional<T> result{std::in_place};
        if (receive(*result, ticks))
            return result;
        return std::nullopt;
    }

    bool send_from_isr(const T& item, BaseType_t& higher_pri_task_woken)
    {
        GCHECK(queue_ && "invalid operation");
        return xQueueSendFromISR(queue_, &item, &higher_pri_task_woken) == pdTRUE;
    }

    bool receive_from_isr(T& item, BaseType_t& higher_pri_task_woken)
    {
        GCHECK(queue_ && "invalid operation");
        return xQueueReceiveFromISR(queue_, &item, &higher_pri_task_woken) == pdTRUE;
    }

    [[nodiscard]] std::optional<T> receive_from_isr(BaseType_t& higher_pri_task_woken)
        requires(std::is_default_constructible_v<T>)
    {
        std::optional<T> result{std::in_place};
        if (receive_from_isr(*result, higher_pri_task_woken))
            return result;
        return std::nullopt;
    }

    [[nodiscard]] size_t messages_waiting() const
    {
        GCHECK(queue_ && "invalid operation");
        return static_cast<size_t>(uxQueueMessagesWaiting(queue_));
    }

    [[nodiscard]] size_t spaces_available() const
    {
        GCHECK(queue_ && "invalid operation");
        return uxQueueSpacesAvailable(queue_);
    }

    void reset()
    {
        GCHECK(queue_ && "invalid operation");
        xQueueReset(queue_);
    }

    [[nodiscard]] QueueHandle_t native_handle() const
    {
        return queue_;
    }

private:
    QueueHandle_t queue_{nullptr};
    StaticQueue_t queue_data_{};

    // Uninitialized, intentionally.
    alignas(alignof(T)) uint8_t store_[QUEUE_LENGTH * sizeof(T)];
};

#endif //GARNET_STATIC_QUEUE_H

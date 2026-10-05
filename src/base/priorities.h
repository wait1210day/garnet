// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_PRIORITIES_H
#define GARNET_PRIORITIES_H

#include <cstdint>

#include "FreeRTOS.h"

// A unified priority registry that combines NVIC priority and
// FreeRTOS priority.
enum class Priority : uint16_t
{
    kIdle = 0,

    kFumi = 5,
    kParamSet = 5,

    kAdcOptimizer = 10,
    kJanus = 15,
    kInitAndSpawn = 18,

    kTaskBoundary = configMAX_PRIORITIES,
    kIrqBoundary = 100,
    // IRQ priorities defined here are abstract values ONLY represents
    // their relative priority, and the numerical value does NOT have any
    // relationship with (and cannot be converted to) the register value of NVIC.

    kCtrloopIrq = 200
};
static_assert(Priority::kIrqBoundary > Priority::kTaskBoundary,
    "overlap between irq priorities and task priorities");

template<Priority PRI>
struct PriorityTraits
{
    constexpr static bool is_irq_handler()
    {
        return PRI > Priority::kIrqBoundary;
    }

    constexpr static bool is_task()
    {
        return PRI < Priority::kTaskBoundary;
    }

    constexpr static UBaseType_t to_freertos()
    {
        static_assert(is_task(), "not a valid freertos priority");
        return static_cast<UBaseType_t>(PRI);
    }
};

#endif //GARNET_PRIORITIES_H

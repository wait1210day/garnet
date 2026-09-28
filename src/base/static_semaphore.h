// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_STATIC_SEMAPHORE_H
#define GARNET_STATIC_SEMAPHORE_H

#include "FreeRTOS.h"
#include "semphr.h"

#include "base/defs.h"
#include "base/panic.h"
#include "base/static_resource.h"

// Statically allocated binary semaphore, initially empty.
// Initialize before concurrent use; all users must stop before destruction.
class StaticBinarySemaphore
{
public:
    GARNET_NON_COPYABLE(StaticBinarySemaphore)
    GARNET_NON_MOVABLE(StaticBinarySemaphore)

    // NOLINTNEXTLINE
    explicit StaticBinarySemaphore(StaticInitOption init_option)
    {
        if (init_option == StaticInitOption::kImm)
            init();
    }

    ~StaticBinarySemaphore()
    {
        if (semaphore_)
            vSemaphoreDelete(semaphore_);
    }

    void init()
    {
        if (semaphore_)
            return;

        semaphore_ = xSemaphoreCreateBinaryStatic(&semaphore_data_);
        GCHECK(semaphore_ && "failed to create static binary semaphore");
    }

    bool take(TickType_t ticks)
    {
        GCHECK(semaphore_ && "invalid operation");
        return xSemaphoreTake(semaphore_, ticks) == pdTRUE;
    }

    bool give()
    {
        GCHECK(semaphore_ && "invalid operation");
        return xSemaphoreGive(semaphore_) == pdTRUE;
    }

    bool take_from_isr(BaseType_t& higher_pri_task_woken)
    {
        GCHECK(semaphore_ && "invalid operation");
        return xSemaphoreTakeFromISR(semaphore_, &higher_pri_task_woken) == pdTRUE;
    }

    bool give_from_isr(BaseType_t& higher_pri_task_woken)
    {
        GCHECK(semaphore_ && "invalid operation");
        return xSemaphoreGiveFromISR(semaphore_, &higher_pri_task_woken) == pdTRUE;
    }

    [[nodiscard]] SemaphoreHandle_t native_handle() const
    {
        return semaphore_;
    }

private:
    SemaphoreHandle_t semaphore_{nullptr};
    StaticSemaphore_t semaphore_data_;
};

#endif //GARNET_STATIC_SEMAPHORE_H

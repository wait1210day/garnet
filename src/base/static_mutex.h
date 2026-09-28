// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_STATIC_MUTEX_H
#define GARNET_STATIC_MUTEX_H

#include "FreeRTOS.h"
#include "mutex"
#include "semphr.h"

#include "base/panic.h"
#include "base/static_resource.h"

// A wrapper of FreeRTOS's mutex that compatible with cpp lock guard
class StaticMutex
{
public:
    GARNET_NON_COPYABLE(StaticMutex)
    GARNET_NON_MOVABLE(StaticMutex)

    // NOLINTNEXTLINE
    explicit StaticMutex(StaticInitOption init_option)
    {
        if (init_option == StaticInitOption::kImm)
            init();
    }

    ~StaticMutex()
    {
        if (lock_)
            vSemaphoreDelete(lock_);
    }

    void init()
    {
        if (lock_)
            return;
        lock_ = xSemaphoreCreateMutexStatic(&lock_data_);
        GCHECK(lock_ && "failed to create static mutex semaphore");
    }

    void lock() const
    {
        GCHECK(lock_ && "mutex has not been inited yet");
        xSemaphoreTake(lock_, portMAX_DELAY);
    }

    [[nodiscard]] bool try_lock_for(TickType_t timeout) const
    {
        GCHECK(lock_ && "mutex has not been inited yet");
        return xSemaphoreTake(lock_, timeout) == pdTRUE;
    }

    [[nodiscard]] bool try_lock() const
    {
        GCHECK(lock_ && "mutex has not been inited yet");
        return xSemaphoreTake(lock_, 0) == pdTRUE;
    }

    void unlock() const
    {
        GCHECK(lock_ && "mutex has not been inited yet");
        xSemaphoreGive(lock_);
    }

    [[nodiscard]] SemaphoreHandle_t native_handle() const
    {
        return lock_;
    }

private:
    StaticSemaphore_t lock_data_;
    SemaphoreHandle_t lock_{nullptr};
};

#endif //GARNET_STATIC_MUTEX_H

// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_CRITICAL_SECTION_H
#define GARNET_CRITICAL_SECTION_H

#include "FreeRTOS.h"
#include "task.h"

#include "base/defs.h"

// Scoped FreeRTOS critical section for task context. Supports nesting.
// Keep the scope short and do not call blocking APIs while holding it.
class TaskCriticalSection
{
public:
    GARNET_NON_COPYABLE(TaskCriticalSection)
    GARNET_NON_MOVABLE(TaskCriticalSection)

    TaskCriticalSection()
    {
        taskENTER_CRITICAL();
    }

    ~TaskCriticalSection()
    {
        taskEXIT_CRITICAL();
    }
};

// Scoped critical section for ISRs at FreeRTOS syscall-safe priorities.
// Restores the previous interrupt mask on exit; higher-priority IRQs remain
// unmasked and must not access the protected state.
class IsrCriticalSection
{
public:
    GARNET_NON_COPYABLE(IsrCriticalSection)
    GARNET_NON_MOVABLE(IsrCriticalSection)

    IsrCriticalSection() : mask_(taskENTER_CRITICAL_FROM_ISR())
    {
    }

    ~IsrCriticalSection()
    {
        taskEXIT_CRITICAL_FROM_ISR(mask_);
    }

private:
    UBaseType_t mask_;
};

#endif //GARNET_CRITICAL_SECTION_H

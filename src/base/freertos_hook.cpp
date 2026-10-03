// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdint>
#include <cstddef>

#include "stm32g4xx_ll_cortex.h"

#include "FreeRTOS.h"
#include "task.h"

#include "base/panic.h"

extern "C" {

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

#if (configUSE_TIMERS == 1)

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

#endif // configUSE_TIMERS

void panic_os_c_trampoline(const char *file, uint32_t line, const char *expr)
{
    panic::panic_at(expr, file, "<unknown function>", line);
}

void mpu_update_stack_guard(void *stack_base)
{
    auto base_addr = reinterpret_cast<uint32_t>(stack_base);

    // The MPU requires the region base address to be aligned to its size (32 bytes here).
    // However, the FreeRTOS ARM_CM4F port hardcodes `portBYTE_ALIGNMENT` to 8 bytes,
    // so dynamically allocated task stack buffers are only guaranteed to be 8-byte aligned.
    //
    // Because the Cortex-M stack grows downwards, rounding `raw_base` UP to the next
    // 32-byte boundary places the 32-byte NO_ACCESS guard region inside the bottom of
    // the task's allocated stack buffer. Any stack overflow will hit the upper edge
    // of this barrier first, triggering a MemManage Fault before corrupting memory
    // below `raw_base`.
    //
    // This wastes at most 24 bytes of effective task stack space.
    base_addr = (base_addr + 31u) & ~31u;

    LL_MPU_ConfigRegion(
        LL_MPU_REGION_NUMBER7,
        0x00,
        // stack grows down on cortex-m, so we set the guard region at
        // the start of stack buffer (stack base address).
        base_addr,
        LL_MPU_INSTRUCTION_ACCESS_DISABLE
        // FreeRTOS will read the full stack to determine the free stack space in
        // `uxTaskGetStackHighWaterMark()`. We allow it, so PRIV_RO.
        | LL_MPU_REGION_PRIV_RO
        | LL_MPU_ACCESS_NOT_SHAREABLE
        | LL_MPU_ACCESS_NOT_CACHEABLE
        | LL_MPU_ACCESS_NOT_BUFFERABLE
        | LL_MPU_REGION_SIZE_32B
    );

    __DSB();
    __ISB();
}

} // extern "C"

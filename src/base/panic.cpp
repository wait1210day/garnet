// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstring>
#include <cstdio>

#include "stm32g4xx_ll_cortex.h"

#include "FreeRTOS.h"
#include "task.h"

#include "base/panic.h"
#include "base/panic_init.h"

namespace panic
{

namespace
{

void panic_uart_transmit(USART_TypeDef *uart, std::string_view v)
{
    if (v.empty())
        return;

    for (char c : v)
    {
        while (!LL_USART_IsActiveFlag_TXE_TXFNF(uart))
            __NOP();

        LL_USART_TransmitData8(uart, static_cast<uint8_t>(c));
    }

    // wait for the completion of transfer
    while (!LL_USART_IsActiveFlag_TC(uart))
        __NOP();
}

void panic_uart_transmit_u32(USART_TypeDef *uart, uint32_t val)
{
    char buf[10];
    int len = 0;

    do {
        buf[len++] = static_cast<char>('0' + (val % 10));
        val /= 10;
    } while (val > 0);

    for (int i = len - 1; i >= 0; --i)
    {
        while (!LL_USART_IsActiveFlag_TXE_TXFNF(uart))
            __NOP();

        LL_USART_TransmitData8(uart, buf[i]);
    }

    // wait for the completion of transfer
    while (!LL_USART_IsActiveFlag_TC(uart))
        __NOP();
}

void panic_uart_transmit_hex32(USART_TypeDef *uart, uint32_t val)
{
    static constexpr char kHexDigits[] = "0123456789ABCDEF";
    char buf[11];
    buf[0] = '0';
    buf[1] = 'x';
    for (int i = 7; i >= 0; --i)
    {
        buf[2 + (7 - i)] = kHexDigits[(val >> (i * 4)) & 0x0F];
    }
    buf[10] = '\0';
    panic_uart_transmit(uart, std::string_view(buf, 10));
}

void panic_blink_led(const Config& config)
{
    // Turn off the LED first
    if (config.led_active_low)
        LL_GPIO_SetOutputPin(config.led_gpio, config.led_gpio_pin);
    else
        LL_GPIO_ResetOutputPin(config.led_gpio, config.led_gpio_pin);

    // then blinking
    for (int32_t i = 0; i < 10; i++)
    {
        LL_GPIO_TogglePin(config.led_gpio, config.led_gpio_pin);
        for (volatile int32_t j = 0; j < 2000'000; j = j + 1)
            __NOP();
    }
}

void panic_try_raise_breakpoint()
{
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
        __BKPT(0);
}

struct Resources
{
    struct CallbackSlot
    {
        FailSafeCallback *pfn;
        void *userdata;
    };

    bool ready{false};
    bool in_panic{false};
    Config config{};
    CallbackSlot callback_slots[kNbCallbackSlots]{};
};

Resources g_res;
char g_concat_buf[128];

void panic_enter_safe_state()
{
    __disable_irq();

    if (g_res.in_panic)
    {
        // double assertion failure
        panic_try_raise_breakpoint();
        while (true) {}
    }

    g_res.in_panic = true;

    for (const auto& slot : g_res.callback_slots)
    {
        if (slot.pfn)
            slot.pfn(slot.userdata);
    }
}

[[noreturn]] void panic_enter_final_state()
{
    if (g_res.ready)
    {
        panic_uart_transmit(g_res.config.uart, "panic: system will reset...\r\n");
        panic_blink_led(g_res.config);

        panic_try_raise_breakpoint();

        NVIC_SystemReset();
    }

    while (true)
        __NOP();
}

} // namespace anonymous

void init_panic_resources(const Config& config)
{
    if (g_res.ready)
        return;

    g_res.config = config;
    g_res.ready = true;
}

void append_fail_safe_callback(FailSafeCallback *callback, void *userdata)
{
    if (!g_res.ready)
    {
        panic_try_raise_breakpoint();
        while (true) {}
    }

    for (Resources::CallbackSlot& slot : g_res.callback_slots)
    {
        if (!slot.pfn)
        {
            slot.pfn = callback;
            slot.userdata = userdata;
            return;
        }
    }
    GCHECK(false && "no more free callback slots");
}

void panic_at(const char *msg, const char *file, const char *func, uint32_t line)
{
    panic_enter_safe_state();
    if (g_res.ready)
    {
        auto *uart = g_res.config.uart;

        panic_uart_transmit(uart, "panic: ");
        panic_uart_transmit(uart, msg);
        panic_uart_transmit(uart, "\r\n");

        panic_uart_transmit(uart, "  in ");
        panic_uart_transmit(uart, func);
        panic_uart_transmit(uart, " at ");
        panic_uart_transmit(uart, file);
        panic_uart_transmit(uart, ":");
        panic_uart_transmit_u32(uart, line);
        panic_uart_transmit(uart, "\r\n");
    }
    panic_enter_final_state();
}

void panic(const char *msg, std::source_location loc)
{
    panic_at(msg, loc.file_name(), loc.function_name(), loc.line());
}

void panic_with_errstatus(ErrStatus err, std::string_view info,
                          std::source_location loc)
{
    const std::string_view err_str = get_error_str(err);

    memset(g_concat_buf, 0, sizeof(g_concat_buf));
    char *curr = g_concat_buf;
    // -1 to prevent '\0' at the tail being overwritten
    const char *end = g_concat_buf + sizeof(g_concat_buf) - 1;

    auto append = [&](std::string_view s) {
        const size_t remaining = end - curr;
        size_t need_copy_bytes = s.size();
        if (remaining < need_copy_bytes)
            need_copy_bytes = remaining;

        if (need_copy_bytes > 0)
        {
            memcpy(curr, s.data(), need_copy_bytes);
            curr += need_copy_bytes;
        }
    };

    append(info);
    append(" (");
    append(err_str);
    append(")");

    panic(g_concat_buf, loc);
}

void mpu_stack_guard_init()
{
    // Enable MemManage Fault exception.
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

    LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);

    __DSB();
    __ISB();
}

// MPU protection exception.
// `used` attribute is necessary to avoid LTO optimizer eliminates the symbol
// because it cannot see the reference in the inlined assembly of `MemManage_Handler()`.
extern "C" [[gnu::used]] void mm_fault_handler(const uint32_t *stack_frame, uint32_t exc_ret)
{
    panic_enter_safe_state();

    struct MmFaultSavedFrameEntry
    {
        const char *regname;
        uint32_t frame_idx;
    };
    constexpr static MmFaultSavedFrameEntry mm_frame_entries[] {
        { "r0", 0 },
        { "r1", 1 },
        { "r2", 2 },
        { "r3", 3 },
        { "r12", 4 },
        { "lr", 5 },
        { "pc", 6 },
        { "psr", 7 }
    };

    uint32_t cfsr = SCB->CFSR;
    auto mmfsr = static_cast<uint8_t>(cfsr & 0xff);
    bool mmfar_valid = (mmfsr & SCB_CFSR_MMARVALID_Msk) != 0;
    uint32_t fault_addr = mmfar_valid ? SCB->MMFAR : 0;

    const char *task_name = "<none>";
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        if (TaskHandle_t task = xTaskGetCurrentTaskHandle())
            task_name = pcTaskGetName(task);
    }
    if (!task_name)
        task_name = "<failed to get>";
    else if (task_name[0] == '\0')
        task_name = "<empty>";

    if (g_res.ready)
    {
        auto *uart = g_res.config.uart;

        panic_uart_transmit(uart, "panic: memory protection exception\r\n");
        panic_uart_transmit(uart, "  task: ");
        panic_uart_transmit(uart, task_name);
        panic_uart_transmit(uart, "\r\n");
        panic_uart_transmit(uart, "  accessed addr: ");
        if (mmfar_valid)
        {
            panic_uart_transmit_hex32(uart, fault_addr);
            panic_uart_transmit(uart, "\r\n");
        }
        else
        {
            panic_uart_transmit(uart, "<unknown>\r\n");
        }

        panic_uart_transmit(uart, "  mmfsr: ");
        if (mmfsr & 0x02)
            panic_uart_transmit(uart, "DACCVIOL (data access violation)\r\n");
        if (mmfsr & 0x01)
            panic_uart_transmit(uart, "IACCVIOL (instruction fetch violation)\r\n");
        if (mmfsr & 0x10)
            panic_uart_transmit(uart, "MSTKERR (stacking error)\r\n");

        panic_uart_transmit(uart, "  saved context:\r\n");
        for (const auto& entry : mm_frame_entries)
        {
            panic_uart_transmit(uart, "    ");
            panic_uart_transmit(uart, entry.regname);
            panic_uart_transmit(uart, " = ");
            panic_uart_transmit_hex32(uart, stack_frame[entry.frame_idx]);
            panic_uart_transmit(uart, "\r\n");
        }
    }

    panic_enter_final_state();
}

extern "C" [[gnu::naked]] void MemManage_Handler()
{
    __asm volatile(R"(
        tst lr, #4
        ite eq
        mrseq r0, msp
        mrsne r0, psp
        mov r1, lr
        b mm_fault_handler
    )");
}

} // namespace panic

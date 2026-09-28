// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstring>
#include <cstdio>

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

char g_concat_buf[128];

} // namespace anonymous

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

void panic(std::string_view msg, std::source_location loc)
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

    panic_try_raise_breakpoint();

    if (g_res.ready)
    {
        auto *uart = g_res.config.uart;

        panic_uart_transmit(uart, "panic: ");
        panic_uart_transmit(uart, msg);
        panic_uart_transmit(uart, "\r\n");

        panic_uart_transmit(uart, "  in ");
        panic_uart_transmit(uart, loc.function_name());
        panic_uart_transmit(uart, " at ");
        panic_uart_transmit(uart, loc.file_name());
        panic_uart_transmit(uart, ":");
        panic_uart_transmit_u32(uart, loc.line());
        panic_uart_transmit(uart, "\r\n");

        panic_uart_transmit(uart, "panic: system will reset...\r\n");
        panic_blink_led(g_res.config);

        NVIC_SystemReset();
    }

    while (true)
        __NOP();
}

void panic_with_errstatus(ErrStatus err, std::string_view info,
                          std::source_location loc)
{
    const std::string_view err_str = get_error_str(err);

    char *curr = g_concat_buf;
    const char *end = g_concat_buf + sizeof(g_concat_buf);

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

    panic({g_concat_buf, static_cast<size_t>(curr - g_concat_buf)}, loc);
}

} // namespace panic

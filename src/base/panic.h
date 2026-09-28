// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_PANIC_H
#define GARNET_PANIC_H

#include <string_view>
#include <source_location>

#include "base/err.h"

namespace panic
{

constexpr size_t kNbCallbackSlots = 8;

using FailSafeCallback = void(void*);

#define PANIC_ENTRY_ATTR __attribute__((noreturn, naked))

// Fail-safe callbacks will be called when a panic happens, after disabling
// the interrupts, before printing debug information.
// Here you have chance to re-configure GPIOs and peripherals to make them
// in a safe state (e.g. disable the board's power gates, trip PWMs, ...).
//
// The last appended is the first called. `kNbCallbackSlots` available slots,
// more than this will cause a panic.
//
// Double panic will freeze the system (infinite loop) immediately.
void append_fail_safe_callback(FailSafeCallback *callback, void *userdata);

[[noreturn]] void panic(
    std::string_view msg, std::source_location loc = std::source_location::current());

[[noreturn]] inline void unreachable_code(
    const std::source_location loc = std::source_location::current())
{
    panic("unreachable code", loc);
}

[[noreturn]] void panic_with_errstatus(
    ErrStatus err, std::string_view info,
    std::source_location loc = std::source_location::current());

} // namespace panic

#define GCHECK(expr)                                    \
    do {                                                \
        if (!(expr)) [[unlikely]] {                     \
            ::panic::panic("assertion failed: " #expr); \
        }                                               \
    } while (0)

#define GCHECK_OK(err, info)                            \
    do {                                                \
        if (err != ErrStatus::kOk) [[unlikely]] {       \
            ::panic::panic_with_errstatus(err, info);   \
        }                                               \
    } while (0)

#endif //GARNET_PANIC_H

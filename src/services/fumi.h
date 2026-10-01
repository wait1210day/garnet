// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_FUMI_H
#define GARNET_FUMI_H

#include <cstdio>
#include <cstdarg>
#include <atomic>

#include "stm32g4xx_hal.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "base/err.h"
#include "base/static_resource.h"
#include "base/static_queue.h"
#include "drivers/uart.h"

namespace fumi
{

enum class Level : uint8_t
{
    kDebug = 0,
#define FUMI_LEVEL_DEBUG 0
    kInfo,
#define FUMI_LEVEL_INFO 1
    kWarn,
#define FUMI_LEVEL_WARN 2
    kError
#define FUMI_LEVEL_ERROR 3
};

#ifndef FUMI_COMPILE_LEVEL
#define FUMI_COMPILE_LEVEL  FUMI_LEVEL_DEBUG
#endif // FUMI_COMPILE_LEVEL

constexpr static Level kDefaultLevel = Level::kDebug;
constexpr static uint32_t kMsgMaxLen = 256;
constexpr static uint32_t kNbSlots = 16;
constexpr static int kTagMaxLen = 24;

// FreeRTOS task stack depth in `StackType_t` unit, not bytes
constexpr static uint32_t kTaskStackDepth = 384;
// 56 bytes is enough for about ~12 arguments
constexpr static uint32_t kArgsCapacity = 56;

constexpr static TickType_t kNoSlotTimeout = pdMS_TO_TICKS(0);

struct Stats
{
    uint32_t sent;
    uint32_t uart_tx_error;
    uint32_t drop_no_free_slot;
    uint32_t drop_in_isr;
    uint32_t drop_unknown;
};

struct Message;
using FormatDispatcher = size_t(const void *args_store, char *out_buf, size_t max_len);

struct Message
{
    Level level;
    uint32_t timestamp;
    const char *tag;
    FormatDispatcher *format_fn;
    // Stores a `Payload<...>` object
    alignas(8) uint8_t args_store[kArgsCapacity];
};

template<typename...Args>
struct Payload
{
    const char *fmt;
    std::tuple<std::decay_t<Args>...> args;
};

char get_level_char(Level level);

template<typename...Args>
size_t format_dispatcher(const void *args_store, char *out_buf, size_t max_len)
{
    using Payload = Payload<Args...>;
    const auto& payload = *static_cast<const Payload*>(args_store);

    if constexpr (sizeof...(Args) == 0)
    {
        return snprintf(out_buf, max_len, "%s", payload.fmt);
    }
    else
    {
        return std::apply([&](const auto&...args) {
            return snprintf(out_buf, max_len, payload.fmt, args...);
        }, payload.args);
    }
}


class Logger
{
public:

    struct Config
    {
        uart::Transmitter *tx;
        uint16_t task_priority;
    };

    ErrStatus init(const Config& config);

    void set_level(Level level);

    // Note: `args` are stored in a buffer then submitted to the logging task.
    // If a string pointer ("%s") is contained the arguments pack, it must be
    // valid until the log is formatted by the task asynchronously.
    // A suggested way to avoid misuse is NEVER pass non-static %s strings.
    // A pointer with %p is always safe since the pointer will not be dereferenced.
    template<typename...Args>
    void log(Level level, const char *tag, const char *fmt, Args&&...args)
    {
        if (!ready_ || level < level_.load(std::memory_order_relaxed))
            return;

        using Payload = Payload<Args...>;
        static_assert(sizeof(Payload) <= kArgsCapacity,
                      "log arguments exceed slot storage capacity");

        Payload payload{fmt, std::make_tuple(std::forward<Args>(args)...)};
        submit_log(level, tag, &format_dispatcher<Args...>, &payload, sizeof(Payload));
    }

    // Note: approx. stats, since the data may be updated when copying
    // the structure.
    void get_stats_approx(Stats& out) const;

private:
    static_assert(std::atomic<uint8_t>::is_always_lock_free &&
        "requires lock-free atomic implementation");

    void submit_log(Level level, const char *tag, FormatDispatcher *format_fn,
                    const void *payload, size_t payload_size);

    [[noreturn]] void task_routine();

    bool ready_{false};
    std::atomic<Level> level_{kDefaultLevel};
    uart::Transmitter *uart_tx_{nullptr};

    StaticQueue<uint8_t, kNbSlots> q_ready_slot_{StaticInitOption::kDefer};
    StaticQueue<uint8_t, kNbSlots> q_free_slot_{StaticInitOption::kDefer};

    TaskHandle_t task_{nullptr};
    StaticTask_t task_data_{};
    StackType_t task_stack_[kTaskStackDepth]{};

    Stats stats_{};
    Message slots_[kNbSlots]{};
    alignas(4) char format_buf_[kMsgMaxLen]{};
};

Logger& get_default_instance();

} // namespace fumi

#if FUMI_LEVEL_DEBUG >= FUMI_COMPILE_LEVEL
#define FUMI_LOGD(tag, fmt, ...) \
    do { \
        if (false) { printf(fmt, ##__VA_ARGS__); } \
        fumi::get_default_instance().log(fumi::Level::kDebug, tag, fmt, ##__VA_ARGS__); \
    } while (0)
#else
#define FUMI_LOGD(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_DEBUG

#if FUMI_LEVEL_INFO >= FUMI_COMPILE_LEVEL
#define FUMI_LOGI(tag, fmt, ...) \
    do { \
        if (false) { printf(fmt, ##__VA_ARGS__); } \
        fumi::get_default_instance().log(fumi::Level::kInfo, tag, fmt, ##__VA_ARGS__); \
    } while (0)
#else
#define FUMI_LOGI(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_INFO

#if FUMI_LEVEL_WARN >= FUMI_COMPILE_LEVEL
#define FUMI_LOGW(tag, fmt, ...) \
    do { \
        if (false) { printf(fmt, ##__VA_ARGS__); } \
        fumi::get_default_instance().log(fumi::Level::kWarn, tag, fmt, ##__VA_ARGS__); \
    } while (0)
#else
#define FUMI_LOGW(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_WARN

#if FUMI_LEVEL_ERROR >= FUMI_COMPILE_LEVEL
#define FUMI_LOGE(tag, fmt, ...) \
    do { \
        if (false) { printf(fmt, ##__VA_ARGS__); } \
        fumi::get_default_instance().log(fumi::Level::kError, tag, fmt, ##__VA_ARGS__); \
    } while (0)

#else
#define FUMI_LOGE(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_ERROR

#endif //GARNET_FUMI_H

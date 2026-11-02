// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_FUMI_H
#define GARNET_FUMI_H

#include <cstdio>
#include <atomic>

#include "FreeRTOS.h"
#include "queue.h"

#include "base/defs.h"
#include "base/err.h"
#include "base/static_resource.h"
#include "base/static_queue.h"
#include "drivers/uart.h"
#include "services/fumi/wirecodec.h"
#include "services/fumi/protocol.h"
#include "services/fumi/shared.h"
#include "services/fumi/hash.h"

namespace fumi
{

#ifndef FUMI_COMPILE_LEVEL
#define FUMI_COMPILE_LEVEL  FUMI_LEVEL_DEBUG
#endif // FUMI_COMPILE_LEVEL

constexpr static proto::LogLevel kDefaultLevel = proto::LogLevel::kDebug;
// 1 (type tag) + 4 (common data width)
constexpr static uint32_t kArgsCapacity = (1 + 4) * 12;


struct Stats
{
    uint32_t sent;
    uint32_t uart_tx_error;
    uint32_t encode_error;
    uint32_t resource_unavail_error;
    uint32_t drop_no_free_slot;
    uint32_t drop_in_isr;
    uint32_t drop_unknown;
};

struct Message
{
    proto::LogLevel level;
    // TODO(masshiroio): switch to 64bit monotonic timestamp
    uint32_t timestamp;
    LiteralStrId tag;
    LiteralStrId fmt;
    std::size_t nb_args;
    std::size_t argsbuf_len;
    uint8_t argsbuf[kArgsCapacity];
};

LiteralStrId strerr(ErrStatus err);

class Logger
{
public:
    constexpr static uint32_t kMsgMaxLen = 256;
    constexpr static uint32_t kNbSlots = 16;
    constexpr static int kTagMaxLen = 24;

    // FreeRTOS task stack depth in `StackType_t` unit, not bytes
    constexpr static uint32_t kTaskStackDepth = 384;
    constexpr static TickType_t kNoSlotTimeout = pdMS_TO_TICKS(0);

    struct Config
    {
        SharedResources *shared;
        uint16_t task_priority;
    };

    ErrStatus init(const Config& config);

    void set_level(proto::LogLevel level);

    // NEVER pass non-static %s strings.
    // A pointer with %p is always safe since the pointer will not be dereferenced.
    template<typename...Args>
    void log(proto::LogLevel level, LiteralStrId tag, LiteralStrId fmt, Args&&...args)
    {
        if (!ready_ || level < level_.load(std::memory_order_relaxed))
            return;
        if (level == proto::LogLevel::kSilent)
            return;

        int32_t slot_idx = try_get_free_slot_index();
        if (slot_idx < 0)
            return;
        Message& msg = slots_[slot_idx];

        msg.level = level;
        msg.tag = tag;
        msg.fmt = fmt;
        msg.timestamp = xTaskGetTickCount();
        msg.nb_args = sizeof...(Args);

        constexpr std::size_t kArgsSizeBytes =
            proto::prim_tagged_data_tight_packed_size<std::decay_t<Args>...>();
        static_assert(kArgsSizeBytes <= kArgsCapacity, "log arguments exceed slot storage capacity");

        [[maybe_unused]] std::size_t pos = 0;
        (..., proto::prim_tagged_data_store_unsafe<std::decay_t<Args>>(msg.argsbuf, pos, args));

        msg.argsbuf_len = kArgsSizeBytes;

        submit_slot(slot_idx);
    }

    // Note: approx. stats, since the data may be updated when copying
    // the structure.
    void get_stats_approx(Stats& out) const;

private:
    static_assert(std::atomic<uint8_t>::is_always_lock_free &&
        "requires lock-free atomic implementation");

    int32_t try_get_free_slot_index();
    void submit_slot(uint8_t slot_idx);

    [[noreturn]] void task_routine();

    bool ready_{false};
    std::atomic<proto::LogLevel> level_{kDefaultLevel};
    SharedResources *shared_{nullptr};

    StaticQueue<uint8_t, kNbSlots> q_ready_slot_{StaticInitOption::kDefer};
    StaticQueue<uint8_t, kNbSlots> q_free_slot_{StaticInitOption::kDefer};

    TaskHandle_t task_{nullptr};
    StaticTask_t task_data_{};
    alignas(portBYTE_ALIGNMENT) StackType_t task_stack_[kTaskStackDepth]{};

    Stats stats_{};
    Message slots_[kNbSlots]{};
    alignas(4) char format_buf_[kMsgMaxLen]{};
};

Logger& get_default_logger();

namespace detail
{

    template<class T>
    constexpr T as_printf_check_arg(T value)
    {
        return value;
    }

    constexpr const char *as_printf_check_arg(LiteralStrId)
    {
        return "";
    }

} // namespace fumi::detail

} // namespace fumi

#define FUMI_DETAIL_PRINTF_CHECK_ARG(arg) \
    , ::fumi::detail::as_printf_check_arg((arg))

#define FUMI_LOGx(tag_id, fmt, level, ...)                                      \
    do {                                                                        \
        if (false)                                                              \
        {                                                                       \
            std::printf(fmt GARNET_PP_FOR_EACH(                                 \
                FUMI_DETAIL_PRINTF_CHECK_ARG, __VA_ARGS__));                    \
        }                                                                       \
        constexpr auto fmt_id = FUMI_LITERAL_STR(fmt);                          \
        fumi::get_default_logger().log(level, tag_id, fmt_id, ##__VA_ARGS__);   \
    } while (0)


#if FUMI_LEVEL_DEBUG >= FUMI_COMPILE_LEVEL
#define FUMI_LOGD(tag, fmt, ...) FUMI_LOGx(tag, fmt, fumi::proto::LogLevel::kDebug, ##__VA_ARGS__)
#else
#define FUMI_LOGD(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_DEBUG

#if FUMI_LEVEL_INFO >= FUMI_COMPILE_LEVEL
#define FUMI_LOGI(tag, fmt, ...) FUMI_LOGx(tag, fmt, fumi::proto::LogLevel::kInfo, ##__VA_ARGS__)
#else
#define FUMI_LOGI(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_INFO

#if FUMI_LEVEL_WARN >= FUMI_COMPILE_LEVEL
#define FUMI_LOGW(tag, fmt, ...) FUMI_LOGx(tag, fmt, fumi::proto::LogLevel::kWarn, ##__VA_ARGS__)
#else
#define FUMI_LOGW(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_WARN

#if FUMI_LEVEL_ERROR >= FUMI_COMPILE_LEVEL
#define FUMI_LOGE(tag, fmt, ...) FUMI_LOGx(tag, fmt, fumi::proto::LogLevel::kError, ##__VA_ARGS__)
#else
#define FUMI_LOGE(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_ERROR

#endif //GARNET_FUMI_H

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
constexpr static uint32_t kTaskStackDepth = 256;

constexpr static TickType_t kNoSlotTimeout = pdMS_TO_TICKS(0);

struct Stats
{
    uint32_t sent;
    uint32_t uart_tx_error;
    uint32_t drop_no_free_slot;
    uint32_t drop_format_failure;
    uint32_t drop_in_isr;
    uint32_t drop_unknown;
};

struct Message
{
    size_t len;
    char buf[kMsgMaxLen];
};

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
    void printf(Level level, const char *tag, const char *fmt, ...)
        __attribute__((format(printf, 4, 5)));
    void vprintf(Level level, const char *tag, const char *fmt, va_list ap);

    // Note: approx. stats, since the data may be updated when copying
    // the structure.
    void get_stats_approx(Stats& out) const;

private:
    static_assert(std::atomic<uint8_t>::is_always_lock_free &&
        "requires lock-free atomic implementation");

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
};

Logger& get_default_instance();

} // namespace fumi

#if FUMI_LEVEL_DEBUG >= FUMI_COMPILE_LEVEL
#define FUMI_LOGD(tag, fmt, ...) \
    fumi::get_default_instance().printf(fumi::Level::kDebug, tag, fmt, ##__VA_ARGS__)
#else
#define FUMI_LOGD(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_DEBUG

#if FUMI_LEVEL_INFO >= FUMI_COMPILE_LEVEL
#define FUMI_LOGI(tag, fmt, ...) \
    fumi::get_default_instance().printf(fumi::Level::kInfo, tag, fmt, ##__VA_ARGS__)
#else
#define FUMI_LOGI(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_INFO

#if FUMI_LEVEL_WARN >= FUMI_COMPILE_LEVEL
#define FUMI_LOGW(tag, fmt, ...) \
    fumi::get_default_instance().printf(fumi::Level::kWarn, tag, fmt, ##__VA_ARGS__)
#else
#define FUMI_LOGW(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_WARN

#if FUMI_LEVEL_ERROR >= FUMI_COMPILE_LEVEL
#define FUMI_LOGE(tag, fmt, ...) \
    fumi::get_default_instance().printf(fumi::Level::kError, tag, fmt, ##__VA_ARGS__)

#else
#define FUMI_LOGE(tag, fmt, ...) ((void)0)
#endif // FUMI_LEVEL_ERROR

#endif //GARNET_FUMI_H

// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstring>

#include "FreeRTOS.h"
#include "queue.h"

#include "base/panic.h"
#include "services/fumi.h"

namespace fumi
{

namespace
{
Logger g_default_instance;
} // namespace anonymous

Logger& get_default_instance()
{
    return g_default_instance;
}


[[noreturn]] void Logger::task_routine()
{
    while (true)
    {
        uint8_t slot_idx;
        if (!q_ready_slot_.receive(slot_idx, portMAX_DELAY))
            continue;
        GCHECK(slot_idx < kNbSlots && "invalid slot index");

        Message *slot = &slots_[slot_idx];

        if (auto tx = uart_tx_->acquire(pdMS_TO_TICKS(100)))
        {
            ErrStatus err = tx->transmit({
                reinterpret_cast<uint8_t*>(slot->buf), slot->len}, XferMethod::kDMA, pdMS_TO_TICKS(20));

            if (err != ErrStatus::kOk)
                stats_.uart_tx_error++;
            else
                stats_.sent++;
        }
        else
        {
            stats_.uart_tx_error++;
        }

        slot->len = 0;

        q_free_slot_.send(slot_idx, 0);
    }
}

ErrStatus Logger::init(const Config& config)
{
    uart_tx_ = config.tx;

    q_free_slot_.init();
    q_ready_slot_.init();

    // Initially, all the slots are available (free). Add them to the
    // FREE queue so that a log producer can get one free slot.
    for (uint8_t i = 0; i < kNbSlots; i++)
        q_free_slot_.send(i, 0);

    task_ = xTaskCreateStatic(+[](void *self) {
        static_cast<Logger*>(self)->task_routine();
    }, "fumi", kTaskStackDepth, this, config.task_priority, task_stack_, &task_data_);
    if (task_ == nullptr)
        return ErrStatus::kFail;

    ready_ = true;
    return ErrStatus::kOk;
}

void Logger::set_level(Level level)
{
    level_.store(level, std::memory_order_relaxed);
}

void Logger::printf(Level level, const char *tag, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    Logger::vprintf(level, tag, fmt, ap);;
    va_end(ap);
}

namespace
{

char get_level_char(Level level)
{
    switch (level)
    {
        case Level::kDebug: return 'D';
        case Level::kInfo: return 'I';
        case Level::kWarn: return 'W';
        case Level::kError: return 'E';
        default:
            return '?';
    }
}

int format_prefix(char *buf, size_t size, Level level, const char *tag)
{
    const uint32_t ts = xTaskGetTickCount();

    return snprintf(
        buf, size, "%c (%lu) %.*s: ",
        get_level_char(level),
        static_cast<unsigned long>(ts),
        kTagMaxLen,
        tag ? tag : "???"
    );
}

size_t format_msg(Message *msg, Level level,
                  const char *tag, const char *fmt, va_list ap)
{
    int n = format_prefix(msg->buf, sizeof(msg->buf), level, tag);
    if (n < 0)
        return 0;

    auto used = static_cast<size_t>(n);
    if (used >= sizeof(msg->buf))
        used = sizeof(msg->buf) - 1;

    size_t rem = sizeof(msg->buf) - used;

    if (vsnprintf(msg->buf + used, rem, fmt, ap) < 0)
        return 0;

    used = strnlen(msg->buf, sizeof(msg->buf));

    if (used + 2 < sizeof(msg->buf))
    {
        msg->buf[used++] = '\r';
        msg->buf[used++] = '\n';
        msg->buf[used] = '\0';
    }
    else if (used + 1 < sizeof(msg->buf))
    {
        msg->buf[used++] = '\n';
        msg->buf[used] = '\0';
    }
    else if (used > 0)
    {
        msg->buf[sizeof(msg->buf) - 2] = '\r';
        msg->buf[sizeof(msg->buf) - 1] = '\n';
        used = sizeof(msg->buf);
    }

    return used;
}

} // namespace anonymous

void Logger::vprintf(Level level, const char *tag, const char *fmt, va_list ap)
{
    // not initialized yet
    if (!ready_)
        return;

    if (!fmt)
        return;

    if (level < level_.load(std::memory_order_relaxed))
        return;

    if (__get_IPSR() != 0U)
    {
        stats_.drop_in_isr++;
        return;
    }

    // Try to get a free slot
    uint8_t slot_idx;
    if (!q_free_slot_.receive(slot_idx, kNoSlotTimeout))
    {
        stats_.drop_no_free_slot++;
        return;
    }
    GCHECK(slot_idx < kNbSlots);

    // Format
    Message *msg = &slots_[slot_idx];
    va_list apdup;
    va_copy(apdup, ap);
    msg->len = format_msg(msg, level, tag, fmt, apdup);
    va_end(apdup);

    if (!msg->len)
    {
        msg->len = 0;
        stats_.drop_format_failure++;
        q_free_slot_.send(slot_idx, 0);
        return;
    }

    if (!q_ready_slot_.send(slot_idx, 0))
    {
        stats_.drop_unknown++;
        q_free_slot_.send(slot_idx, 0);
    }
}

void Logger::get_stats_approx(Stats& out) const
{
    memcpy(&out, &stats_, sizeof(stats_));
}

} // namespace fumi

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

size_t format_slot(const Message& slot, char *out_buf, size_t max_len)
{
    int prefix_len = snprintf(out_buf, max_len, "%c (%lu) %.*s: ",
                              get_level_char(slot.level),
                              static_cast<unsigned long>(slot.timestamp),
                              kTagMaxLen,
                              slot.tag ? slot.tag : "???");

    if (prefix_len < 0)
        return 0;

    auto used = static_cast<size_t>(prefix_len);
    if (used >= max_len)
        used = max_len - 1;

    size_t rem = max_len - used;
    int body_len = 0;
    if (slot.format_fn)
    {
        body_len = slot.format_fn(slot.args_store, out_buf + used, rem);
    }
    if (body_len < 0) return 0;

    used = strlen(out_buf);
    if (used + 2 < max_len)
    {
        out_buf[used++] = '\r';
        out_buf[used++] = '\n';
        out_buf[used] = '\0';
    }
    else if (used + 1 < max_len)
    {
        out_buf[used++] = '\n';
        out_buf[used] = '\0';
    }
    else if (used > 0)
    {
        out_buf[max_len - 2] = '\r';
        out_buf[max_len - 1] = '\n';
        used = max_len;
    }
    return used;
}

Logger& get_default_instance()
{
    return g_default_instance;
}


void Logger::submit_log(Level level, const char *tag, FormatDispatcher *format_fn,
                        const void *payload, size_t payload_size)
{
    if (__get_IPSR() != 0U)
    {
        stats_.drop_in_isr++;
        return;
    }

    uint8_t slot_idx;
    if (!q_free_slot_.receive(slot_idx, kNoSlotTimeout))
    {
        stats_.drop_no_free_slot++;
        return;
    }

    GCHECK(slot_idx < kNbSlots);

    Message& slot = slots_[slot_idx];

    slot.level = level;
    slot.timestamp = xTaskGetTickCount();
    slot.tag = tag;
    slot.format_fn = format_fn;
    memcpy(slot.args_store, payload, payload_size);

    if (!q_ready_slot_.send(slot_idx, 0))
    {
        stats_.drop_unknown++;
        q_free_slot_.send(slot_idx, 0);
    }
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
        size_t len = format_slot(*slot, format_buf_, sizeof(format_buf_));

        slot->format_fn = nullptr;
        q_free_slot_.send(slot_idx, 0);

        if (auto tx = uart_tx_->acquire(pdMS_TO_TICKS(100)))
        {
            const ErrStatus err = tx->transmit({
                reinterpret_cast<uint8_t*>(format_buf_), len},
                XferMethod::kDMA, pdMS_TO_TICKS(20));

            if (err != ErrStatus::kOk)
                stats_.uart_tx_error++;
            else
                stats_.sent++;
        }
        else
        {
            stats_.uart_tx_error++;
        }
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

void Logger::get_stats_approx(Stats& out) const
{
    memcpy(&out, &stats_, sizeof(stats_));
}

} // namespace fumi

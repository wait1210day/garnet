// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstring>

#include "FreeRTOS.h"
#include "queue.h"

#include "base/panic.h"
#include "services/fumi/logger.h"
#include "services/fumi/wirecodec.h"
#include "services/fumi/protocol.h"

namespace fumi
{

namespace
{
Logger g_default_logger;
} // namespace anonymous

Logger& get_default_logger()
{
    return g_default_logger;
}


int32_t Logger::try_get_free_slot_index()
{
    if (__get_IPSR() != 0U)
    {
        stats_.drop_in_isr++;
        return -1;
    }

    uint8_t slot_idx;
    if (!q_free_slot_.receive(slot_idx, kNoSlotTimeout))
    {
        stats_.drop_no_free_slot++;
        return -1;
    }
    GCHECK(slot_idx < kNbSlots);

    return slot_idx;
}

void Logger::submit_slot(uint8_t slot_idx)
{
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
        ScopeExitGuard send_free_slot([&] {
            q_free_slot_.send(slot_idx, 0);
        });

        auto result = shared_->encode_send_frame(
            proto::Channel::kLog, pdMS_TO_TICKS(10), [&](proto::PacketWriter& w) {
                w.write_enum(slot->level);
                w.write_numeric<uint64_t>(slot->timestamp);
                w.write_sstr(slot->tag);
                w.write_sstr(slot->fmt);
                w.write_numeric<uint8_t>(slot->nb_args);
                w.write_bytes({slot->argsbuf, slot->argsbuf_len});

                send_free_slot.advance();
                return true;
            }
        );

        if (result == FrameError::kOk)
            stats_.sent++;
        else if (result == FrameError::kErrAcquireLock)
            stats_.resource_unavail_error++;
        else if (result == FrameError::kErrTransport)
            stats_.uart_tx_error++;
        else if (result == FrameError::kErrEncode)
            stats_.encode_error++;
        else
            stats_.drop_unknown++;
    }
}

ErrStatus Logger::init(const Config& config)
{
    shared_ = config.shared;

    q_free_slot_.init();
    q_ready_slot_.init();

    // Initially, all the slots are available (free). Add them to the
    // FREE queue so that a log producer can get one free slot.
    for (uint8_t i = 0; i < kNbSlots; i++)
        q_free_slot_.send(i, 0);

    task_ = xTaskCreateStatic(+[](void *self) {
        static_cast<Logger*>(self)->task_routine();
    },
        "fumi::logger", kTaskStackDepth, this,
        config.task_priority, task_stack_, &task_data_
    );
    if (task_ == nullptr)
        return ErrStatus::kFail;

    ready_ = true;
    return ErrStatus::kOk;
}

void Logger::set_level(proto::LogLevel level)
{
    level_.store(level, std::memory_order_relaxed);
}

void Logger::get_stats_approx(Stats& out) const
{
    memcpy(&out, &stats_, sizeof(stats_));
}

} // namespace fumi

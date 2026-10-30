// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_CMDPIPE_H
#define GARNET_CMDPIPE_H

#include "FreeRTOS.h"
#include "task.h"

#include "base/err.h"
#include "base/flags.h"
#include "drivers/uart.h"
#include "services/fumi/wirecodec.h"
#include "services/fumi/shared.h"

namespace fumi
{

struct CmdParseInfo
{
    proto::PacketReader& reader;
    uint16_t transaction_id;
    uint8_t nb_operands;
};

class CmdPipe
{
public:
    constexpr static uint32_t kStackDepth = 256;
    constexpr static TickType_t kTimeout = pdMS_TO_TICKS(10);

    struct Config
    {
        SharedResources *shared;
        uart::Receiver *uart_rx;
        uint32_t task_priority;
    };

    template<typename F>
    FrameError send_resp(uint16_t transaction_id, uint16_t resp_seq,
                         Flags<proto::ResponseFlag> flags, F&& fill_payload)
    {
        return shared_->encode_send_frame(
            proto::Channel::kResponse, kTimeout, [&](proto::PacketWriter& w) {
                w.write_numeric(transaction_id);
                w.write_numeric(resp_seq);
                w.write_numeric(flags.raw_value());
                return fill_payload(w);
            }
        );
    }

    FrameError send_resp_err(uint16_t transaction_id, proto::ResponseFlag err,
                             uint16_t seq, const char *detail_sstr)
    {
        Flags flags{err};
        flags.set(proto::ResponseFlag::kDone);
        return send_resp(
            transaction_id, seq, flags, [&](proto::PacketWriter& w) {
                return w.write_sstr(detail_sstr);
            }
        );
    }

    ErrStatus init(const Config& config);

private:
    void handle_frame(std::span<uint8_t> frame);
    [[noreturn]] void task_routine();

    // Protocol defined command handlers
    void handle_cmd_host_hello(const CmdParseInfo& info);

    SharedResources *shared_{nullptr};
    uart::Receiver *uart_rx_{nullptr};

    wire::FrameCollector frame_clctor_{};

    TaskHandle_t task_{nullptr};
    StaticTask_t task_data_{};
    alignas(portBYTE_ALIGNMENT) StackType_t task_stack_[kStackDepth]{};
};

} // namespace anonymous

#endif //GARNET_CMDPIPE_H

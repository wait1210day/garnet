// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include "base/panic.h"
#include "base/flags.h"
#include "base/utils.h"
#include "services/fumi/logger.h"
#include "services/fumi/cmdpipe.h"

namespace fumi
{

namespace
{
    constexpr auto kTAG = FUMI_LITERAL_STR("cmdpipe");
    using RespFlag = proto::ResponseFlag;
} // namespace anonymous

ErrStatus CmdPipe::init(const Config &config)
{
    if (!config.shared || !config.uart_rx)
        return ErrStatus::kInvalidArg;

    shared_ = config.shared;
    uart_rx_ = config.uart_rx;

    task_ = xTaskCreateStatic(+[](void *param) {
        static_cast<CmdPipe*>(param)->task_routine();
    },
        "fumi::cmdpipe", kStackDepth, this,
        config.task_priority, task_stack_, &task_data_
    );
    if (!task_)
    {
        FUMI_LOGE(kTAG, "failed to create cmdpipe task");
        return ErrStatus::kFail;
    }

    return ErrStatus::kOk;
}

void CmdPipe::handle_frame(std::span<uint8_t> frame)
{
    proto::PacketReader reader(frame);

    auto magic = reader.read_numeric<uint8_t>();
    auto version = reader.read_numeric<uint8_t>();
    auto channel = reader.read_enum<proto::Channel>();
    auto payload_len = reader.read_numeric<uint16_t>();
    if (reader.has_error())
        return;

    if (magic != proto::kMagic || version != proto::kVersion)
        return;

    if (payload_len != reader.remaining_bytes())
        return;

    if (channel == proto::Channel::kCmd)
    {
        if (payload_len == 0)
            return;

        auto opcode = reader.read_enum<proto::CmdOpcode>();
        auto transaction_id = reader.read_numeric<uint16_t>();
        auto nb_operands = reader.read_numeric<uint8_t>();
        if (reader.has_error())
            return;

        CmdParseInfo info{
            .reader = reader,
            .transaction_id = transaction_id,
            .nb_operands = nb_operands
        };

        // Protocol defined commands
        switch (opcode)
        {
            case proto::CmdOpcode::kHostHello:
                handle_cmd_host_hello(info);
                break;

            case proto::CmdOpcode::kSetLevel:
                handle_cmd_set_level(info);
                break;

            case proto::CmdOpcode::kGetStats:
                handle_cmd_get_stats(info);
                break;

            case proto::CmdOpcode::kPing:
                handle_cmd_ping(info);
                break;

            default:
                FUMI_LOGW(kTAG, "unrecognized opcode 0x%02x", static_cast<uint8_t>(opcode));
                send_resp_err(info.transaction_id, 0, proto::ErrCode::kInvalidOpcode);
                break;
        }
    }
}

#define READER_ZERO_REMAIN_CHECK(info)              \
    if ((info).reader.remaining_bytes() > 0) {      \
        send_resp_err((info).transaction_id, 0,     \
            proto::ErrCode::kIllFormedPacket);      \
        return;                                     \
    }

#define RESP_INVALID_OPERAND_IF(info, cond)         \
    if (cond) {                                     \
        send_resp_err((info).transaction_id, 0,     \
            proto::ErrCode::kInvalidOperands);      \
        return;                                     \
    }


void CmdPipe::handle_cmd_host_hello(const CmdParseInfo& info)
{
    RESP_INVALID_OPERAND_IF(info, info.nb_operands != 0)
    READER_ZERO_REMAIN_CHECK(info)

    auto build_hash = get_build_id_sha1();
    GCHECK(build_hash.size_bytes() == 20);

    send_resp(
        info.transaction_id, 0,
        Flags{proto::ResponseFlag::kDone},
        [&](proto::PacketWriter& w) {
            w.write_istr("Garnet");
            w.write_istr("GaN buck-boost DC/DC converter");
            return w.write_ibytes(build_hash);
        }
    );
}

void CmdPipe::handle_cmd_set_level(const CmdParseInfo& info)
{
    RESP_INVALID_OPERAND_IF(info, info.nb_operands != 1)

    auto level = info.reader.read_tagged_primitive<uint8_t>();
    READER_ZERO_REMAIN_CHECK(info)
    RESP_INVALID_OPERAND_IF(info, info.reader.has_error())

    RESP_INVALID_OPERAND_IF(info,
        level != static_cast<uint8_t>(proto::LogLevel::kDebug) &&
        level != static_cast<uint8_t>(proto::LogLevel::kInfo) &&
        level != static_cast<uint8_t>(proto::LogLevel::kWarn) &&
        level != static_cast<uint8_t>(proto::LogLevel::kError) &&
        level != static_cast<uint8_t>(proto::LogLevel::kSilent))

    FUMI_LOGI(kTAG, "log level will be set to 0x%02x", static_cast<uint8_t>(level));
    get_default_logger().set_level(static_cast<proto::LogLevel>(level));

    send_resp(info.transaction_id, 0, Flags{proto::ResponseFlag::kDone},
        [&](proto::PacketWriter& w) { return true; });
}

void CmdPipe::handle_cmd_get_stats(const CmdParseInfo& info)
{
    RESP_INVALID_OPERAND_IF(info, info.nb_operands != 0)
    READER_ZERO_REMAIN_CHECK(info)

    Stats stats{};
    get_default_logger().get_stats_approx(stats);

    send_resp(
        info.transaction_id, 0, Flags{proto::ResponseFlag::kDone},
        [&](proto::PacketWriter& w) {
            w.write_numeric<uint32_t>(stats.sent);
            w.write_numeric<uint32_t>(stats.uart_tx_error);
            w.write_numeric<uint32_t>(stats.encode_error);
            w.write_numeric<uint32_t>(stats.resource_unavail_error);
            w.write_numeric<uint32_t>(stats.drop_no_free_slot);
            w.write_numeric<uint32_t>(stats.drop_in_isr);
            return w.write_numeric<uint32_t>(stats.drop_unknown);
        }
    );
}

void CmdPipe::handle_cmd_ping(const CmdParseInfo& info)
{
    RESP_INVALID_OPERAND_IF(info, info.nb_operands != 0)
    READER_ZERO_REMAIN_CHECK(info)

    send_resp(
        info.transaction_id, 0, Flags{proto::ResponseFlag::kDone},
        [](proto::PacketWriter& w) {
            return w.write_istr("PONG!");
        }
    );
}


[[noreturn]] void CmdPipe::task_routine()
{
    WireCpuCrc16 cpu_crc;

    while (true)
    {
        auto recvbuf = frame_clctor_.writable_span();
        GCHECK(!recvbuf.empty() && "failed to get a writable buffer");

        // UART receiver is exclusively used by this task,
        // so we use 0 timeout for a non-blocking acquisition.
        auto rxscope = uart_rx_->acquire(0);
        GCHECK(rxscope);

        // This function wakes us up for many reasons, like a buffer-full event,
        // UART idle interrupt, a transport layer error, etc.
        // But no matter what (and how long) we read, the frame collector knows
        // the frame boundary and handles oversized frames.
        auto read_result = rxscope->read_some(recvbuf, portMAX_DELAY);
        if (read_result.status != uart::Receiver::Status::kOk)
        {
            FUMI_LOGE(kTAG, "uart rx error, reset frame collector and try again later...");
            frame_clctor_.reset();

            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        FUMI_LOGD(kTAG, "read %u bytes from uart", read_result.size);

        frame_clctor_.commit_write(read_result.size);

        while (auto frame = frame_clctor_.next_frame())
        {
            uint16_t expected_crc;
            auto decoded = wire::decode_inplace(*frame, expected_crc);
            if (decoded.empty())
            {
                FUMI_LOGW(kTAG, "got a frame of size %u but failed to decode, ignored",
                          frame->size_bytes());
                continue;
            }

            const std::size_t len = decoded.size();
            if (len > proto::kMaxFrameBytes)
            {
                // oversized frame
                FUMI_LOGW(kTAG, "oversized frame, dropped");
                continue;
            }

            auto actual_crc = shared_->compute_crc16(0, decoded);
            if (!actual_crc)
            {
                FUMI_LOGW(kTAG, "failed to acquire the CRC hw resource, fallback to cpu");

                cpu_crc.reset();
                for (uint8_t byte : decoded)
                    cpu_crc.update(byte);

                actual_crc = cpu_crc.value();
            }

            FUMI_LOGD(kTAG, "got a frame, len=%u decoded_len=%u actual_crc=0x%04x expected_crc=0x%04x",
                frame->size_bytes(), len, *actual_crc, expected_crc);

            if (*actual_crc != expected_crc)
            {
                FUMI_LOGW(kTAG, "frame dropped because of CRC");
                continue;
            }

            handle_frame(decoded);
        }
    }
}

} // namespace fumi

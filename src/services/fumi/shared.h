// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_SHARED_H
#define GARNET_SHARED_H

#include <cstddef>
#include <cstdint>

#include <optional>

#include "FreeRTOS.h"

#include "base/static_mutex.h"
#include "drivers/uart.h"
#include "services/fumi/crc16.h"
#include "services/fumi/wirecodec.h"
#include "services/fumi/protocol.h"

namespace fumi
{

enum class FrameError
{
    kOk,
    kFail,
    kErrAcquireLock,
    kErrSize,
    kErrEncode,
    kErrTransport
};

class SharedResources
{
public:
    struct Config
    {
        CRC_TypeDef *ll_crc;
        uart::Transmitter *uart_tx;
    };

    void init(const Config& config);

    std::optional<uint16_t> compute_crc16(TickType_t timeout, std::span<const uint8_t> data) const;

    template<typename F>
    FrameError encode_send_frame(proto::Channel ch, TickType_t timeout, F&& callback)
    {
        auto tx = uart_tx_->acquire(timeout);
        if (!tx)
            return FrameError::kErrAcquireLock;

        proto::PacketWriter writer(std::span{
            tx_buffer_ + wire::kCobsHeadroom, wire::kMaxCobsInputBytes});

        // Header
        writer.write_numeric(proto::kMagic);
        writer.write_numeric(proto::kVersion);
        writer.write_enum(ch);
        std::size_t payload_size_pos = writer.current_pos();
        writer.skip_bytes(sizeof(uint16_t));
        if (writer.has_error())
            return FrameError::kErrSize;

        // Write payload
        if (!callback(writer))
            return FrameError::kErrSize;
        if (writer.has_error() || writer.current_pos() > proto::kMaxFrameBytes)
            return FrameError::kErrSize;

        const std::size_t payload_size = writer.current_pos()
            - payload_size_pos - sizeof(uint16_t);
        writer.write_numeric_at<uint16_t>(payload_size, payload_size_pos);

        // Compute and append CRC
        auto crc16 = compute_crc16(timeout, writer.result());
        if (!crc16)
            return FrameError::kErrAcquireLock;
        writer.write_numeric<uint16_t>(*crc16);
        GCHECK(!writer.has_error());

        // COBS encoding
        auto encoded = wire::encode_inplace(tx_buffer_, writer.current_pos());
        if (encoded.empty())
            return FrameError::kErrEncode;

        // Send
        if (tx->transmit(encoded, XferMethod::kDMA, timeout) != ErrStatus::kOk)
            return FrameError::kErrTransport;
        return FrameError::kOk;
    }

private:
    StaticMutex crc_mutex_{StaticInitOption::kDefer};
    WireAccelCrc16 crc_;

    uart::Transmitter *uart_tx_{nullptr};
    alignas(4) uint8_t tx_buffer_[wire::kWireBufferBytes]{};
};

} // namespace fumi

#endif //GARNET_SHARED_H

// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_WIRECODEC_H
#define GARNET_WIRECODEC_H

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <concepts>
#include <bit>
#include <span>
#include <array>
#include <cstring>
#include <optional>
#include <string_view>

#include "base/panic.h"
#include "services/fumi/crc16.h"
#include "services/fumi/protocol.h"

namespace fumi::wire
{

constexpr std::size_t cobs_headroom(std::size_t n)
{
    return n / 254 + 1;
}

// CRC16, 1-byte 0x00 delimiter
constexpr std::size_t kCrcBytes = 2;
constexpr std::size_t kDelimBytes = 1;

constexpr std::size_t kMaxCobsInputBytes = proto::kMaxFrameBytes + kCrcBytes;
constexpr std::size_t kCobsHeadroom = kMaxCobsInputBytes / 254 + 1;
constexpr std::size_t kWireBufferBytes = kMaxCobsInputBytes + kCobsHeadroom + kDelimBytes;

// Encode data in `buffer` in place. Returns a subspan of `buffer`, zero-copy
// memory encoding. Empty span will be returned if fails.
// The provided buffer must have `kCobsHeadroom` reserved at the beginning as the
// encoding headroom, and effective data is `buffer[kCobsHeadroom, kCobsHeadroom + data_size)`.
std::span<uint8_t> encode_inplace(std::span<uint8_t> buffer, std::size_t data_size);


class FrameCollector
{
public:
    FrameCollector() = default;

    void reset();

    // Get a free space which can be written by the transport layer driver.
    // Note that the caller must consume all the collected frames, i.e.
    // `next_frame()` returns empty, before getting a writable span.
    [[nodiscard]] std::span<uint8_t> writable_span();

    // Feedback, how many bytes you have written into the buffer you
    // get from `writable_span()`?
    void commit_write(std::size_t size);

    // Extract a received frame if we have one, otherwise, an empty span.
    // Caller is allowed to modify the returned buffer span, e.g. decode
    // a frame in place.
    // Note that the returned span does NOT contain the deliminator 0x00.
    [[nodiscard]] std::optional<std::span<uint8_t>> next_frame();

private:
    std::array<uint8_t, kWireBufferBytes> buf_;
    std::size_t write_pos_{0};
    std::size_t scan_pos_{0};
    bool discarding_oversize_{false};
};

std::span<uint8_t> decode_inplace(std::span<uint8_t> buf, uint16_t& crc);

} // namespace fumi::wire

#endif //GARNET_WIRECODEC_H

// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "base/panic.h"
#include "services/fumi/wirecodec.h"

namespace fumi::wire
{

std::span<uint8_t> encode_inplace(std::span<uint8_t> buffer, std::size_t data_size)
{
    // Check both the input placement and the worst-case output size before
    // modifying anything. The fixed headroom keeps writes behind unread input.
    if (data_size > kMaxCobsInputBytes || buffer.size() < kCobsHeadroom
        || data_size > buffer.size() - kCobsHeadroom)
        return {};
    if (data_size + cobs_headroom(data_size) + kDelimBytes > buffer.size())
        return {};

    const std::size_t read_end = kCobsHeadroom + data_size;
    std::size_t write_pos = 1;
    std::size_t code_pos = 0;
    uint8_t code = 1;
    bool full_final_block = false;

    for (std::size_t read_pos = kCobsHeadroom; read_pos < read_end; ++read_pos)
    {
        const uint8_t byte = buffer[read_pos];
        full_final_block = false;
        if (byte != 0)
        {
            buffer[write_pos++] = byte;
            ++code;
        }

        if (byte == 0 || code == 0xff)
        {
            buffer[code_pos] = code;
            code_pos = write_pos++;
            code = 1;
            full_final_block = byte != 0;
        }
    }

    // A full final block needs no empty successor; a trailing zero does.
    if (full_final_block)
        write_pos = code_pos;
    else
        buffer[code_pos] = code;

    buffer[write_pos++] = 0x00;
    return buffer.first(write_pos);
}

void FrameCollector::reset()
{
    write_pos_ = 0;
    scan_pos_ = 0;
    discarding_oversize_ = false;
}

std::span<uint8_t> FrameCollector::writable_span()
{
    if (scan_pos_ > 0)
    {
        // We have remaining, pending bytes at the tail, and they should
        // be moved to the beginning of assembly buffer.
        const std::size_t remaining = write_pos_ - scan_pos_;
        if (remaining > 0)
            std::memmove(buf_.data(), buf_.data() + scan_pos_, remaining);

        write_pos_ = remaining;
        scan_pos_ = 0;
    }

    // Still full after moving, may be a oversized frame.
    if (write_pos_ == buf_.size())
    {
        write_pos_ = 0;
        discarding_oversize_ = true;
    }

    return {buf_.data() + write_pos_, buf_.size() - write_pos_};
}

void FrameCollector::commit_write(std::size_t size)
{
    write_pos_ += size;
    GCHECK(write_pos_ <= buf_.size() && "frame assembly buffer overflow");
}

std::optional<std::span<uint8_t>> FrameCollector::next_frame()
{
    while (scan_pos_ < write_pos_)
    {
        std::size_t delim_idx = scan_pos_;
        while (delim_idx < write_pos_)
        {
            if (buf_[delim_idx] == 0x00)
                break;
            delim_idx++;
        }
        if (delim_idx == write_pos_)
            return std::nullopt;

        const std::size_t frame_start = scan_pos_;
        const std::size_t frame_len = delim_idx - frame_start;
        // Move to the next byte of delim (0x00)
        scan_pos_ = delim_idx + 1;

        if (discarding_oversize_)
        {
            // the garbage was finally end
            discarding_oversize_ = false;
            continue;
        }

        if (frame_len == 0)
            continue;

        return std::span<uint8_t>{buf_.data() + frame_start, frame_len};
    }

    return std::nullopt;
}

std::span<uint8_t> decode_inplace(std::span<uint8_t> buf, uint16_t& crc)
{
    // minimum frame size (without 0x00 delimiter): [code:u8] [crc16:u16]
    if (buf.size() < 3)
        return {};

    std::size_t r_idx = 0, w_idx = 0;
    const std::size_t raw_len = buf.size();

    while (r_idx < raw_len)
    {
        const uint8_t code = buf[r_idx++];
        // 0x00 is not allowed in COBS encoding
        if (code == 0)
            return {};

        const std::size_t copy_len = static_cast<std::size_t>(code) - 1;
        // corrupted data
        if (r_idx + copy_len > raw_len)
            return {};

        for (std::size_t i = 0; i < copy_len; i++)
            buf[w_idx++] = buf[r_idx++];

        if (code < 0xff && r_idx < raw_len)
            buf[w_idx++] = 0x00;
    }

    // at least 2 bytes of CRC16
    if (w_idx < 2)
        return {};

    crc = static_cast<uint16_t>(buf[w_idx - 2])
        | (static_cast<uint16_t>(buf[w_idx - 1]) << 8);
    return buf.first(w_idx - 2);
}

} // namespace fumi

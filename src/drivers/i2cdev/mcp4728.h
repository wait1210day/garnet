// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_MCP4728_H
#define GARNET_MCP4728_H

#include <cstdint>
#include <array>
#include <span>
#include <expected>

#include "drivers/i2c_command.h"

namespace mcp4728
{

constexpr uint8_t kDefaultAddr = 0x60;
constexpr uint16_t kResolutionBits = 12;
constexpr uint16_t kMaxDacCode = (1u << kResolutionBits) - 1u; // 4095

enum class Channel : uint8_t
{
    kA = 0,
    kB = 1,
    kC = 2,
    kD = 3
};

enum class Vref : uint8_t
{
    kVdd = 0,
    // internal 2.048V precision reference
    kInt = 1
};

enum class Gain : uint8_t
{
    kX1 = 0,
    kX2 = 1
};

enum class PowerDown : uint8_t
{
    kNormal = 0b00,
    k1kOhm = 0b01,
    k100kOhm = 0b10,
    k500kOhm = 0b11
};


struct ChConf
{
    Channel channel{Channel::kA};
    // UDAC: 0 = convert immediately, 1 = wait for LDAC signal
    bool update_now{true};
    Vref vref{Vref::kVdd};
    PowerDown pd{PowerDown::kNormal};
    Gain gain{Gain::kX1};
    // 12-bit (0 ~ 4095)
    uint16_t data{0};
};

struct FastChConf
{
    PowerDown pd{PowerDown::kNormal};
    uint16_t data{0};
};

// Fast write DAC input register, not affect EEPROM.
// Starting from channel A, continuously write N channels.
template<uint8_t N>
struct FastWrite
{
    static_assert(N >= 1 && N <= 4, "number of channels written must be in [1..4]");

    static constexpr uint8_t kChannelCount = N;
    static constexpr uint8_t kMaxTxBytes = N * 2;

    FastChConf channels[N];

    [[nodiscard]] constexpr std::expected<size_t, i2c::Err>
    serialize(std::span<uint8_t> dst) const
    {
        if (dst.size() < kMaxTxBytes)
            return std::unexpected(i2c::Err::kInvalidArg);

        for (uint8_t i = 0; i < N; i++)
        {
            const FastChConf& ch = channels[i];
            const uint16_t code = ch.data > kMaxDacCode ? kMaxDacCode : ch.data;

            // byte 1: [0 0 PD1 PD0 D11 D10 D9 D8]
            //          ^-- has not effect, fill with 0
            dst[i * 2] = (static_cast<uint8_t>(ch.pd) << 4)
                       | static_cast<uint8_t>((code >> 8) & 0x0F);
            // byte 2: [D7 D6 D5 D4 D3 D2 D1 D0]
            dst[i * 2 + 1] = static_cast<uint8_t>(code & 0xFF);
        }
        return kMaxTxBytes;
    }
};

// Multi-write command.
// Write configurations and DAC codes of selected channels,
// not affecting EEPROM.
template<uint8_t N>
struct MultiWrite
{
    static_assert(N > 0 && N <= 4, "number of channels must be in [1..4]");

    static constexpr uint8_t kChannelCount = N;
    static constexpr uint8_t kMaxTxBytes = N * 3;

    ChConf channels[N]{};

    [[nodiscard]] constexpr std::expected<size_t, i2c::Err>
    serialize(std::span<uint8_t> dst) const
    {
        if (dst.size() < kMaxTxBytes)
            return std::unexpected(i2c::Err::kInvalidArg);

        for (size_t i = 0; i < N; ++i)
        {
            const auto& s = channels[i];
            const uint16_t code = (s.data > kMaxDacCode) ? kMaxDacCode : s.data;
            const size_t idx = i * 3;

            dst[idx] = 0b0100'0000 |
                       (static_cast<uint8_t>(s.channel) << 1) |
                       (s.update_now ? 0 : 1);
            dst[idx + 1] = (static_cast<uint8_t>(s.vref) << 7) |
                           (static_cast<uint8_t>(s.pd) << 5) |
                           (static_cast<uint8_t>(s.gain) << 4) |
                           static_cast<uint8_t>((code >> 8) & 0x0F);
            dst[idx + 2] = static_cast<uint8_t>(code & 0xFF);
        }
        return kMaxTxBytes;
    }
};

// Single write, affect EEPROM.
struct SingleWrite
{
    static constexpr size_t kMaxTxBytes = 3;

    ChConf setting;

    constexpr SingleWrite() = default;
    constexpr explicit SingleWrite(const ChConf& s) : setting(s) {}

    [[nodiscard]] std::expected<size_t, i2c::Err> serialize(std::span<uint8_t> dst) const
    {
        if (dst.size() < kMaxTxBytes)
            return std::unexpected(i2c::Err::kInvalidArg);

        const uint16_t code = (setting.data > kMaxDacCode) ? kMaxDacCode : setting.data;

        // 1st Byte: [0 1 0 1 1 DAC1 DAC0 UDAC]
        dst[0] = 0b0101'1000 |
                 (static_cast<uint8_t>(setting.channel) << 1) |
                 (setting.update_now ? 0 : 1);
        // 2nd Byte: [VREF PD1 PD0 Gx D11 D10 D9 D8]
        dst[1] = (static_cast<uint8_t>(setting.vref) << 7) |
                 (static_cast<uint8_t>(setting.pd) << 5) |
                 (static_cast<uint8_t>(setting.gain) << 4) |
                 static_cast<uint8_t>((code >> 8) & 0x0F);
        // 3rd Byte: [D7 D6 D5 D4 D3 D2 D1 D0]
        dst[2] = static_cast<uint8_t>(code & 0xFF);

        return kMaxTxBytes;
    }
};

struct SequentialWrite
{
    // 1 command byte + 4 channels * 2 bytes = 9
    static constexpr size_t kMaxTxBytes = 9;

    Channel start_channel{Channel::kA};
    bool update_now{true};
    ChConf channels[4]{};

    [[nodiscard]] std::expected<size_t, i2c::Err> serialize(std::span<uint8_t> dst) const
    {
        const uint8_t start_idx = static_cast<uint8_t>(start_channel);
        const uint8_t channel_count = 4 - start_idx;
        const size_t needed = 1 + channel_count * 2;

        if (dst.size() < needed)
            return std::unexpected(i2c::Err::kInvalidArg);

        // 1st Byte: [0 1 0 1 0 DAC1 DAC0 UDAC]
        dst[0] = 0b0101'0000 | (start_idx << 1) | (update_now ? 0 : 1);

        for (uint8_t i = 0; i < channel_count; ++i)
        {
            const auto& s = channels[start_idx + i];
            const uint16_t code = (s.data > kMaxDacCode) ? kMaxDacCode : s.data;
            const size_t offset = 1 + i * 2;

            dst[offset] = (static_cast<uint8_t>(s.vref) << 7) |
                          (static_cast<uint8_t>(s.pd) << 5) |
                          (static_cast<uint8_t>(s.gain) << 4) |
                          static_cast<uint8_t>((code >> 8) & 0x0F);
            dst[offset + 1] = static_cast<uint8_t>(code & 0xFF);
        }
        return needed;
    }
};

struct WriteVref
{
    static constexpr size_t kMaxTxBytes = 1;

    Vref vref_a{Vref::kVdd};
    Vref vref_b{Vref::kVdd};
    Vref vref_c{Vref::kVdd};
    Vref vref_d{Vref::kVdd};

    [[nodiscard]] std::expected<size_t, i2c::Err> serialize(std::span<uint8_t> dst) const
    {
        if (dst.empty())
            return std::unexpected(i2c::Err::kInvalidArg);

        // Byte: [1 0 0 0 VREF.A VREF.B VREF.C VREF.D]
        dst[0] = 0b1000'0000 |
                 (static_cast<uint8_t>(vref_a) << 3) |
                 (static_cast<uint8_t>(vref_b) << 2) |
                 (static_cast<uint8_t>(vref_c) << 1) |
                 static_cast<uint8_t>(vref_d);
        return 1;
    }
};

struct WriteGain
{
    static constexpr size_t kMaxTxBytes = 1;

    Gain gain_a{Gain::kX1};
    Gain gain_b{Gain::kX1};
    Gain gain_c{Gain::kX1};
    Gain gain_d{Gain::kX1};

    [[nodiscard]] std::expected<size_t, i2c::Err> serialize(std::span<uint8_t> dst) const
    {
        if (dst.empty())
            return std::unexpected(i2c::Err::kInvalidArg);

        // Byte: [1 1 0 0 Gx.A Gx.B Gx.C Gx.D]
        dst[0] = 0b1100'0000 |
                 (static_cast<uint8_t>(gain_a) << 3) |
                 (static_cast<uint8_t>(gain_b) << 2) |
                 (static_cast<uint8_t>(gain_c) << 1) |
                 static_cast<uint8_t>(gain_d);
        return 1;
    }
};

struct WritePowerDown
{
    static constexpr size_t kMaxTxBytes = 2;

    PowerDown pd_a{PowerDown::kNormal};
    PowerDown pd_b{PowerDown::kNormal};
    PowerDown pd_c{PowerDown::kNormal};
    PowerDown pd_d{PowerDown::kNormal};

    [[nodiscard]] std::expected<size_t, i2c::Err> serialize(std::span<uint8_t> dst) const
    {
        if (dst.size() < kMaxTxBytes)
            return std::unexpected(i2c::Err::kInvalidArg);

        // 1st Byte: [1 0 1 0 PD1.A PD0.A PD1.B PD0.B]
        dst[0] = 0b1010'0000 |
                 (static_cast<uint8_t>(pd_a) << 2) |
                 static_cast<uint8_t>(pd_b);
        // 2nd Byte: [PD1.C PD0.C PD1.D PD0.D 0 0 0 0]
        dst[1] = (static_cast<uint8_t>(pd_c) << 6) |
                 (static_cast<uint8_t>(pd_d) << 4);
        return 2;
    }
};

struct DeviceStatus
{
    static constexpr size_t kRxBytes = 24;

    struct ChannelRecord
    {
        // RDY/BSY: 1 = EEPROM ready, 0 = is writing
        bool is_ready{false};
        // POR: 1 = normal, 0 = power-on-reset status
        bool por_flag{false};
        uint8_t address_bits{0};

        Vref vref{Vref::kVdd};
        PowerDown pd{PowerDown::kNormal};
        Gain gain{Gain::kX1};
        uint16_t dac_val{0};

        Vref eeprom_vref{Vref::kVdd};
        PowerDown eeprom_pd{PowerDown::kNormal};
        Gain eeprom_gain{Gain::kX1};
        uint16_t eeprom_val{0};
    };

    std::array<ChannelRecord, 4> channels{};

    [[nodiscard]] i2c::Err deserialize(std::span<const uint8_t> src)
    {
        if (src.size() < kRxBytes)
            return i2c::Err::kInvalidArg;

        for (size_t ch = 0; ch < 4; ++ch)
        {
            const size_t base = ch * 6;
            auto& rec = channels[ch];

            // Byte 0: [RDY/BSY, POR, DAC1, DAC0, 0, A2, A1, A0]
            rec.is_ready = (src[base] & 0x80) != 0;
            rec.por_flag = (src[base] & 0x40) != 0;
            rec.address_bits = src[base] & 0x07;

            // Byte 1: [VREF, PD1, PD0, Gx, D11..D8]
            rec.vref = static_cast<Vref>((src[base + 1] >> 7) & 0x01);
            rec.pd   = static_cast<PowerDown>((src[base + 1] >> 5) & 0x03);
            rec.gain = static_cast<Gain>((src[base + 1] >> 4) & 0x01);
            rec.dac_val = static_cast<uint16_t>((src[base + 1] & 0x0F) << 8) | src[base + 2];

            // Byte 4: [VREF, PD1, PD0, Gx, D11..D8]
            rec.eeprom_vref = static_cast<Vref>((src[base + 4] >> 7) & 0x01);
            rec.eeprom_pd   = static_cast<PowerDown>((src[base + 4] >> 5) & 0x03);
            rec.eeprom_gain = static_cast<Gain>((src[base + 4] >> 4) & 0x01);
            rec.eeprom_val  = static_cast<uint16_t>((src[base + 4] & 0x0F) << 8) | src[base + 5];
        }

        return i2c::Err::kOk;
    }
};

} // namespace mcp4728

#endif //GARNET_MCP4728_H

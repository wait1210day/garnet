// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_CRC16_H
#define GARNET_CRC16_H

#include <cstdint>

#include "stm32g4xx_ll_crc.h"

#include "base/defs.h"

namespace fumi
{

template<typename T>
concept Crc16Engine = requires(T crc, uint8_t byte) {
    { crc.reset() };
    { crc.update(byte) };
    { crc.value() } -> std::same_as<uint16_t>;
};

// CRC16 CCITT
constexpr uint16_t kCrc16PolynomialCoeffs = 0x1021;

class WireCpuCrc16
{
public:
    void update(uint8_t byte) noexcept
    {
        crc_ ^= static_cast<uint16_t>(byte) << 8;
        for (int i = 0; i < 8; ++i)
        {
            if (crc_ & 0x8000)
            {
                crc_ = (crc_ << 1) ^ kCrc16PolynomialCoeffs;
            }
            else
            {
                crc_ <<= 1;
            }
        }
    }

    [[nodiscard]] uint16_t value() const noexcept { return crc_; }
    void reset() noexcept { crc_ = 0xFFFF; }

private:
    uint16_t crc_{0xFFFF};
};

class WireAccelCrc16
{
public:
    GARNET_NON_COPYABLE(WireAccelCrc16)
    GARNET_NON_MOVABLE(WireAccelCrc16)

    WireAccelCrc16() = default;

    void init(CRC_TypeDef *crc)
    {
        crc_ = crc;

        LL_CRC_SetPolynomialSize(crc_, LL_CRC_POLYLENGTH_16B);
        LL_CRC_SetPolynomialCoef(crc_, kCrc16PolynomialCoeffs);
        LL_CRC_SetInitialData(crc_, 0xffff);
        LL_CRC_SetInputDataReverseMode(crc_, LL_CRC_INDATA_REVERSE_NONE);
        LL_CRC_SetOutputDataReverseMode(crc_, LL_CRC_OUTDATA_REVERSE_NONE);
    }

    void update(uint8_t byte) const
    {
        LL_CRC_FeedData8(crc_, byte);
    }

    [[nodiscard]] uint16_t value() const
    {
        return LL_CRC_ReadData16(crc_);
    }

    void reset() const
    {
        LL_CRC_ResetCRCCalculationUnit(crc_);
    }

private:
    CRC_TypeDef *crc_{nullptr};
};

} // namespace fumi

#endif //GARNET_CRC16_H

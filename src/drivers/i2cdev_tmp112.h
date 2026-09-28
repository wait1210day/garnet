// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_I2CDEV_TMP112_H
#define GARNET_I2CDEV_TMP112_H

#include <cstdint>

#include "drivers/i2c_registers.h"

namespace tmp112
{

namespace addr
{
    // address of the device when `ADD0` pin connects to:
    constexpr static uint8_t kGnd = 0x48;
    constexpr static uint8_t kVcc = 0x49;
    constexpr static uint8_t kSda = 0x4A;
    constexpr static uint8_t kScl = 0x4B;
}; // namespace addr

enum class AlertPolarity : uint8_t
{
    kActiveLow = 0,
    kActiveHigh = 1
};

enum class ThermostatMode : uint8_t
{
    kComparator = 0,
    kInterrupt = 1
};

enum class ShutdownMode : uint8_t
{
    kContinuous = 0,
    kShutdown = 1
};

enum class ExtendedMode : uint8_t
{
    kNormal12Bit = 0,   // -55°C ~ +128°C
    kExtended13Bit = 1  // -55°C ~ +150°C
};

enum class ConvRate : uint8_t
{
    k0_25Hz = 0b00,
    k1Hz    = 0b01,
    // default after POR
    k4Hz    = 0b10,
    k8Hz    = 0b11
};

enum class FaultQueue : uint8_t
{
    k1Fault  = 0b00,
    k2Faults = 0b01,
    k4Faults = 0b10,
    k6Faults = 0b11
};

constexpr float kResolution = 0.0625f;


// temperature register
struct TMPR : i2c::Register<0x00, 1, uint16_t, 2, i2c::RO, i2c::BE>
{
    // Normal mode 12-bit temperature
    using Value12 = i2c::Field<TMPR, 4, 12>;
    // Extended mode 13-bit temperature
    using Value13 = i2c::Field<TMPR, 3, 13>;
    // Extended mode flag
    using EM = i2c::Field<TMPR, 0, 1>;
};

// configuration register
struct CR : i2c::Register<0x01, 1, uint16_t, 2, i2c::RW, i2c::BE>
{
    // One-shot start/ready flasg
    using OS   = i2c::Field<CR, 15, 1>;
    // ADC resolution, power on default 11b
    using RES  = i2c::Field<CR, 13, 2>;
    // fault queue
    using FQ   = i2c::Field<CR, 11, 2, FaultQueue>;
    // alert polarity
    using POL  = i2c::Field<CR, 10, 1, AlertPolarity>;
    // comparator/interrupt mode
    using TM   = i2c::Field<CR, 9, 1, ThermostatMode>;
    // shutdown mode
    using SD   = i2c::Field<CR, 8, 1, ShutdownMode>;

    // conversion rate
    using RATE = i2c::Field<CR, 6, 2, ConvRate>;
    // alert status
    using AL   = i2c::Field<CR, 5, 1>;
    // extended mode enabled
    using EM   = i2c::Field<CR, 4, 1, ExtendedMode>;
};

// Tlow limit register, default +75°C
struct TLOW : i2c::Register<0x02, 1, uint16_t, 2, i2c::RW, i2c::BE>
{
    using Value12 = i2c::Field<TLOW, 4, 12>;
    using Value13 = i2c::Field<TLOW, 3, 13>;
};

// Thigh limit register, default +80°C
struct THIGH : i2c::Register<0x03, 1, uint16_t, 2, i2c::RW, i2c::BE>
{
    using Value12 = i2c::Field<THIGH, 4, 12>;
    using Value13 = i2c::Field<THIGH, 3, 13>;
};


[[nodiscard]] constexpr float raw_to_celsius_12bit(uint16_t raw)
{
    return static_cast<float>(static_cast<int16_t>(raw) >> 4) * kResolution;
}

[[nodiscard]] constexpr float raw_to_celsius_13bit(uint16_t raw)
{
    return static_cast<float>(static_cast<int16_t>(raw) >> 3) * kResolution;
}

[[nodiscard]] constexpr uint16_t celsius_to_raw_12bit(float celsius)
{
    return static_cast<uint16_t>(static_cast<int16_t>(celsius / kResolution) << 4);
}

[[nodiscard]] constexpr uint16_t celsius_to_raw_13bit(float celsius)
{
    return static_cast<uint16_t>(static_cast<int16_t>(celsius / kResolution) << 3);
}

} // namespace tmp112

#endif // GARNET_I2CDEV_TMP112_H

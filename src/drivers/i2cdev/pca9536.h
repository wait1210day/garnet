// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_I2CDEV_PCA9536_H
#define GARNET_I2CDEV_PCA9536_H

#include <cstdint>

#include "drivers/i2c_registers.h"

namespace pca9536
{

constexpr uint8_t kDevAddr = 0x41;

enum class Level : uint8_t
{
    kLow = 0,
    kHigh = 1
};

enum class Dir : uint8_t
{
    kOut = 0,
    kIn = 1
};

// Input Data Register
struct IDR : i2c::Register<0x00, 1, uint8_t, 1, i2c::RO, i2c::BE>
{
    using P0 = i2c::Field<IDR, 0, 1, Level>;
    using P1 = i2c::Field<IDR, 1, 1, Level>;
    using P2 = i2c::Field<IDR, 2, 1, Level>;
    using P3 = i2c::Field<IDR, 3, 1, Level>;
    using Px = i2c::Field<IDR, 0, 4>;
};

// Output Data Register
struct ODR : i2c::Register<0x01, 1, uint8_t, 1, i2c::RW, i2c::BE>
{
    using P0 = i2c::Field<ODR, 0, 1, Level>;
    using P1 = i2c::Field<ODR, 1, 1, Level>;
    using P2 = i2c::Field<ODR, 2, 1, Level>;
    using P3 = i2c::Field<ODR, 3, 1, Level>;
    using Px = i2c::Field<ODR, 0, 4>;
};

// Input Polarity Inversion
struct IPI : i2c::Register<0x02, 1, uint8_t, 1, i2c::RW, i2c::BE>
{
    using P0 = i2c::Field<IPI, 0, 1>;
    using P1 = i2c::Field<IPI, 1, 1>;
    using P2 = i2c::Field<IPI, 2, 1>;
    using P3 = i2c::Field<IPI, 3, 1>;
    using Px = i2c::Field<IPI, 0, 4>;
};

// Configuration Register
struct CR : i2c::Register<0x03, 1, uint8_t, 1, i2c::RW, i2c::BE>
{
    using P0 = i2c::Field<CR, 0, 1, Dir>;
    using P1 = i2c::Field<CR, 1, 1, Dir>;
    using P2 = i2c::Field<CR, 2, 1, Dir>;
    using P3 = i2c::Field<CR, 3, 1, Dir>;
};

} // namespace pca9536

#endif //GARNET_I2CDEV_PCA9536_H

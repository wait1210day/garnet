// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_I2C_COMMAND_H
#define GARNET_I2C_COMMAND_H

#include <cstdint>
#include <cstddef>
#include <span>
#include <type_traits>
#include <expected>

#include "drivers/i2c_bus.h"

namespace i2c
{

template<typename T>
concept TxCommand = requires(const T& cmd, std::span<uint8_t> dst)
{
    { T::kMaxTxBytes } -> std::convertible_to<size_t>;
    { cmd.serialize(dst) } -> std::same_as<std::expected<size_t, Err>>;
};

template<typename T>
concept RxFrame = requires(T& resp, std::span<const uint8_t> src)
{
    { T::kRxBytes } -> std::convertible_to<size_t>;
    { resp.deserialize(src) } -> std::same_as<Err>;
};

} // namespace i2c

#endif //GARNET_I2C_COMMAND_H

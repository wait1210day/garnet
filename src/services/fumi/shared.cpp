// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include "services/fumi/shared.h"

namespace fumi
{

void SharedResources::init(const Config& config)
{
    crc_mutex_.init();
    crc_.init(config.ll_crc);
    uart_tx_ = config.uart_tx;
}

std::optional<uint16_t>
SharedResources::compute_crc16(TickType_t timeout, std::span<const uint8_t> data) const
{
    if (!crc_mutex_.try_lock_for(timeout))
        return std::nullopt;

    crc_.reset();
    for (uint8_t byte : data)
        crc_.update(byte);

    uint16_t result = crc_.value();
    crc_mutex_.unlock();
    return result;
}

} // namespace fumi

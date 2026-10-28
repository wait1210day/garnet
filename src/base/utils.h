// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_UTILS_H
#define GARNET_UTILS_H

#include <cstddef>
#include <cstdint>
#include <span>

bool check_build_id_sha1();
std::span<const uint8_t, 20> get_build_id_sha1();

#endif //GARNET_UTILS_H

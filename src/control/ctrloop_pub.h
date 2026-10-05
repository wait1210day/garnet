// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_CTRLOOP_PUB_H
#define GARNET_CTRLOOP_PUB_H

#include <cstdint>


namespace ctrl
{

enum class OpRegion : uint8_t
{
    kIdle,
    kBuck,
    kBoost,
    kBoostWithBuckAux,
    kBuckWithBoostAux
};

enum class OutputState : uint8_t
{
    kOn,
    kOff
};

struct TargetSet
{
    OutputState outstate;
    float vout;
    float iout;
};

} // namespace ctrl

#endif //GARNET_CTRLOOP_PUB_H

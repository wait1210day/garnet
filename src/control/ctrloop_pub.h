// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_CTRLOOP_PUB_H
#define GARNET_CTRLOOP_PUB_H

#include <cstdint>


enum class CtrlOpRegion : uint8_t
{
    kIdle,
    kBuck,
    kBoost,
    kBoostWithBuckAux,
    kBuckWithBoostAux
};

enum class CtrlOutputState : uint8_t
{
    kOn,
    kOff
};

struct CtrlTargetSet
{
    CtrlOutputState outstate;
    float vout;
    float iout;
};

#endif //GARNET_CTRLOOP_PUB_H

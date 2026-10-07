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
    kOff,
    // Normal mode, output is regulated.
    kRegulated,
    // Output is directly connected to the input, Vin=Vout,
    // not regulated. Protections are enabled.
    kPassthrough
};

struct TargetSet
{
    OutputState outstate;
    float vout;
    float iout;
};

enum class ControlState
{
    kUnmanaged,
    kOperational
};

struct Measures
{
    ControlState ctrl_state = ControlState::kUnmanaged;
    OpRegion op_region = OpRegion::kIdle;
    OutputState outstate = OutputState::kOff;
    float v1 = 0;
    float v2 = 0;
    float i1 = 0;
    float i2 = 0;
    float iref = 0;
};

} // namespace ctrl

#endif //GARNET_CTRLOOP_PUB_H

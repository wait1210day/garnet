// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_HERMES_MESSAGES_H
#define GARNET_HERMES_MESSAGES_H

#include "control/ctrloop_pub.h"
#include "services/hermes.h"

namespace msg
{

enum TopicId
{
    kSetpoint,
    kMeasures
};

struct Setpoint
{
    using Topic = hermes::Topic<TopicId::kSetpoint, Setpoint>;

    ctrl::OutputState outstate;
    float vout;
    float iout;
};

struct Measures
{
    using Topic = hermes::Topic<TopicId::kMeasures, Measures>;

    float vout_set;
    float iout_set;
    float vin;
    float iin;
    float vout;
    float iout;
    float eff;
    float temp1;
    float temp2;
    float coretemp;
    float vdd_5v;
    ctrl::OpRegion topo_op_region;
};

} // namespace msg

#endif //GARNET_HERMES_MESSAGES_H

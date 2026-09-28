// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_HERMES_MESSAGES_H
#define GARNET_HERMES_MESSAGES_H

#include "services/hermes.h"

namespace msg
{

enum TopicId
{
    kTest
};

struct Test
{
    using Topic = hermes::Topic<TopicId::kTest, Test>;

    uint32_t num;
};

} // namespace msg

#endif //GARNET_HERMES_MESSAGES_H

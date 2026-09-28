// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include "services/hermes.h"

namespace hermes
{

namespace
{

Bus<> g_default_bus;

} // namespace anonymous

Bus<>& get_default_bus()
{
    return g_default_bus;
}

} // namespace hermes

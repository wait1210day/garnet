// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_STATIC_RESOURCE_H
#define GARNET_STATIC_RESOURCE_H

enum class StaticInitOption
{
    // Defers initialization.
    // Constructor will NOT initialize the internal FreeRTOS structure,
    // and the user is required to call `.init()` method before using.
    // Typically used for global declared instance.
    kDefer,

    // Initialize immediately.
    // Constructor will init the internal FreeRTOS structure.
    // Typically used for stack allocated instance.
    kImm
};

#endif //GARNET_STATIC_RESOURCE_H

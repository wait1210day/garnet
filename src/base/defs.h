// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_DEFS_H
#define GARNET_DEFS_H

#define GARNET_NORETURN       [[noreturn]]
#define GARNET_MAYBE_UNUSED   [[maybe_unused]]
#define GARNET_ALWAYS_INLINE  [[gnu::always_inline]]

#define GARNET_CCMRAM_TEXT    [[gnu::section(".ccmram.text")]]
#define GARNET_CCMRAM_DATA    [[gnu::section(".ccmram.data")]]

#define GARNET_NON_COPYABLE(type) \
    type(const type&) = delete; \
    type& operator=(const type&) = delete;

#define GARNET_NON_MOVABLE(type) \
    type(type&&) = delete; \
    type& operator=(type&&) = delete;

#endif //GARNET_DEFS_H

// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_DEFS_H
#define GARNET_DEFS_H

#define GARNET_NORETURN       __attribute__((__noreturn__))
#define GARNET_MAYBE_UNUSED   __attribute__((__unused__))
#define GARNET_ALWAYS_INLINE  __attribute__((__always_inline__))

#define GARNET_CCMRAM_TEXT    __attribute__((__section__(".ccmram.text")))
#define GARNET_CCMRAM_DATA    __attribute__((__section__(".ccmram.data")))

#define GARNET_NON_COPYABLE(type) \
    type(const type&) = delete; \
    type& operator=(const type&) = delete;

#define GARNET_NON_MOVABLE(type) \
    type(type&&) = delete; \
    type& operator=(type&&) = delete;

#endif //GARNET_DEFS_H

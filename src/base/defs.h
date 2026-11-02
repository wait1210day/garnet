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

// Delayed recursion needs repeated rescanning. This fixed expansion budget
// supports at least 64 arguments, including the empty list (C++20 or later).
#define GARNET_PP_DETAIL_PARENS ()
#define GARNET_PP_DETAIL_EXPAND4(...) __VA_ARGS__
#define GARNET_PP_DETAIL_EXPAND3(...) \
    GARNET_PP_DETAIL_EXPAND4(GARNET_PP_DETAIL_EXPAND4( \
        GARNET_PP_DETAIL_EXPAND4(GARNET_PP_DETAIL_EXPAND4(__VA_ARGS__))))
#define GARNET_PP_DETAIL_EXPAND2(...) \
    GARNET_PP_DETAIL_EXPAND3(GARNET_PP_DETAIL_EXPAND3( \
        GARNET_PP_DETAIL_EXPAND3(GARNET_PP_DETAIL_EXPAND3(__VA_ARGS__))))
#define GARNET_PP_DETAIL_EXPAND1(...) \
    GARNET_PP_DETAIL_EXPAND2(GARNET_PP_DETAIL_EXPAND2( \
        GARNET_PP_DETAIL_EXPAND2(GARNET_PP_DETAIL_EXPAND2(__VA_ARGS__))))
#define GARNET_PP_DETAIL_EXPAND(...) \
    GARNET_PP_DETAIL_EXPAND1(GARNET_PP_DETAIL_EXPAND1( \
        GARNET_PP_DETAIL_EXPAND1(GARNET_PP_DETAIL_EXPAND1(__VA_ARGS__))))

// Apply macro(arg) to each argument; macro supplies any separators it needs.
// Expressions containing unparenthesized commas must be parenthesized.
#define GARNET_PP_FOR_EACH(macro, ...) \
    __VA_OPT__(GARNET_PP_DETAIL_EXPAND(GARNET_PP_DETAIL_FOR_EACH(macro, __VA_ARGS__)))
#define GARNET_PP_DETAIL_FOR_EACH(macro, arg, ...) \
    macro(arg) __VA_OPT__(GARNET_PP_DETAIL_FOR_EACH_AGAIN GARNET_PP_DETAIL_PARENS (macro, __VA_ARGS__))
#define GARNET_PP_DETAIL_FOR_EACH_AGAIN() GARNET_PP_DETAIL_FOR_EACH

#endif //GARNET_DEFS_H

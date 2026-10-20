// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_FAST_LL_H
#define GARNET_FAST_LL_H

#include <bit>
#include <cstdint>

#include "stm32g4xx_ll_hrtim.h"

namespace ctrl::fastll
{

// Resolve LL identifiers at compile time, avoiding POSITION_VAL's volatile
// RBIT instructions and Flash offset tables. The register writes retain the
// corresponding STM32G4 LL semantics; callers provide their own CCMRAM placement.
namespace detail
{

template<uint32_t kTimer>
consteval uint32_t timer_index()
{
    static_assert(std::has_single_bit(kTimer) && (kTimer & LL_HRTIM_TIMER_X) == kTimer,
                  "expected one HRTIM timer A-F");
    return std::countr_zero(kTimer) - HRTIM_MCR_TACEN_Pos;
}

template<uint32_t kOutput>
consteval uint32_t output_index()
{
    static_assert(std::has_single_bit(kOutput)
                  && kOutput >= LL_HRTIM_OUTPUT_TA1 && kOutput <= LL_HRTIM_OUTPUT_TF2,
                  "expected one HRTIM output TA1-TF2");
    return std::countr_zero(kOutput) - std::countr_zero(uint32_t{LL_HRTIM_OUTPUT_TA1});
}

template<uint32_t kTimer, uint64_t kTrigger>
consteval uint32_t capture_config()
{
    // LL encodes common events in the low word and timer events in the high
    // word. Its low nibble occupies the selected timer's four-bit event field.
    constexpr uint32_t shift = 12U + 4U * timer_index<kTimer>();
    constexpr uint32_t other_timer_mask = uint32_t{0xfffff000U}
        & ~static_cast<uint32_t>(uint64_t{0xfU} << shift);
    constexpr uint32_t other_timer_events = static_cast<uint32_t>(kTrigger >> 32)
        & other_timer_mask;
    // Use a 64-bit shift so timer F's absent own-event field (shift 32) is
    // discarded without an undefined 32-bit shift.
    constexpr uint32_t own_timer_events = static_cast<uint32_t>(
        ((kTrigger >> 32) & 0xfU) << shift);
    return static_cast<uint32_t>(kTrigger & 0xfffU) | other_timer_events | own_timer_events;
}

} // namespace detail

template<uint32_t kTimer>
[[gnu::always_inline]]
inline void hrtim_set_compare1(HRTIM_TypeDef *hrtim, uint32_t value)
{
    if constexpr (kTimer == LL_HRTIM_TIMER_MASTER)
        MODIFY_REG(hrtim->sMasterRegs.MCMP1R, HRTIM_MCMP1R_MCMP1R, value);
    else
        MODIFY_REG(hrtim->sTimerxRegs[detail::timer_index<kTimer>()].CMP1xR,
                   HRTIM_MCMP1R_MCMP1R, value);
}

template<uint32_t kTimer>
[[gnu::always_inline]]
inline void hrtim_set_compare2(HRTIM_TypeDef *hrtim, uint32_t value)
{
    if constexpr (kTimer == LL_HRTIM_TIMER_MASTER)
        MODIFY_REG(hrtim->sMasterRegs.MCMP2R, HRTIM_MCMP1R_MCMP2R, value);
    else
        MODIFY_REG(hrtim->sTimerxRegs[detail::timer_index<kTimer>()].CMP2xR,
                   HRTIM_MCMP1R_MCMP2R, value);
}

template<uint32_t kTimer>
[[gnu::always_inline]]
inline void hrtim_set_compare3(HRTIM_TypeDef *hrtim, uint32_t value)
{
    if constexpr (kTimer == LL_HRTIM_TIMER_MASTER)
        MODIFY_REG(hrtim->sMasterRegs.MCMP3R, HRTIM_MCMP1R_MCMP3R, value);
    else
        MODIFY_REG(hrtim->sTimerxRegs[detail::timer_index<kTimer>()].CMP3xR,
                   HRTIM_MCMP1R_MCMP3R, value);
}

template<uint32_t kTimer>
[[gnu::always_inline]]
inline void hrtim_set_compare4(HRTIM_TypeDef *hrtim, uint32_t value)
{
    if constexpr (kTimer == LL_HRTIM_TIMER_MASTER)
        MODIFY_REG(hrtim->sMasterRegs.MCMP4R, HRTIM_MCMP1R_MCMP4R, value);
    else
        MODIFY_REG(hrtim->sTimerxRegs[detail::timer_index<kTimer>()].CMP4xR,
                   HRTIM_MCMP1R_MCMP4R, value);
}

template<uint32_t kTimer, uint32_t kUnit, uint64_t kTrigger>
[[gnu::always_inline]]
inline void hrtim_set_capture_trigger(HRTIM_TypeDef *hrtim)
{
    static_assert(kUnit == LL_HRTIM_CAPTUREUNIT_1 || kUnit == LL_HRTIM_CAPTUREUNIT_2,
                  "expected HRTIM capture unit 1 or 2");
    constexpr uint32_t index = detail::timer_index<kTimer>();
    constexpr uint32_t config = detail::capture_config<kTimer, kTrigger>();
    if constexpr (kUnit == LL_HRTIM_CAPTUREUNIT_1)
        WRITE_REG(hrtim->sTimerxRegs[index].CPT1xCR, config);
    else
        WRITE_REG(hrtim->sTimerxRegs[index].CPT2xCR, config);
}

template<uint32_t kOutput>
[[gnu::always_inline]]
inline void hrtim_set_output_set_source(HRTIM_TypeDef *hrtim, uint32_t source)
{
    constexpr uint32_t output = detail::output_index<kOutput>();
    if constexpr (output % 2U == 0)
        WRITE_REG(hrtim->sTimerxRegs[output / 2U].SETx1R, source);
    else
        WRITE_REG(hrtim->sTimerxRegs[output / 2U].SETx2R, source);
}

template<uint32_t kOutput>
[[gnu::always_inline]]
inline void hrtim_set_output_reset_source(HRTIM_TypeDef *hrtim, uint32_t source)
{
    constexpr uint32_t output = detail::output_index<kOutput>();
    if constexpr (output % 2U == 0)
        WRITE_REG(hrtim->sTimerxRegs[output / 2U].RSTx1R, source);
    else
        WRITE_REG(hrtim->sTimerxRegs[output / 2U].RSTx2R, source);
}

} // namespace ctrl::fastll

#endif //GARNET_FAST_LL_H

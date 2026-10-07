// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_LL_HELPER_H
#define GARNET_LL_HELPER_H

#include <type_traits>
#include <span>

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_hrtim.h"

#include "base/err.h"
#include "base/panic.h"

// Very thing wrapper for LL library, a stateless version of HAL.

namespace llhelper
{

// G4 DMA supports a byte (1), a half word (2), and a word (4) per transfer.
constexpr static size_t kMaxDmaBytesPerTransfer = 4;

ErrStatus adc_disable(ADC_TypeDef *adc);
ErrStatus adc_enable(ADC_TypeDef *adc);

ErrStatus adc_start_calibration(ADC_TypeDef *adc, uint32_t single_diff);

ErrStatus adc_start_dma_no_irq(ADC_TypeDef *adc, DMA_TypeDef *dma,
                           uint32_t dma_channel, uint32_t dma_transfer_mode,
                           uint8_t *addr, uint32_t count);

template<typename TransferWord, size_t Extent = std::dynamic_extent>
    requires(!std::is_const_v<TransferWord> && sizeof(TransferWord) <= kMaxDmaBytesPerTransfer)
ErrStatus adc_start_dma_no_irq(ADC_TypeDef *adc, DMA_TypeDef *dma,
                           uint32_t dma_channel, uint32_t dma_transfer_mode,
                           std::span<TransferWord, Extent> dmabuf)
{
    return adc_start_dma_no_irq(adc, dma, dma_channel, dma_transfer_mode,
                                reinterpret_cast<uint8_t*>(dmabuf.data()), dmabuf.size());
}

ErrStatus adc_start_injected_irq(ADC_TypeDef *adc, uint8_t jeoc, uint8_t jeos);

// Clear DMA interrupt flags (HTx, TEx, TCx) for a specific channel.
inline void dma_clear_intr_flags(DMA_TypeDef *dma, uint32_t channel)
{
    // G4 LL channel identifiers are 0-based; each channel has four flags
    static_assert(LL_DMA_CHANNEL_1 == 0, "dma channel number must start from 0");
    LL_DMA_WriteReg(dma, IFCR, DMA_IFCR_CGIF1 << (4 * channel));
}

// Get DMA interrupt flags (HTx, TEx, TCx) for a specific channel
inline uint32_t dma_get_intr_flags(DMA_TypeDef *dma, uint32_t channel)
{
    static_assert(LL_DMA_CHANNEL_1 == 0, "dma channel number must start from 0");
    return LL_DMA_ReadReg(dma, ISR) >> (4 * channel);
}

inline bool hrtim_has_intr_fault_flag(HRTIM_TypeDef *hrtim, uint32_t fault_num)
{
    static_assert(LL_HRTIM_FAULT_1 == HRTIM_ISR_FLT1
                  && LL_HRTIM_FAULT_2 == HRTIM_ISR_FLT2
                  && LL_HRTIM_FAULT_3 == HRTIM_ISR_FLT3
                  && LL_HRTIM_FAULT_4 == HRTIM_ISR_FLT4
                  && LL_HRTIM_FAULT_5 == HRTIM_ISR_FLT5,
                  "LL_HRTIM_FAULT_x definition not satisfies the assumption");

    // `LL_HRTIM_FAULT_6 != HRTIM_ISR_FLT6` since the `HRTIM_ISR[5]` bit
    // is occupied by SYSFLT instead of FLT6.
    if (fault_num == LL_HRTIM_FAULT_6)
        return LL_HRTIM_IsActiveFlag_FLT6(hrtim);

    return (hrtim->sCommonRegs.ISR & fault_num) == fault_num;
}

inline void hrtim_clear_intr_fault_flag(HRTIM_TypeDef *hrtim, uint32_t fault_num)
{
    static_assert(LL_HRTIM_FAULT_1 == HRTIM_ICR_FLT1C
                  && LL_HRTIM_FAULT_2 == HRTIM_ICR_FLT2C
                  && LL_HRTIM_FAULT_3 == HRTIM_ICR_FLT3C
                  && LL_HRTIM_FAULT_4 == HRTIM_ICR_FLT4C
                  && LL_HRTIM_FAULT_5 == HRTIM_ICR_FLT5C,
                  "LL_HRTIM_FAULT_x definition not satisfies the assumption");

    // `LL_HRTIM_FAULT_6 != HRTIM_ICR_FLT6C` since the `HRTIM_ICR[5]` bit
    // is occupied by SYSFLT instead of FLT6.
    if (fault_num == LL_HRTIM_FAULT_6)
        return LL_HRTIM_ClearFlag_FLT6(hrtim);

    hrtim->sCommonRegs.ICR |= fault_num;
}

} // namespace llhelper

#endif //GARNET_LL_HELPER_H

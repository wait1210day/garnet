// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include "stm32g4xx_ll_adc.h"

#include "base/panic.h"
#include "drivers/ll_helper.h"

// Reference: ST HAL library

namespace
{

constexpr uint32_t kAdcDisableTimeout = 2;

// Fixed timeout value for ADC calibration.
// Values defined to be higher than the worst case: low clock frequency
// and maximum prescalers.
// See also: `ADC_CALIBRATION_TIMEOUT` in `stm32g4xx_hal_adc_ex.c`.
constexpr uint32_t kAdcCaliTimeout = 318978;

} // namespace anonymous


namespace llhelper
{

ErrStatus adc_disable(ADC_TypeDef *adc)
{
    if (!LL_ADC_IsEnabled(adc) || LL_ADC_IsDisableOngoing(adc))
        return ErrStatus::kOk;

    LL_ADC_Disable(adc);

    // Wait for ADC effectively disabled with a given timeout
    uint32_t tickstart = HAL_GetTick();
    while ((adc->CR & ADC_CR_ADEN) != 0ul)
    {
        if ((HAL_GetTick() - tickstart) > kAdcDisableTimeout)
        {
            // Check again to avoid false timeout detection in case of preemption
            if ((adc->CR & ADC_CR_ADEN) != 0ul)
                return ErrStatus::kFail;
        }
    }

    return ErrStatus::kOk;
}

ErrStatus adc_enable(ADC_TypeDef *adc)
{
    if (LL_ADC_IsEnabled(adc) == 0)
    {
        LL_ADC_ClearFlag_ADRDY(adc);
        LL_ADC_Enable(adc);
        while (LL_ADC_IsActiveFlag_ADRDY(adc) == 0)
            ;
    }
    return ErrStatus::kOk;
}

ErrStatus adc_start_calibration(ADC_TypeDef *adc, uint32_t single_diff)
{
    if (adc_disable(adc) != ErrStatus::kOk)
        return ErrStatus::kFail;

    // Enable the ADC voltage regulator and wait for it becoming stable.
    LL_ADC_EnableInternalRegulator(adc);
    HAL_Delay(10);

    LL_ADC_StartCalibration(adc, single_diff);

    // Wait for calibration completion
    uint32_t loop_count = 0;
    while (LL_ADC_IsCalibrationOnGoing(adc) != 0ul)
    {
        loop_count++;
        if (loop_count >= kAdcCaliTimeout)
            return ErrStatus::kFail;
    }
    return ErrStatus::kOk;
}


ErrStatus adc_start_dma_no_irq(ADC_TypeDef *adc,
                                     DMA_TypeDef *dma,
                                     uint32_t dma_channel,
                                     uint32_t dma_transfer_mode,
                                     uint8_t *addr,
                                     uint32_t count)
{
    LL_DMA_DisableChannel(dma, dma_channel);

    dma_clear_intr_flags(dma, dma_channel);

    LL_DMA_SetMemoryAddress(dma, dma_channel, (uint32_t) addr);
    LL_DMA_SetPeriphAddress(
        dma, dma_channel, LL_ADC_DMA_GetRegAddr(adc, LL_ADC_DMA_REG_REGULAR_DATA));
    LL_DMA_SetDataLength(dma, dma_channel, count);

    LL_DMA_EnableChannel(dma, dma_channel);

    LL_ADC_ClearFlag_ADRDY(adc);
    LL_ADC_REG_SetDMATransfer(adc, dma_transfer_mode);
    LL_ADC_Enable(adc);

    while (LL_ADC_IsActiveFlag_ADRDY(adc) == 0ul)
    {
        // Just wait for ADC ready
    }
    LL_ADC_REG_StartConversion(adc);

    return ErrStatus::kOk;
}

ErrStatus adc_start_injected_irq(ADC_TypeDef *adc, uint8_t jeoc, uint8_t jeos)
{
    LL_ADC_ClearFlag_JEOC(adc);
    LL_ADC_ClearFlag_JEOS(adc);

    // Enable the interruption of injected END of conversion
    if (jeos)
        LL_ADC_EnableIT_JEOS(adc);
    if (jeoc)
        LL_ADC_EnableIT_JEOC(adc);

    LL_ADC_INJ_StartConversion(adc);
    return ErrStatus::kOk;
}

} // namespace llhelper

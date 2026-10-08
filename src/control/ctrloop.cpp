// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

// The modulation strategy and hysteresis boundary logic implemented herein are
// independently developed and inspired by publicly available documentation of
// synchronous 4-switch buck-boost architectures (specifically referencing the
// operational principles described in Analog Devices / Linear Technology LT8390 datasheet).
//
// All code is an original software implementation written for STM32G474 HRTIM.
//
// "LT8390" and related terms are trademarks of Analog Devices, Inc.
// This project is an independent open-source effort and is NOT affiliated with,
// sponsored, or endorsed by Analog Devices, Inc.

#include <cstring>
#include <cmath>

#include "stm32g4xx_ll_dac.h"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"

#include "base/defs.h"
#include "base/cali.h"
#include "drivers/ll_helper.h"
#include "control/laplace.h"
#include "control/ctrloop.h"
#include "services/fumi.h"


#define OUT_L_HG    LL_HRTIM_OUTPUT_TB1
#define OUT_L_LG    LL_HRTIM_OUTPUT_TB2
#define OUT_R_HG    LL_HRTIM_OUTPUT_TD2
#define OUT_R_LG    LL_HRTIM_OUTPUT_TD1
#define OUT_ALL     (OUT_L_LG | OUT_L_HG | OUT_R_LG | OUT_R_HG)
#define TIMER_L     LL_HRTIM_TIMER_B
#define TIMER_R     LL_HRTIM_TIMER_D


namespace ctrl
{

constexpr uint32_t kTimerCounterPeriod = 10880;
constexpr float kCvRampingStep = 0.005f;

// Default HRTIM compare unit values
namespace cmpval
{
    // Switching sequence:
    //                   (master cmp3)                                     (new period)
    //     COUNTER   0     kDeadtime1    EEV4/CMP3  EEV4/CMP3+kDeadtime2    0...
    //  switch_out   x         S             R               x              x
    //  syncre_out   R         x             x               S              R
    //              (1)       (2)           (3)             (4)            (5)
    //
    // figure: x - not changed
    //         S - output is set (turn on)
    //         R - output is reset (turn off)
    //
    // 1. Start of a new period, `syncre_out` is reset by the PER event
    // 2. `switch_out` is set by master compare 3 (=kDeadtime1)
    // 3. `switch_out` is reset by EEV4 or CMP3, which has different semantics
    //    according to the leg's mode:
    //      - peak-current mode: EEV4 is the peak-current event, and CMP3 will
    //        reset the output if EEV4 does not happen before CMP3. I.e. CMP3
    //        is the maximum duty cycle limit.
    //      - fixed-duty mode: EEV4 is disabled, and CMP3 determines the duty
    //        cycle.
    // 4. `syncre_out` is reset by HRTIM auto-delay mechanism (=EEV4+CMP4(=kDeadtime2)
    //    if EEV4 is enabled, or =CMP3+CMP4 if EEV4 is disabled.
    //

    constexpr uint32_t kDeadtime1 = 120;

    // Minimum duty-cycle in FIXED-DUTY mode is limited by deadtime1
    constexpr uint32_t kMinSwitchDuty = 240;

    constexpr uint32_t kDeadtime2 = 120;
    constexpr uint32_t kBlanking = 600;
    constexpr uint32_t kMaxDuty = 10200;
    // DAC sawtooth generation trigger.
    // HRTIM hardware treats this value as an interval time of sawtooth step.
    // e.g. if set to 100, sawtooth step event will be triggered when the timer's
    // counter reaches 100, 200, 300, 400, ...
    constexpr uint32_t kSawtooth = 250;

} // namespace cmpval

constexpr uint32_t kMeasUpdatePrescaler = 200;

constexpr float kInductorNominalUh = 3.3f;

// 184ps * counter_period = Tsw
constexpr float kFswHz = (1.0f / (0.184f * kTimerCounterPeriod)) * 1000'000'000.0f;
constexpr float kFsampleHz = 250'000;


#define INISR GARNET_ALWAYS_INLINE inline

constexpr const char *kTAG = "ctrloop";

constexpr laplace::DiscreteComp kDefaultBuckCvCompCoeffs =
    laplace::ContinuousParams::from_freqs(5, 80, 726000)
    .solve_2p2z(kFsampleHz)
    .cast_to<float>();

constexpr laplace::DiscreteComp kDefaultBoostCvCompCoeffs =
    laplace::ContinuousParams::from_freqs(3, 80, 555000)
    .solve_2p2z(kFsampleHz)
    .cast_to<float>();

namespace
{
    INISR float iir_2p2z_update(Iir2p2zComp *comp, const laplace::DiscreteComp<float>& coeffs,
                                float err, float sat_min, float sat_max)
    {
        float acc = coeffs.b0 * err
                  + coeffs.b1 * comp->e_n1
                  + coeffs.b2 * comp->e_n2
                  + coeffs.a1 * comp->y_n1
                  + coeffs.a2 * comp->y_n2;

        if (acc < sat_min)
            acc = sat_min;
        else if (acc > sat_max)
            acc = sat_max;

        comp->e_n2 = comp->e_n1;
        comp->e_n1 = err;
        comp->y_n2 = comp->y_n1;
        comp->y_n1 = acc;

        return acc;
    }

    GARNET_CCMRAM_TEXT void iir_2p2z_reset(Iir2p2zComp *comp)
    {
        comp->e_n1 = 0;
        comp->e_n2 = 0;
        comp->y_n1 = 0;
        comp->y_n2 = 0;
    }

    INISR void update_peak_current_sawtooth(Ctrloop::FastRamContext *ctx, float peak_iref, float dv_fall)
    {
        const float k = ctx->cali.il_inv.k, b = ctx->cali.il_inv.b;

        if (dv_fall < 0)
            dv_fall = 0;

        ctx->slope_dv_fall_filtered = 0.8f * ctx->slope_dv_fall_filtered
                                      + 0.2f * dv_fall;

        auto reset_val_raw = static_cast<int32_t>(peak_iref * k + b);
        if (reset_val_raw >= 4095)
            reset_val_raw = 4095;
        if (reset_val_raw <= 0)
            reset_val_raw = 0;

        auto step_val_raw = static_cast<int32_t>(
            ctx->slope_dv_fall_filtered * ctx->slope_dac_step_raw_per_volt);
        if (step_val_raw >= 0xffff)
            step_val_raw = 0xffff;
        if (step_val_raw <= 16)
            step_val_raw = 16;

        LL_DAC_SetWaveSawtoothResetData(ctx->ll.dac_pcmc, LL_DAC_CHANNEL_1, reset_val_raw);
        LL_DAC_SetWaveSawtoothStepData(ctx->ll.dac_pcmc, LL_DAC_CHANNEL_1, step_val_raw);
    }

    template<bool kDiodeEmulation = false>
    GARNET_CCMRAM_TEXT void hrtim_config_peak_curr_mode_leg(HRTIM_TypeDef *hrtim, uint32_t timer,
                                                            uint32_t out_switch, uint32_t out_syncre)
    {
        LL_HRTIM_TIM_SetCompare3(hrtim, timer, cmpval::kMaxDuty);

        LL_HRTIM_OUT_SetOutputSetSrc(hrtim, out_switch, LL_HRTIM_OUTPUTSET_MASTERCMP3);
        LL_HRTIM_OUT_SetOutputResetSrc(hrtim, out_switch, LL_HRTIM_OUTPUTRESET_TIMCMP3 | LL_HRTIM_OUTPUTRESET_EEV_4);
        LL_HRTIM_OUT_SetOutputSetSrc(hrtim, out_syncre, LL_HRTIM_OUTPUTSET_TIMCMP4);

        uint32_t reset_source = LL_HRTIM_OUTPUTRESET_TIMPER;
        if constexpr (kDiodeEmulation)
            reset_source |= LL_HRTIM_OUTPUTRESET_EEV_2;
        LL_HRTIM_OUT_SetOutputResetSrc(hrtim, out_syncre, reset_source);

        // enable peak-current event
        LL_HRTIM_TIM_SetCaptureTrig(hrtim, timer, LL_HRTIM_CAPTUREUNIT_2, LL_HRTIM_CAPTURETRIG_EEV_4);
    }

    GARNET_CCMRAM_TEXT void hrtim_config_fixed_duty_leg(HRTIM_TypeDef *hrtim, uint32_t timer,
                                                        uint32_t out_switch, uint32_t out_syncre,
                                                        uint32_t duty_cmp_value)
    {
        if (duty_cmp_value >= kTimerCounterPeriod)
            duty_cmp_value = kTimerCounterPeriod;
        else if (duty_cmp_value <= cmpval::kMinSwitchDuty)
            duty_cmp_value = cmpval::kMinSwitchDuty;

        LL_HRTIM_TIM_SetCompare3(hrtim, timer, duty_cmp_value);

        LL_HRTIM_OUT_SetOutputSetSrc(hrtim, out_switch, LL_HRTIM_OUTPUTSET_MASTERCMP3);
        LL_HRTIM_OUT_SetOutputResetSrc(hrtim, out_switch, LL_HRTIM_OUTPUTRESET_TIMCMP3);

        // deadtime inserted: in auto-delay mode, this output will be set when the counter
        // reaches `CMP3 + CMP4` if capture is not triggered.
        LL_HRTIM_OUT_SetOutputSetSrc(hrtim, out_syncre, LL_HRTIM_OUTPUTSET_TIMCMP4);
        LL_HRTIM_OUT_SetOutputResetSrc(hrtim, out_syncre, LL_HRTIM_OUTPUTRESET_TIMPER);

        // Disable capture trigger, so that capture timeout event will be triggered and
        // `out_syncre` will be set when counter reaches `CMP3 + CMP4` by auto-delay mechanism.
        LL_HRTIM_TIM_SetCaptureTrig(hrtim, timer, LL_HRTIM_CAPTUREUNIT_2, LL_HRTIM_CAPTURETRIG_NONE);
    }

    GARNET_CCMRAM_TEXT void hrtim_config_highz_leg(HRTIM_TypeDef *hrtim, uint32_t timer,
                                                   uint32_t out_high, uint32_t out_low)
    {
        LL_HRTIM_TIM_SetCompare3(hrtim, timer, cmpval::kDeadtime1);

        LL_HRTIM_OUT_SetOutputSetSrc(hrtim, out_high, 0);
        LL_HRTIM_OUT_SetOutputResetSrc(hrtim, out_high, LL_HRTIM_OUTPUTRESET_TIMCMP3);

        LL_HRTIM_OUT_SetOutputSetSrc(hrtim, out_low, 0);
        LL_HRTIM_OUT_SetOutputResetSrc(hrtim, out_low, LL_HRTIM_OUTPUTRESET_TIMPER);

        LL_HRTIM_TIM_SetCaptureTrig(hrtim, timer, LL_HRTIM_CAPTUREUNIT_2, LL_HRTIM_CAPTURETRIG_NONE);
    }

    // High-side constant ON is achievable since the existence of isolated auxiliary
    // power supply.
    GARNET_CCMRAM_TEXT void hrtim_config_pullup_leg(HRTIM_TypeDef *hrtim, uint32_t timer,
                                                    uint32_t out_high, uint32_t out_low)
    {
        LL_HRTIM_TIM_SetCompare3(hrtim, timer, cmpval::kMaxDuty);

        LL_HRTIM_OUT_SetOutputSetSrc(hrtim, out_high, LL_HRTIM_OUTPUTSET_MASTERCMP3);
        LL_HRTIM_OUT_SetOutputResetSrc(hrtim, out_high, 0);
        LL_HRTIM_OUT_SetOutputSetSrc(hrtim, out_low, 0);
        LL_HRTIM_OUT_SetOutputResetSrc(hrtim, out_low, LL_HRTIM_OUTPUTRESET_TIMPER);

        // peak-current event should not trigger the capture unit,
        // since we're working in pass-through mode.
        LL_HRTIM_TIM_SetCaptureTrig(hrtim, timer, LL_HRTIM_CAPTUREUNIT_2, LL_HRTIM_CAPTURETRIG_NONE);
    }

    GARNET_CCMRAM_TEXT void hrtim_config_pulldown_leg(HRTIM_TypeDef *hrtim, uint32_t timer,
                                                      uint32_t out_high, uint32_t out_low)
    {
        LL_HRTIM_TIM_SetCompare3(hrtim, timer, cmpval::kDeadtime1);

        LL_HRTIM_OUT_SetOutputSetSrc(hrtim, out_high, 0);
        LL_HRTIM_OUT_SetOutputResetSrc(hrtim, out_high, LL_HRTIM_OUTPUTRESET_TIMCMP3);

        LL_HRTIM_OUT_SetOutputSetSrc(hrtim, out_low, LL_HRTIM_OUTPUTSET_TIMCMP4);
        LL_HRTIM_OUT_SetOutputResetSrc(hrtim, out_low, 0);

        LL_HRTIM_TIM_SetCaptureTrig(hrtim, timer, LL_HRTIM_CAPTUREUNIT_2, LL_HRTIM_CAPTURETRIG_NONE);
    }


    INISR void topo_conf_adaptive_fsbb(Ctrloop::FastRamContext *ctx, float vin, float vout)
    {
        float m = 2.0f;
        if (vout >= 1.0f)
        {
            // vin/vout decides the converter's operation region
            // (buck, buck with boost aux., boost with buck aux., boost)
            m = vin / vout;
        }

        OpRegion next_op_region = ctx->op_region;
        switch (ctx->op_region)
        {
            case OpRegion::kIdle:
                if (m < 0.85f)
                    next_op_region = OpRegion::kBoost;
                else if (m >= 0.75f && m < 1.05f)
                    next_op_region = OpRegion::kBoostWithBuckAux;
                else if (m >= 0.95f && m < 1.33f)
                    next_op_region = OpRegion::kBuckWithBoostAux;
                else if (m > 1.18f)
                    next_op_region = OpRegion::kBuck;

                break;

            case OpRegion::kBoost:
                if (m > 0.85f)
                    next_op_region = OpRegion::kBoostWithBuckAux;
                break;

            case OpRegion::kBoostWithBuckAux:
                if (m < 0.75f)
                    next_op_region = OpRegion::kBoost;
                else if (m > 1.05f)
                    next_op_region = OpRegion::kBuckWithBoostAux;
                break;

            case OpRegion::kBuckWithBoostAux:
                if (m < 0.95f)
                    next_op_region = OpRegion::kBoostWithBuckAux;
                else if (m > 1.33f)
                    next_op_region = OpRegion::kBuck;
                break;

            case OpRegion::kBuck:
                if (m < 1.18f)
                    next_op_region = OpRegion::kBuckWithBoostAux;
                break;
        }

        if (ctx->op_region != next_op_region)
        {
            // Switch topology
            ctx->op_region = next_op_region;
            switch (next_op_region)
            {
                case OpRegion::kBoost:
                    hrtim_config_pullup_leg(ctx->ll.hrtim, TIMER_L, OUT_L_HG, OUT_L_LG);
                    hrtim_config_peak_curr_mode_leg(ctx->ll.hrtim, TIMER_R, OUT_R_LG, OUT_R_HG);
                    break;

                case OpRegion::kBoostWithBuckAux:
                    hrtim_config_fixed_duty_leg(ctx->ll.hrtim, TIMER_L, OUT_L_HG, OUT_L_LG,
                                                static_cast<uint32_t>(0.85f * kTimerCounterPeriod));
                    hrtim_config_peak_curr_mode_leg(ctx->ll.hrtim, TIMER_R, OUT_R_LG, OUT_R_HG);
                    break;

                case OpRegion::kBuckWithBoostAux:
                    hrtim_config_fixed_duty_leg(ctx->ll.hrtim, TIMER_R, OUT_R_LG, OUT_R_HG,
                                                static_cast<uint32_t>(0.15f * kTimerCounterPeriod));
                    hrtim_config_peak_curr_mode_leg(ctx->ll.hrtim, TIMER_L, OUT_L_HG, OUT_L_LG);
                    break;

                case OpRegion::kBuck:
                    hrtim_config_pullup_leg(ctx->ll.hrtim, TIMER_R, OUT_R_HG, OUT_R_LG);
                    hrtim_config_peak_curr_mode_leg(ctx->ll.hrtim, TIMER_L, OUT_L_HG, OUT_L_LG);
                    break;

                case OpRegion::kIdle:
                    // unreachable code
                    break;
            }
        }
    }

    INISR void topo_conf_inductor_denergize(Ctrloop::FastRamContext *ctx)
    {
        // Inductor de-energize mode
        if (ctx->topo_nb_denergize_cycles == 0)
        {
            // First de-energize cycle, switch our topology first.
            hrtim_config_pulldown_leg(ctx->ll.hrtim, TIMER_L, OUT_L_HG, OUT_L_LG);
            hrtim_config_pulldown_leg(ctx->ll.hrtim, TIMER_R, OUT_R_HG, OUT_R_LG);
        }

        // Wait for 100 sampling cycles (4us * 100 = 400us)
        if (ctx->topo_nb_denergize_cycles < 100)
            ctx->topo_nb_denergize_cycles++;

        if (ctx->topo_nb_denergize_cycles == 100)
        {
            hrtim_config_highz_leg(ctx->ll.hrtim, TIMER_L, OUT_L_HG, OUT_L_LG);
            hrtim_config_highz_leg(ctx->ll.hrtim, TIMER_R, OUT_R_HG, OUT_R_LG);

            // To prevent this branch being executed in the following cycles.
            // Since LL calls are expensive.
            ctx->topo_nb_denergize_cycles++;
        }
    }

    GARNET_CCMRAM_TEXT void topo_conf_reset(Ctrloop::FastRamContext *ctx)
    {
        ctx->topo_nb_denergize_cycles = 0;
    }

    // passthrough mode
    GARNET_CCMRAM_TEXT void pt_start(Ctrloop::FastRamContext *ctx, float vin, float vout)
    {
        if (vin >= 1.0f)
        {
            // pre-biased startup (output capacitor is partially charged
            // or an external voltage source is connected).
            const float estimated_duty = vout / vin;
            ctx->pt_softstart_duty_curr =
                static_cast<uint32_t>(estimated_duty * kTimerCounterPeriod);
        }
        else
        {
            ctx->pt_softstart_duty_curr = cmpval::kMinSwitchDuty;
        }

        if (ctx->pt_softstart_duty_curr < cmpval::kMinSwitchDuty)
            ctx->pt_softstart_duty_curr = cmpval::kMinSwitchDuty;
        else if (ctx->pt_softstart_duty_curr > cmpval::kMaxDuty)
            ctx->pt_softstart_duty_curr = cmpval::kMaxDuty;

        ctx->pt_state = Ctrloop::PtModeState::kSoftStart;

        hrtim_config_fixed_duty_leg(ctx->ll.hrtim, TIMER_L, OUT_L_HG, OUT_L_LG,
                                    ctx->pt_softstart_duty_curr);
        hrtim_config_pullup_leg(ctx->ll.hrtim, TIMER_R, OUT_R_HG, OUT_R_LG);
    }

    GARNET_CCMRAM_TEXT void pt_run(Ctrloop::FastRamContext *ctx)
    {
        if (ctx->pt_softstart_duty_curr > cmpval::kMaxDuty)
            return;

        LL_HRTIM_TIM_SetCompare3(ctx->ll.hrtim, TIMER_L, ctx->pt_softstart_duty_curr);
        ctx->pt_softstart_duty_curr += 3;

        if (ctx->pt_softstart_duty_curr > cmpval::kMaxDuty)
            hrtim_config_pullup_leg(ctx->ll.hrtim, TIMER_L, OUT_L_HG, OUT_L_LG);
    }

} // namespace anonymous

void Ctrloop::update_measures(FastRamContext *fctx,
                              float v1, float v2, float i1, float i2, float iref)
{
    if (++fctx->mea_update_counter > kMeasUpdatePrescaler)
    {
        // Writing volatile variables is quite expensive, so we update them
        // in a pre-scaled, relatively low frequency.
        fctx->mea_update_counter = 0;
        meas_.store({
            .ctrl_state = fctx->ctrl_state,
            .op_region = fctx->op_region,
            .outstate = fctx->outstate,
            .v1 = v1,
            .v2 = v2,
            .i1 = i1,
            .i2 = i2,
            .iref = iref
        });
    }
}

GARNET_CCMRAM_TEXT void Ctrloop::on_adc_injected_irq()
{
    LL_GPIO_SetOutputPin(LPDBG_PULSE_GPIO_Port, LPDBG_PULSE_Pin);
    ScopeExitGuard on_exit_gpio([] {
        LL_GPIO_ResetOutputPin(LPDBG_PULSE_GPIO_Port, LPDBG_PULSE_Pin);
    });

    FastRamContext *ctx = config_.fast_ram_ctx;
    ADC_TypeDef *ll_adc1 = ctx->ll.adc1, *ll_adc2 = ctx->ll.adc2;

    if (!LL_ADC_IsActiveFlag_JEOS(ll_adc1))
    {
        LL_GPIO_ResetOutputPin(LPDBG_PULSE_GPIO_Port, LPDBG_PULSE_Pin);
        return;
    }
    LL_ADC_ClearFlag_JEOS(ll_adc1);
    LL_ADC_ClearFlag_JEOS(ll_adc2);

#define RD(field, adc, adc_rank)    \
    uint32_t field##_raw = LL_ADC_INJ_ReadConversionData12(adc, adc_rank); \
    float field = ctx->cali.field.k * (float) field##_raw + ctx->cali.field.b;

    RD(i1, ll_adc1, LL_ADC_INJ_RANK_1)
    RD(i2, ll_adc1, LL_ADC_INJ_RANK_2)
    RD(v1, ll_adc2, LL_ADC_INJ_RANK_2)
    RD(v2, ll_adc2, LL_ADC_INJ_RANK_1)
#undef RD

    ControlState req_ctrl_state = requested_ctrl_state_.load(
        std::memory_order_relaxed);

    switch (ctx->ctrl_state)
    {
        case ControlState::kUnmanaged: [[unlikely]]
        {
            if (req_ctrl_state == ControlState::kOperational)
            {
                LL_HRTIM_EnableOutput(ctx->ll.hrtim, OUT_ALL);
                LL_HRTIM_FLT_Enable(ctx->ll.hrtim, config_.hrtim_ovp_fault_num);

                ctx->ctrl_state = ControlState::kOperational;
            }
            else
            {
                // unmanaged state: by-pass everything, only updating the
                // measurements (feedback).
                ctx->outstate = OutputState::kOff;
                update_measures(ctx, v1, v2, i1, i2, 0);
                return;
            }
            break;
        }

        case ControlState::kOperational:
        {
            if (req_ctrl_state == ControlState::kUnmanaged) [[unlikely]]
            {
                hrtim_config_highz_leg(ctx->ll.hrtim, TIMER_L, OUT_L_HG, OUT_L_LG);
                hrtim_config_highz_leg(ctx->ll.hrtim, TIMER_R, OUT_R_HG, OUT_R_LG);
                iir_2p2z_reset(&ctx->buck_cv_2p2z);
                iir_2p2z_reset(&ctx->boost_cv_2p2z);
                topo_conf_reset(ctx);

                ctx->ctrl_state = ControlState::kUnmanaged;
                return;
            }
            break;
        }
    }

    const TargetSet requested_target = target_reader_.load();
    switch (ctx->outstate)
    {
        case OutputState::kOff:
        {
            if (requested_target.outstate == OutputState::kRegulated)
            {
                iir_2p2z_reset(&ctx->buck_cv_2p2z);
                iir_2p2z_reset(&ctx->boost_cv_2p2z);
                topo_conf_reset(ctx);

                // Biased startup: ramping from the current output voltage.
                // This usually happens when the output capacitor is partially charged,
                // or an external voltage source is connected.
                ctx->cv_setpoint_ramping = v2;

                // Just for safety. Correct peak-current sawtooth will be set immediately
                // by the CV loop's code.
                update_peak_current_sawtooth(ctx, 0, 0);

                ctx->outstate = OutputState::kRegulated;
            }
            else if (requested_target.outstate == OutputState::kPassthrough)
            {
                topo_conf_reset(ctx);
                pt_start(ctx, v1, v2);
                ctx->outstate = OutputState::kPassthrough;
            }

            break;
        }

        case OutputState::kRegulated:
        case OutputState::kPassthrough:
        {
            // Regulated and Passthrough states cannot transition directly between
            // each other without transitioning to Off state first.

            if (requested_target.outstate == OutputState::kOff)
                ctx->outstate = OutputState::kOff;
            break;
        }
    }

    float iref = 0;
    if (ctx->outstate == OutputState::kRegulated)
    {
        topo_conf_adaptive_fsbb(ctx, v1, v2);
        // TODO(masshiroio): implement pulse-skipping/burst mode, and constant-current loop

        // Soft start & soft setpoint transient (linear ramping)
        const float cv_setpoint = requested_target.vout;
        if (cv_setpoint != ctx->cv_setpoint_ramping) [[unlikely]]
        {
            if (cv_setpoint > ctx->cv_setpoint_ramping)
            {
                ctx->cv_setpoint_ramping = std::min(
                    ctx->cv_setpoint_ramping + kCvRampingStep, cv_setpoint);
            }
            else
            {
                ctx->cv_setpoint_ramping = std::max(
                    ctx->cv_setpoint_ramping - kCvRampingStep, cv_setpoint);
            }
        }

        const float err = ctx->cv_setpoint_ramping - v2;
        if (ctx->op_region == OpRegion::kBuck ||
            ctx->op_region == OpRegion::kBuckWithBoostAux)
        {
            iref = iir_2p2z_update(&ctx->buck_cv_2p2z, ctx->buck_cv_coeffs, err, -2.0f, 30.0f);
            update_peak_current_sawtooth(ctx, iref, v2);
        }
        else if (ctx->op_region == OpRegion::kBoost ||
                 ctx->op_region == OpRegion::kBoostWithBuckAux)
        {
            iref = iir_2p2z_update(&ctx->boost_cv_2p2z, ctx->boost_cv_coeffs, err, -2.0f, 30.0f);
            update_peak_current_sawtooth(ctx, iref, v2 - v1);
        }
    }
    else if (ctx->outstate == OutputState::kPassthrough)
    {
        ctx->op_region = OpRegion::kIdle;
        pt_run(ctx);
    }
    else if (ctx->outstate == OutputState::kOff)
    {
        ctx->op_region = OpRegion::kIdle;
        topo_conf_inductor_denergize(ctx);
    }

    update_measures(ctx, v1, v2, i1, i2, iref);
}

void Ctrloop::on_hrtim_fault_irq()
{
    // HRTIM fault interruption, PWM output has been disabled by
    // the fault circuitry before we reach here.
    FastRamContext *fctx = config_.fast_ram_ctx;
    HRTIM_TypeDef *hrtim = fctx->ll.hrtim;

    if (llhelper::hrtim_has_intr_fault_flag(hrtim, config_.hrtim_ovp_fault_num))
    {
        llhelper::hrtim_clear_intr_fault_flag(hrtim, config_.hrtim_ovp_fault_num);
        // Disable the fault line to prevent interrupt flooding.
        // We will lock at fault state in software.
        LL_HRTIM_FLT_Disable(hrtim, config_.hrtim_ovp_fault_num);

        // Although the PWM has been disabled, we do it again manually
        // for better safety.
        LL_HRTIM_DisableOutput(hrtim, OUT_ALL);

        requested_ctrl_state_ = ControlState::kUnmanaged;
    }
}

static void init_slope_comp_coeff(Ctrloop::FastRamContext *fastctx)
{
    // Slope of current: S[A/us] = Kcomp * dv_fall / L
    // => convert to S[LSB/us] = S[A/us] * k = Kcomp * dv_fall / L * k
    // => step value: S[LSB/step] = 0.046us/step * S[LSB/us]
    float slope_amps_per_usec = 1.0f / kInductorNominalUh;
    float slope_lsb_per_usec = slope_amps_per_usec * fastctx->cali.il_inv.k;

    // HRTIM triggers DAC step in every 46ns (0.046us), and LSB/us * us/step = LSB/step.
    float step_val_f = slope_lsb_per_usec * 0.046f;

    // `LL_DAC_SetWaveSawtoothStepData()` actually uses Q12.4 format
    // representation for `StepData` parameter. So we must x16 to get the
    // correct value to be written into register.
    fastctx->slope_dac_step_raw_per_volt = step_val_f * 16.0f;
}

ErrStatus Ctrloop::init(const Config& config, AtomicTargetReader&& target_reader)
{
    config_ = config;
    target_reader_ = std::move(target_reader);

    FastRamContext *fastctx = config_.fast_ram_ctx;

    fastctx->ll.adc1 = config.ll_adc1;
    fastctx->ll.adc2 = config.ll_adc2;
    fastctx->ll.hrtim = config.ll_hrtim;
    fastctx->ll.dac_pcmc = config.dac_pcmc->Instance;

    fastctx->ctrl_state = ControlState::kOperational;

    fastctx->cali = *config_.cali;

    fastctx->buck_cv_coeffs = kDefaultBuckCvCompCoeffs;
    fastctx->boost_cv_coeffs = kDefaultBoostCvCompCoeffs;

    topo_conf_reset(fastctx);
    fastctx->mea_update_counter = 0;

    // Init slope compensation
    init_slope_comp_coeff(fastctx);

    FUMI_LOGI(kTAG, "calibrate and init ADC units");

    ErrStatus err = llhelper::adc_start_calibration(config.ll_adc1, LL_ADC_SINGLE_ENDED);
    if (err != ErrStatus::kOk)
    {
        FUMI_LOGE(kTAG, "failed to calibrate ADC1: %s", get_error_str(err));
        return err;
    }

    err = llhelper::adc_start_calibration(config.ll_adc2, LL_ADC_SINGLE_ENDED);
    if (err != ErrStatus::kOk)
    {
        FUMI_LOGE(kTAG, "failed to calibrate ADC2: %s", get_error_str(err));
        return err;
    }

    llhelper::adc_enable(config.ll_adc1);
    llhelper::adc_enable(config.ll_adc2);

    // Zero current detection for DEM (diode emulation mode).
    // TODO(sora): not implemented yet. set DAC output 0 to disable this function.
    HAL_DAC_Start(config.dac_zcd, DAC_CHANNEL_1);
    HAL_DAC_SetValue(config.dac_zcd, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
    HAL_COMP_Start(config.comp_zcd);

    HRTIM_TypeDef *ll_hrtim = config_.ll_hrtim;

    // Master timer:
    //   CMP1: ADC trigger
    //   CMP3: high-side delay ON (to avoid timing race condition, also provides
    //                             deadtime between low-side OFF and high-side ON)
    LL_HRTIM_TIM_SetCompare1(ll_hrtim, LL_HRTIM_TIMER_MASTER, 2960);
    LL_HRTIM_TIM_SetCompare3(ll_hrtim, LL_HRTIM_TIMER_MASTER, cmpval::kDeadtime1);

    //   CMP1: blanking
    //   CMP2: sawtooth generation trigger
    //   CMP3: maximum duty limit + auto-delay timeout trigger
    //   CMP4: trigger TB2 (low-side), with auto-delay to generate dead-time,
    //         provides deadtime between high-side OFF and low-side ON

    // Timer B:
    LL_HRTIM_TIM_SetCompare1(ll_hrtim, TIMER_L, cmpval::kBlanking);
    LL_HRTIM_TIM_SetCompare2(ll_hrtim, TIMER_L, cmpval::kSawtooth);
    LL_HRTIM_TIM_SetCompare3(ll_hrtim, TIMER_L, cmpval::kMaxDuty);
    LL_HRTIM_TIM_SetCompare4(ll_hrtim, TIMER_L, cmpval::kDeadtime2);

    // Timer D:
    LL_HRTIM_TIM_SetCompare1(ll_hrtim, TIMER_R, cmpval::kBlanking);
    LL_HRTIM_TIM_SetCompare3(ll_hrtim, TIMER_R, cmpval::kMaxDuty);
    LL_HRTIM_TIM_SetCompare4(ll_hrtim, TIMER_R, cmpval::kDeadtime2);

    fastctx->op_region = OpRegion::kIdle;
    hrtim_config_highz_leg(ll_hrtim, TIMER_L, OUT_L_HG, OUT_L_LG);
    hrtim_config_highz_leg(ll_hrtim, TIMER_R, OUT_R_HG, OUT_R_LG);

    LL_HRTIM_ForceUpdate(ll_hrtim, TIMER_L | TIMER_R | LL_HRTIM_TIMER_MASTER);

    // PCMC comparator (EEV4)
    HAL_DAC_Start(config.dac_pcmc, DAC_CHANNEL_1);
    LL_DAC_SetWaveSawtoothResetData(config.dac_pcmc->Instance, LL_DAC_CHANNEL_1, 0);
    LL_DAC_SetWaveSawtoothStepData(config.dac_pcmc->Instance, LL_DAC_CHANNEL_1, 0);
    HAL_COMP_Start(config.comp_pcmc);

    fastctx->outstate = OutputState::kOff;

    // All the initializations above must happen before enabling the IRQ.
    std::atomic_signal_fence(std::memory_order_seq_cst);

    // ADC1 and ADC2 are triggered by HRTIM simultaneously, and has the same
    // sampling time (cycles) for each inject group rank, the same number of ranks.
    // Therefore, they should complete conversions simultaneously. We only need
    // to enable one ADC's JEOS irq.
    llhelper::adc_start_injected_irq(config.ll_adc1, 0, 1);
    llhelper::adc_start_injected_irq(config.ll_adc2, 0, 0);

    LL_HRTIM_TIM_CounterEnable(ll_hrtim, TIMER_L | TIMER_R | LL_HRTIM_TIMER_MASTER);
    LL_HRTIM_EnableOutput(ll_hrtim, OUT_L_HG | OUT_L_LG | OUT_R_HG | OUT_R_LG);

    // Enable protections
    LL_HRTIM_EnableIT_FLT4(ll_hrtim);

    return ErrStatus::kOk;
}

} // namespace ctrl

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
#include "control/fast_ll.h"
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

    constexpr uint32_t kNeverEq = 0xffdf;
    static_assert(kNeverEq > kTimerCounterPeriod);

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

constexpr float kPSkipIrefThreshold = 0.5f;
constexpr float kPSkipIrefHysteresis = 0.3f;

namespace
{
    // Suspend Timer B/D preload transfers while writing each complete topology
    // configuration, then resume updates at the next configured timer event.
    //
    // Since the shadow registers are configured to update at the start of a
    // PWM PERIOD, and our fsw/2 == fsample(fcontrol):
    //
    //    preload               preload             preload
    //    update                update              update
    //      V                    V                    V
    //      ------------         ------------         ------------
    //      |          |         |          |         |          |
    //  -----          -----------          -----------          -----
    //
    //    ADC trigger
    //      V
    //      ---------------------------------         ----------------
    //      |                               |         |
    //  -----                               -----------
    //                      |----------|
    //                 (writing shadow registers)
    //                           ^
    //                           ` race condition: partially written!
    //
    // So use this RAII guard to protect HRTIM register writes. A race condition here
    // merely defers the shadow register update to the next PWM rising edge.
    // This transient latency is acceptable since HRTIM writes only happen during
    // topology/timing changes, while fast control loop outputs target the DAC directly.
    class UpdateProhibitedScope
    {
    public:
        GARNET_NON_COPYABLE(UpdateProhibitedScope)
        GARNET_NON_MOVABLE(UpdateProhibitedScope)

        constexpr static uint32_t kUpdateMask = HRTIM_CR1_TBUDIS | HRTIM_CR1_TDUDIS;

        INISR UpdateProhibitedScope(HRTIM_TypeDef *hrtim) : hrtim_(hrtim) {
            SET_BIT(hrtim->sCommonRegs.CR1, kUpdateMask);
        }

        INISR ~UpdateProhibitedScope() {
            CLEAR_BIT(hrtim_->sCommonRegs.CR1, kUpdateMask);
        }

    private:
        HRTIM_TypeDef *hrtim_;
    };

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

    struct DemMode {};
    struct FccmMode {};

    // Keep the leg identifiers constant through to fastll register selection.
    template<uint32_t kTimer, uint32_t kOutSwitch, uint32_t kOutSyncre, typename Mode>
    GARNET_CCMRAM_TEXT void hrtim_config_peak_curr_mode_leg(HRTIM_TypeDef *hrtim)
    {
        fastll::hrtim_set_compare3<kTimer>(hrtim, cmpval::kMaxDuty);

        fastll::hrtim_set_output_set_source<kOutSwitch>(hrtim, LL_HRTIM_OUTPUTSET_MASTERCMP3);
        fastll::hrtim_set_output_reset_source<kOutSwitch>(
            hrtim, LL_HRTIM_OUTPUTRESET_TIMCMP3 | LL_HRTIM_OUTPUTRESET_EEV_4);
        fastll::hrtim_set_output_set_source<kOutSyncre>(hrtim, LL_HRTIM_OUTPUTSET_TIMCMP4);

        if constexpr (std::is_same_v<Mode, DemMode>)
        {
            fastll::hrtim_set_output_reset_source<kOutSyncre>(
                hrtim, LL_HRTIM_OUTPUTRESET_TIMPER | LL_HRTIM_OUTPUTRESET_EEV_2);
        }
        else
        {
            fastll::hrtim_set_output_reset_source<kOutSyncre>(
                hrtim, LL_HRTIM_OUTPUTRESET_TIMPER);
        }

        // enable peak-current event
        fastll::hrtim_set_capture_trigger<
            kTimer, LL_HRTIM_CAPTUREUNIT_2, LL_HRTIM_CAPTURETRIG_EEV_4>(hrtim);
    }

    template<uint32_t kTimer, uint32_t kOutSwitch, uint32_t kOutSyncre, typename Mode>
    GARNET_CCMRAM_TEXT void hrtim_config_fixed_duty_leg(HRTIM_TypeDef *hrtim, uint32_t duty_cmp_value)
    {
        if (duty_cmp_value >= kTimerCounterPeriod)
            duty_cmp_value = kTimerCounterPeriod;
        else if (duty_cmp_value <= cmpval::kMinSwitchDuty)
            duty_cmp_value = cmpval::kMinSwitchDuty;

        fastll::hrtim_set_compare3<kTimer>(hrtim, duty_cmp_value);

        fastll::hrtim_set_output_set_source<kOutSwitch>(hrtim, LL_HRTIM_OUTPUTSET_MASTERCMP3);
        fastll::hrtim_set_output_reset_source<kOutSwitch>(hrtim, LL_HRTIM_OUTPUTRESET_TIMCMP3);

        // deadtime inserted: in auto-delay mode, this output will be set when the counter
        // reaches `CMP3 + CMP4` if capture is not triggered.
        fastll::hrtim_set_output_set_source<kOutSyncre>(hrtim, LL_HRTIM_OUTPUTSET_TIMCMP4);
        if constexpr (std::is_same_v<Mode, DemMode>)
        {
            fastll::hrtim_set_output_reset_source<kOutSyncre>(
                hrtim, LL_HRTIM_OUTPUTRESET_TIMPER | LL_HRTIM_OUTPUTRESET_EEV_2);
        }
        else
        {
            fastll::hrtim_set_output_reset_source<kOutSyncre>(
                hrtim, LL_HRTIM_OUTPUTRESET_TIMPER);
        }

        // Disable capture trigger, so that capture timeout event will be triggered and
        // `kOutSyncre` will be set when counter reaches `CMP3 + CMP4` by auto-delay mechanism.
        fastll::hrtim_set_capture_trigger<
            kTimer, LL_HRTIM_CAPTUREUNIT_2, LL_HRTIM_CAPTURETRIG_NONE>(hrtim);
    }

    template<uint32_t kTimer, uint32_t kOutHigh, uint32_t kOutLow>
    GARNET_CCMRAM_TEXT void hrtim_config_highz_leg(HRTIM_TypeDef *hrtim)
    {
        fastll::hrtim_set_compare3<kTimer>(hrtim, cmpval::kDeadtime1);

        fastll::hrtim_set_output_set_source<kOutHigh>(hrtim, 0);
        fastll::hrtim_set_output_reset_source<kOutHigh>(hrtim, LL_HRTIM_OUTPUTRESET_TIMCMP3);

        fastll::hrtim_set_output_set_source<kOutLow>(hrtim, 0);
        fastll::hrtim_set_output_reset_source<kOutLow>(hrtim, LL_HRTIM_OUTPUTRESET_TIMPER);

        fastll::hrtim_set_capture_trigger<
            kTimer, LL_HRTIM_CAPTUREUNIT_2, LL_HRTIM_CAPTURETRIG_NONE>(hrtim);
    }

    // High-side constant ON is achievable since the existence of isolated auxiliary
    // power supply.
    template<uint32_t kTimer, uint32_t kOutHigh, uint32_t kOutLow>
    GARNET_CCMRAM_TEXT void hrtim_config_pullup_leg(HRTIM_TypeDef *hrtim)
    {
        fastll::hrtim_set_compare3<kTimer>(hrtim, cmpval::kMaxDuty);

        fastll::hrtim_set_output_set_source<kOutHigh>(hrtim, LL_HRTIM_OUTPUTSET_MASTERCMP3);
        fastll::hrtim_set_output_reset_source<kOutHigh>(hrtim, 0);
        fastll::hrtim_set_output_set_source<kOutLow>(hrtim, 0);
        fastll::hrtim_set_output_reset_source<kOutLow>(hrtim, LL_HRTIM_OUTPUTRESET_TIMPER);

        // peak-current event should not trigger the capture unit,
        // since we're working in pass-through mode.
        fastll::hrtim_set_capture_trigger<
            kTimer, LL_HRTIM_CAPTUREUNIT_2, LL_HRTIM_CAPTURETRIG_NONE>(hrtim);
    }

    template<uint32_t kTimer, uint32_t kOutHigh, uint32_t kOutLow>
    GARNET_CCMRAM_TEXT void hrtim_config_pulldown_leg(HRTIM_TypeDef *hrtim)
    {
        fastll::hrtim_set_compare3<kTimer>(hrtim, cmpval::kDeadtime1);

        fastll::hrtim_set_output_set_source<kOutHigh>(hrtim, 0);
        fastll::hrtim_set_output_reset_source<kOutHigh>(hrtim, LL_HRTIM_OUTPUTRESET_TIMCMP3);

        fastll::hrtim_set_output_set_source<kOutLow>(hrtim, LL_HRTIM_OUTPUTSET_TIMCMP4);
        fastll::hrtim_set_output_reset_source<kOutLow>(hrtim, 0);

        fastll::hrtim_set_capture_trigger<
            kTimer, LL_HRTIM_CAPTUREUNIT_2, LL_HRTIM_CAPTURETRIG_NONE>(hrtim);
    }

    INISR void topo_conf_switch_to_region(Ctrloop::FastRamContext *ctx)
    {
        switch (ctx->op_region)
        {
            case OpRegion::kBoost:
                hrtim_config_pullup_leg<TIMER_L, OUT_L_HG, OUT_L_LG>(ctx->ll.hrtim);
                hrtim_config_peak_curr_mode_leg<TIMER_R, OUT_R_LG, OUT_R_HG, DemMode>(ctx->ll.hrtim);
                break;

            case OpRegion::kBoostWithBuckAux:
                hrtim_config_fixed_duty_leg<TIMER_L, OUT_L_HG, OUT_L_LG, DemMode>(
                    ctx->ll.hrtim, static_cast<uint32_t>(0.85f * kTimerCounterPeriod));
                hrtim_config_peak_curr_mode_leg<TIMER_R, OUT_R_LG, OUT_R_HG, DemMode>(ctx->ll.hrtim);
                break;

            case OpRegion::kBuckWithBoostAux:
                hrtim_config_fixed_duty_leg<TIMER_R, OUT_R_LG, OUT_R_HG, DemMode>(
                    ctx->ll.hrtim, static_cast<uint32_t>(0.15f * kTimerCounterPeriod));
                hrtim_config_peak_curr_mode_leg<TIMER_L, OUT_L_HG, OUT_L_LG, DemMode>(ctx->ll.hrtim);
                break;

            case OpRegion::kBuck:
                hrtim_config_pullup_leg<TIMER_R, OUT_R_HG, OUT_R_LG>(ctx->ll.hrtim);
                hrtim_config_peak_curr_mode_leg<TIMER_L, OUT_L_HG, OUT_L_LG, DemMode>(ctx->ll.hrtim);
                break;

            case OpRegion::kIdle:
                // unreachable code
                break;
        }
    }

    INISR void topo_conf_switch_to_pskip(Ctrloop::FastRamContext *ctx)
    {
        switch (ctx->op_region)
        {
            case OpRegion::kBoost:
                hrtim_config_pullup_leg<TIMER_L, OUT_L_HG, OUT_L_LG>(ctx->ll.hrtim);
                hrtim_config_highz_leg<TIMER_R, OUT_R_HG, OUT_R_LG>(ctx->ll.hrtim);
                break;

            case OpRegion::kBuck:
                hrtim_config_highz_leg<TIMER_L, OUT_L_HG, OUT_L_LG>(ctx->ll.hrtim);
                hrtim_config_pullup_leg<TIMER_R, OUT_R_HG, OUT_R_LG>(ctx->ll.hrtim);
                break;

            case OpRegion::kBoostWithBuckAux:
            case OpRegion::kBuckWithBoostAux:
                hrtim_config_highz_leg<TIMER_L, OUT_L_HG, OUT_L_LG>(ctx->ll.hrtim);
                hrtim_config_highz_leg<TIMER_R, OUT_R_HG, OUT_R_LG>(ctx->ll.hrtim);
                break;

            default:
                break;
        }
    }

    INISR OpRegion topo_select_op_region(OpRegion old_region, float vin, float vout)
    {
        float m = 2.0f;
        if (vout >= 1.0f)
        {
            // vin/vout decides the converter's operation region
            // (buck, buck with boost aux., boost with buck aux., boost)
            m = vin / vout;
        }

        OpRegion next_op_region = old_region;
        switch (old_region)
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

        return next_op_region;
    }

    INISR void topo_conf_inductor_denergize(Ctrloop::FastRamContext *ctx)
    {
        // Inductor de-energize mode
        if (ctx->topo_nb_denergize_cycles == 0)
        {
            UpdateProhibitedScope scope(ctx->ll.hrtim);
            // First de-energize cycle, switch our topology first.
            hrtim_config_pulldown_leg<TIMER_L, OUT_L_HG, OUT_L_LG>(ctx->ll.hrtim);
            hrtim_config_pulldown_leg<TIMER_R, OUT_R_HG, OUT_R_LG>(ctx->ll.hrtim);
        }

        // Wait for 100 sampling cycles (4us * 100 = 400us)
        if (ctx->topo_nb_denergize_cycles < 100)
            ctx->topo_nb_denergize_cycles++;

        if (ctx->topo_nb_denergize_cycles == 100)
        {
            UpdateProhibitedScope scope(ctx->ll.hrtim);
            hrtim_config_highz_leg<TIMER_L, OUT_L_HG, OUT_L_LG>(ctx->ll.hrtim);
            hrtim_config_highz_leg<TIMER_R, OUT_R_HG, OUT_R_LG>(ctx->ll.hrtim);

            // To prevent this branch being executed in the following cycles.
            // Since LL calls are expensive.
            ctx->topo_nb_denergize_cycles++;
        }
    }

    GARNET_CCMRAM_TEXT void topo_conf_reset(Ctrloop::FastRamContext *ctx)
    {
        ctx->topo_nb_denergize_cycles = 0;
        ctx->pskip = false;
        ctx->op_region = OpRegion::kIdle;
    }

    INISR float regulated_run(Ctrloop::FastRamContext *ctx, float v1, float v2,
                              float i2, float requested_vout)
    {
        // TODO(masshiroio): implement constant-current loop
        const OpRegion old_region = ctx->op_region;
        const bool old_pskip = ctx->pskip;

        ctx->op_region = topo_select_op_region(old_region, v1, v2);

        // Soft start & soft setpoint transient (linear ramping)
        if (requested_vout != ctx->cv_setpoint_ramping) [[unlikely]]
        {
            if (requested_vout > ctx->cv_setpoint_ramping)
            {
                ctx->cv_setpoint_ramping = std::min(
                    ctx->cv_setpoint_ramping + kCvRampingStep, requested_vout);
            }
            else
            {
                ctx->cv_setpoint_ramping = std::max(
                    ctx->cv_setpoint_ramping - kCvRampingStep, requested_vout);
            }
        }

        float iref = 0;
        const float err = ctx->cv_setpoint_ramping - v2;

        switch (ctx->op_region)
        {
            case OpRegion::kBuckWithBoostAux:
            case OpRegion::kBuck:
                iref = iir_2p2z_update(&ctx->buck_cv_2p2z, ctx->buck_cv_coeffs,
                    err, kPSkipIrefThreshold, 30.0f);
                update_peak_current_sawtooth(ctx, iref, v2);
                break;

            case OpRegion::kBoostWithBuckAux:
            case OpRegion::kBoost:
                iref = iir_2p2z_update(&ctx->boost_cv_2p2z, ctx->boost_cv_coeffs,
                    err, kPSkipIrefThreshold, 30.0f);
                update_peak_current_sawtooth(ctx, iref, v2 - v1);
                break;

            default:
                break;
        }

        // PSkip mode hysteresis
        if (iref <= kPSkipIrefThreshold)
            ctx->pskip = true;
        else if (iref > kPSkipIrefThreshold + kPSkipIrefHysteresis)
            ctx->pskip = false;

        if (ctx->op_region != old_region || old_pskip != ctx->pskip)
        {
            UpdateProhibitedScope scope(ctx->ll.hrtim);
            if (ctx->pskip)
                topo_conf_switch_to_pskip(ctx);
            else
                topo_conf_switch_to_region(ctx);
        }

        return iref;
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

        UpdateProhibitedScope scope(ctx->ll.hrtim);

        hrtim_config_fixed_duty_leg<TIMER_L, OUT_L_HG, OUT_L_LG, FccmMode>(
            ctx->ll.hrtim, ctx->pt_softstart_duty_curr);
        hrtim_config_pullup_leg<TIMER_R, OUT_R_HG, OUT_R_LG>(ctx->ll.hrtim);
    }

    GARNET_CCMRAM_TEXT void pt_run(Ctrloop::FastRamContext *ctx)
    {
        if (ctx->pt_softstart_duty_curr > cmpval::kMaxDuty)
            return;

        UpdateProhibitedScope scope(ctx->ll.hrtim);
        fastll::hrtim_set_compare3<TIMER_L>(ctx->ll.hrtim, ctx->pt_softstart_duty_curr);
        ctx->pt_softstart_duty_curr += 3;

        if (ctx->pt_softstart_duty_curr > cmpval::kMaxDuty)
            hrtim_config_pullup_leg<TIMER_L, OUT_L_HG, OUT_L_LG>(ctx->ll.hrtim);
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
                UpdateProhibitedScope scope(ctx->ll.hrtim);
                hrtim_config_highz_leg<TIMER_L, OUT_L_HG, OUT_L_LG>(ctx->ll.hrtim);
                hrtim_config_highz_leg<TIMER_R, OUT_R_HG, OUT_R_LG>(ctx->ll.hrtim);
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
        iref = regulated_run(ctx, v1, v2, i2, requested_target.vout);
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
    fastctx->pskip = false;

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
    const auto zcd_ref_raw = static_cast<int32_t>(config.cali->il_inv.map(0.1f));
    if (zcd_ref_raw <= 0 || zcd_ref_raw >= 4096)
    {
        FUMI_LOGE(kTAG, "DAC raw value of ZCD threshold reference is out of range");
        return ErrStatus::kInvalidArg;
    }

    HAL_DAC_Start(config.dac_zcd, DAC_CHANNEL_1);
    HAL_DAC_SetValue(config.dac_zcd, DAC_CHANNEL_1, DAC_ALIGN_12B_R, zcd_ref_raw);
    HAL_COMP_Start(config.comp_zcd);

    HRTIM_TypeDef *ll_hrtim = config_.ll_hrtim;

    // Master timer:
    //   CMP1: ADC trigger
    //   CMP3: high-side delay ON (to avoid timing race condition, also provides
    //                             deadtime between low-side OFF and high-side ON)
    fastll::hrtim_set_compare1<LL_HRTIM_TIMER_MASTER>(ll_hrtim, 2960);
    fastll::hrtim_set_compare3<LL_HRTIM_TIMER_MASTER>(ll_hrtim, cmpval::kDeadtime1);
    fastll::hrtim_set_compare4<LL_HRTIM_TIMER_MASTER>(ll_hrtim, cmpval::kNeverEq);

    //   CMP1: blanking
    //   CMP2: sawtooth generation trigger
    //   CMP3: maximum duty limit + auto-delay timeout trigger
    //   CMP4: trigger TB2 (low-side), with auto-delay to generate dead-time,
    //         provides deadtime between high-side OFF and low-side ON

    // Timer B:
    fastll::hrtim_set_compare1<TIMER_L>(ll_hrtim, cmpval::kBlanking);
    fastll::hrtim_set_compare2<TIMER_L>(ll_hrtim, cmpval::kSawtooth);
    fastll::hrtim_set_compare3<TIMER_L>(ll_hrtim, cmpval::kMaxDuty);
    fastll::hrtim_set_compare4<TIMER_L>(ll_hrtim, cmpval::kDeadtime2);

    // Timer D:
    fastll::hrtim_set_compare1<TIMER_R>(ll_hrtim, cmpval::kBlanking);
    fastll::hrtim_set_compare3<TIMER_R>(ll_hrtim, cmpval::kMaxDuty);
    fastll::hrtim_set_compare4<TIMER_R>(ll_hrtim, cmpval::kDeadtime2);

    fastctx->op_region = OpRegion::kIdle;
    hrtim_config_highz_leg<TIMER_L, OUT_L_HG, OUT_L_LG>(ll_hrtim);
    hrtim_config_highz_leg<TIMER_R, OUT_R_HG, OUT_R_LG>(ll_hrtim);

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

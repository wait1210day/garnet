// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_CTRLOOP_H
#define GARNET_CTRLOOP_H

#include <cstddef>
#include <cstdint>

#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_adc.h"

#include "base/defs.h"
#include "base/err.h"
#include "base/cali.h"
#include "base/atomic_snapshot.h"
#include "base/priorities.h"
#include "control/ctrloop_pub.h"
#include "control/laplace.h"

namespace ctrl
{

struct Iir2p2zComp
{
    float e_n1;
    float e_n2;
    float y_n1;
    float y_n2;
};

class Ctrloop
{
public:
    using AtomicTargetReader = AtomicSnapshot<TargetSet>::Reader<Priority::kCtrloopIrq>;

    enum class PtModeState
    {
        kSoftStart,
        kSteadyState
    };

    // This structure is supposed to be allocated in the fast CCM RAM,
    // e.g. statically placed in CCM RAM section defined by linker script.
    struct FastRamContext
    {
        struct Peripherals
        {
            ADC_TypeDef *adc1;
            ADC_TypeDef *adc2;
            HRTIM_TypeDef *hrtim;
            DAC_TypeDef *dac_pcmc;
        };

        Peripherals ll;

        ControlState ctrl_state;

        AdcCaliCoeffs cali;
        float cv_setpoint_ramping;

        laplace::DiscreteComp<float> buck_cv_coeffs;
        Iir2p2zComp buck_cv_2p2z;

        laplace::DiscreteComp<float> boost_cv_coeffs;
        Iir2p2zComp boost_cv_2p2z;

        // Slope compensation
        float slope_dac_step_raw_per_volt;
        float slope_dv_fall_filtered;

        OpRegion op_region;
        OutputState outstate;
        bool pskip;

        uint32_t topo_nb_denergize_cycles;

        // passthrough mode softstart cycles counter
        uint32_t pt_softstart_duty_curr;
        PtModeState pt_state;

        uint32_t mea_update_counter;
    };

    struct Config
    {
        ADC_TypeDef *ll_adc1;
        ADC_TypeDef *ll_adc2;
        HRTIM_TypeDef *ll_hrtim;
        uint32_t hrtim_ovp_fault_num;
        DAC_HandleTypeDef *dac_pcmc;
        DAC_HandleTypeDef *dac_zcd;
        COMP_HandleTypeDef *comp_pcmc;
        COMP_HandleTypeDef *comp_zcd;
        const AdcCaliCoeffs *cali;

        FastRamContext *fast_ram_ctx;
    };

    [[nodiscard]] ErrStatus init(const Config& config, AtomicTargetReader&& target_reader);

    GARNET_CCMRAM_TEXT void on_adc_injected_irq();
    void on_hrtim_fault_irq();

#define CTRLOOP_FAST_IRQ_HANDLER_IMPL(sym, instance, type) \
    extern "C" GARNET_CCMRAM_TEXT void sym() { (instance).on_##type##_irq(); }
#define CTRLOOP_IRQ_HANDLER_IMPL(sym, instance, type) \
    extern "C" void sym() { (instance).on_##type##_irq(); }

    // Multiple readers are allowed, and the caller may be blocked.
    [[nodiscard]] auto& get_measures_reader()
    {
        return meas_;
    }

    // Clear the unmanaged state and try to bring the control loop back to normal
    // operational state. The unmanaged state is caused by a HRTIM fault event.
    //
    // Once we get back to operational state, the fault lines will be enabled
    // immediately, and return to unmanaged state if an fault event is triggered
    // again (even if the output is not enabled).
    void clear_unmanaged_state()
    {
        requested_ctrl_state_.store(
            ControlState::kOperational, std::memory_order_relaxed);
    }

private:
    GARNET_CCMRAM_TEXT void update_measures(FastRamContext *fctx,
        float v1, float v2, float i1, float i2, float iref);

    Config config_{};
    AtomicTargetReader target_reader_;
    std::atomic<ControlState> requested_ctrl_state_ = ControlState::kOperational;
    SingleCoreSeqlock<Measures, Priority::kCtrloopIrq> meas_{{}};
};

} // namespace ctrl

#endif //GARNET_CTRLOOP_H

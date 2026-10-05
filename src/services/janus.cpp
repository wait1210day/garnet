// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include "stm32g4xx_ll_gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#include "base/panic.h"
#include "base/coroutine.h"
#include "drivers/i2c_device.h"
#include "drivers/i2cdev/pca9536.h"
#include "drivers/i2cdev/mcp4728.h"
#include "services/fumi.h"
#include "services/hermes_messages.h"
#include "services/janus.h"

#include <filesystem>

namespace
{
constexpr const char *kTAG = "janus";

using PcaCrPortHighBiasEn1 = pca9536::CR::P2;
using PcaCrPortHighBiasEn2 = pca9536::CR::P3;
using PcaOdrPortHighBiasEn1 = pca9536::ODR::P2;
using PcaOdrPortHighBiasEn2 = pca9536::ODR::P3;

constexpr TickType_t kGdrvPwrWaitTicks = pdMS_TO_TICKS(30);
constexpr TickType_t kHibernateTicks = pdMS_TO_TICKS(10000);

// 40V is the physical design limit for both input and output due to the fully
// symmetric topology:
//  40V * (7.5 / 100) = 3.0V
// where 7.5/100 is the Vin/Vout differential sensing circuit gain.
// Thus, the threshold seen by the OVP comparator is 3.0V (target DAC output = 3.0V).
//
// Given the internal 2.048V reference with 2x gain (FSR = 4.096V, 1 mV/LSB),
// the raw DAC value is directly set to 3000.
constexpr uint16_t kHwOvpMaxThreshold = 3000;

constexpr mcp4728::ChConf kMcp4728ChannelConf[4]{
    {
        // channel A, input side OVP
        .channel = mcp4728::Channel::kA,
        .update_now = true,
        .vref = mcp4728::Vref::kInt,
        .pd = mcp4728::PowerDown::kNormal,
        .gain = mcp4728::Gain::kX2,
        .data = kHwOvpMaxThreshold
    },
    {
        // channel B, output side OVP
        .channel = mcp4728::Channel::kB,
        .update_now = true,
        .vref = mcp4728::Vref::kInt,
        .pd = mcp4728::PowerDown::kNormal,
        .gain = mcp4728::Gain::kX2,
        .data = kHwOvpMaxThreshold
    },
    {
        // channel C, inductor OCP
        // TODO(masshiroio): not implemented yet
        .channel = mcp4728::Channel::kC,
        .update_now = true,
        .vref = mcp4728::Vref::kInt,
        .pd = mcp4728::PowerDown::kNormal,
        .gain = mcp4728::Gain::kX2,
        .data = 0
    },
    {
        // channel D, not used
        .channel = mcp4728::Channel::kD,
        .update_now = true,
        .vref = mcp4728::Vref::kInt,
        .pd = mcp4728::PowerDown::kNormal,
        .gain = mcp4728::Gain::kX2,
        .data = 0
    }
};


std::expected<mcp4728::DeviceStatus, i2c::Err>
    mcp4728_wait_por(i2c::Dev& dev)
{
    using namespace mcp4728;

    const TickType_t start_tick = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_tick < pdMS_TO_TICKS(100))
    {
        auto status = dev.receive<DeviceStatus>();
        if (!status)
        {
            FUMI_LOGE(kTAG, "mcp4728: failed to retrieve device status: %s",
                i2c::get_err_str(status.error()));
            return status;
        }
        // POR=1 means normal state.
        // Each channel has a POR state flag, but all of them are
        // the same copy of the device's global POR state flag.
        if (status->channels[0].por_flag)
        {
            // done
            return status;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return std::unexpected(i2c::Err::kTimeout);
}

std::expected<mcp4728::DeviceStatus, i2c::Err>
    mcp4728_wait_eeprom_ready(i2c::Dev& dev)
{
    using namespace mcp4728;

    const TickType_t start_tick = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_tick < pdMS_TO_TICKS(100))
    {
        auto status = dev.receive<DeviceStatus>();
        if (!status)
        {
            FUMI_LOGE(kTAG, "mcp4728: failed to retrieve device status: %s",
                i2c::get_err_str(status.error()));
            return status;
        }

        if (status->channels[0].is_ready)
        {
            // done
            return status;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return std::unexpected(i2c::Err::kTimeout);
}

bool mcp4728_verify_eeprom_config(const mcp4728::DeviceStatus& status)
{
    for (int i = 0; i < 4; i++)
    {
        const auto& eeprom_ch = status.channels[i];
        if (eeprom_ch.eeprom_vref != kMcp4728ChannelConf[i].vref ||
            eeprom_ch.eeprom_gain != kMcp4728ChannelConf[i].gain ||
            eeprom_ch.eeprom_pd != kMcp4728ChannelConf[i].pd ||
            eeprom_ch.eeprom_val != kMcp4728ChannelConf[i].data)
        {
            return false;
        }
    }
    return true;
}

ErrStatus init_mcp4728_dac(i2c::Bus *bus)
{
    using namespace mcp4728;
    if (auto scoped = bus->acquire(pdMS_TO_TICKS(100)))
    {
        FUMI_LOGI(kTAG, "mcp4728: probing device in i2c bus...");
        if (auto err = scoped->probe(kDefaultAddr, pdMS_TO_TICKS(5)); err != i2c::Err::kOk)
        {
            FUMI_LOGE(kTAG, "mcp4728: failed to probe device: %s", i2c::get_err_str(err));
            return ErrStatus::kFail;
        }
        FUMI_LOGI(kTAG, "mcp4728: device probed");

        i2c::Dev dev(*scoped, kDefaultAddr, XferMethod::kIRQ, pdMS_TO_TICKS(100));

        // query for device's current status
        auto status = mcp4728_wait_por(dev);
        if (!status)
            return ErrStatus::kFail;

        FUMI_LOGI(kTAG, "mcp4728: normal operational, POR=%d EEPROM_RDY=%d",
            static_cast<int>(status->channels[0].por_flag),
            static_cast<int>(status->channels[0].is_ready));

        if (!status->channels[0].is_ready)
        {
            FUMI_LOGE(kTAG, "mcp4728: device reports eeprom not ready, "
                "which is abnormal in initialization");
            return ErrStatus::kFail;
        }

        if (mcp4728_verify_eeprom_config(*status))
        {
            FUMI_LOGI(kTAG, "mcp4728: verified device eeprom");
            return ErrStatus::kOk;
        }

        FUMI_LOGI(kTAG, "mcp4728: writing parameters into device's eeprom");

        // this command will save parameters in EEPROM
        auto seq_wr_result = dev.execute<SequentialWrite>({
            .start_channel = Channel::kA,
            .update_now = true,
            .channels = {
                kMcp4728ChannelConf[0],
                kMcp4728ChannelConf[1],
                kMcp4728ChannelConf[2],
                kMcp4728ChannelConf[3]
            }
        });
        if (!seq_wr_result)
        {
            FUMI_LOGE(kTAG, "mcp4728: failed to write parameters: %s",
                i2c::get_err_str(seq_wr_result.error()));
        }

        status = mcp4728_wait_eeprom_ready(dev);
        if (!status)
            return ErrStatus::kFail;

        // verify parameters
        if (!mcp4728_verify_eeprom_config(*status))
        {
            FUMI_LOGE(kTAG, "mcp4728: eeprom verification failed");
            return ErrStatus::kFail;
        }
        FUMI_LOGI(kTAG, "mcp4728: eeprom verified");

        return ErrStatus::kOk;
    }

    FUMI_LOGE(kTAG, "failed to acquire onboard i2c bus");
    return ErrStatus::kTimeout;
}

ErrStatus hwovp_init(i2c::Bus *bus)
{
    return init_mcp4728_dac(bus);
}

} // namespace anonymous

ErrStatus Janus::init(const Config& config)
{
    if (!config.ctrl || !config.bus || !config.gdrv_pwren_port)
        return ErrStatus::kInvalidArg;

    cfg_ = config;
    ctrl_ = config.ctrl;

    q_setpoint_.init();

    ErrStatus err = hwovp_init(cfg_.bus);
    if (err != ErrStatus::kOk)
        return err;

    // Disable the gate driver's power domain.
    // This includes the FET gate driver and high-side isolated power
    // supply.
    LL_GPIO_ResetOutputPin(cfg_.gdrv_pwren_port, cfg_.gdrv_pwren_pin);

    // Configure PCA9536 and enable the high-side power supply
    if (auto bus = cfg_.bus->acquire(pdMS_TO_TICKS(100)))
    {
        namespace d = pca9536;

        i2c::Dev dev(*bus, d::kDevAddr, XferMethod::kIRQ, pdMS_TO_TICKS(100));
        auto seq = dev.update(
            PcaOdrPortHighBiasEn1::assign(d::Level::kHigh),
            PcaOdrPortHighBiasEn2::assign(d::Level::kHigh)
        ).and_then([&] {
            return dev.update(
                PcaCrPortHighBiasEn1::assign(d::Dir::kOut),
                PcaCrPortHighBiasEn2::assign(d::Dir::kOut));
        });

        if (!seq.has_value())
        {
            FUMI_LOGE(kTAG, "failed to config pca9536 on i2c: %s",
                      i2c::get_err_str(seq.error()));
            return ErrStatus::kFail;
        }
    }
    else
    {
        FUMI_LOGE(kTAG, "failed to acquire the onboard i2c bus");
        return ErrStatus::kFail;
    }
    FUMI_LOGI(kTAG, "configured power stage: GDRV_PWR=off, HGBIAS1=on, HGBIAS2=on");

    // initialize control loop
    auto [reader, writer] = ctrl_pipe_
        .make_transfer_pair<Priority::kCtrloopIrq, Priority::kJanus>();
    ctrl_pipe_writer_ = std::move(writer);

    err = ctrl_->init(config.ctrl_config, std::move(reader));
    if (err != ErrStatus::kOk)
    {
        FUMI_LOGE(kTAG, "failed to initialize control: %s", get_error_str(err));
        return err;
    }
    FUMI_LOGI(kTAG, "initialized control");

    task_handle_ = xTaskCreateStatic(+[](void *self) {
        static_cast<Janus*>(self)->task_routine();
    }, "janus", kStackWords, this, config.task_priority, stack_data_, &task_data_);
    GCHECK(task_handle_ && "failed to create task");

    return ErrStatus::kOk;
}


namespace
{

ctrl::TargetSet eval_output_soa_protect(const msg::Setpoint& desired)
{
    // TODO(masshiroio): implement this

    return {
        .outstate = desired.outstate,
        .vout = desired.vout,
        .iout = desired.iout
    };
}

} // namespace anonymous

coro::PollAsync Janus::task_sequence(coro::FrameStore, Janus *self)
{
    GCHECK_OK(hermes::get_default_bus().sub_overwrite(self->q_setpoint_),
              "failed to subscribe event on hermes bus");

    hermes::Message<msg::Setpoint::Topic> req_sp{};
    req_sp.payload.outstate = ctrl::OutputState::kOff;
    req_sp.payload.vout = kDefaultVout;
    req_sp.payload.iout = kDefaultIout;

    bool gdrv_enabled = false;
    TickType_t standby_start_tick = 0;

    auto gdrv_enable = [&] {
        gdrv_enabled = true;
        LL_GPIO_SetOutputPin(self->cfg_.gdrv_pwren_port,
            self->cfg_.gdrv_pwren_pin);
    };

    auto gdrv_disable = [&] {
        gdrv_enabled = false;
        LL_GPIO_ResetOutputPin(self->cfg_.gdrv_pwren_port,
                    self->cfg_.gdrv_pwren_pin);
    };

    while (true)
    {
        TickType_t wait_ticks = portMAX_DELAY;
        if (gdrv_enabled)
        {
            const TickType_t elapsed = xTaskGetTickCount() - standby_start_tick;
            if (elapsed >= kHibernateTicks)
            {
                // enter hibernate state
                gdrv_disable();
                FUMI_LOGI(kTAG, "hibernate: gdrv power disabled");
                continue;
            }
            wait_ticks = kHibernateTicks - elapsed;
        }

        coro::Cond status = co_await coro::queue_recv(wait_ticks, self->q_setpoint_, req_sp);

        if (status != coro::Cond::kMet)
        {
            // no messages arrived during `kHibernateTicks`
            gdrv_disable();
            FUMI_LOGI(kTAG, "hibernate: gdrv power disabled");
            continue;
        }
        if (req_sp.payload.outstate == ctrl::OutputState::kOff)
            continue;

        // an ON state is requested
        const ctrl::TargetSet final_sp = eval_output_soa_protect(req_sp.payload);
        if (final_sp.outstate == ctrl::OutputState::kOff)
        {
            FUMI_LOGW(kTAG, "requested output (%.02fV,%.02fA) was rejected by SOA protection",
                final_sp.vout, final_sp.iout);
            continue;
        }

        if (!gdrv_enabled)
        {
            FUMI_LOGI(kTAG, "recovering from hibernation...");

            // enable gate driver power domain, and wait for the
            // power rail and hardware being stable.
            gdrv_enable();

            const TickType_t start_tick = xTaskGetTickCount();
            TickType_t used_ticks = 0;
            bool request_cancelled = false;
            while (used_ticks < kGdrvPwrWaitTicks)
            {
                status = co_await coro::queue_recv(
                    kGdrvPwrWaitTicks - used_ticks, self->q_setpoint_, req_sp);

                if (status == coro::Cond::kMet &&
                    req_sp.payload.outstate == ctrl::OutputState::kOff)
                {
                    // Sender is ONLY allowed to cancel the output enable request,
                    // by sending an output disable request, during the power-on sequence.
                    // Output parameters cannot be changed.
                    gdrv_disable();
                    request_cancelled = true;
                    break;
                }

                used_ticks = xTaskGetTickCount() - start_tick;
            }

            if (request_cancelled)
            {
                FUMI_LOGI(kTAG, "output enable requested was cancelled");
                continue;
            }
        }
        else
        {
            // hot start
            FUMI_LOGI(kTAG, "hot start, not in hibernation");
        }

        // we are ready, submit the setpoint to control loop
        FUMI_LOGI(kTAG, "output enabled, setpoint (%.02fV,%.02fA)", final_sp.vout, final_sp.iout);
        self->ctrl_pipe_writer_.store(final_sp);

        while (true)
        {
            co_await coro::queue_recv(portMAX_DELAY, self->q_setpoint_, req_sp);
            if (req_sp.payload.outstate == ctrl::OutputState::kOn)
            {
                // update setpoint
                const ctrl::TargetSet update_sp = eval_output_soa_protect(req_sp.payload);
                if (update_sp.outstate == ctrl::OutputState::kOff)
                {
                    // rejected
                    continue;
                }

                FUMI_LOGD(kTAG, "update output setpoint (%.02fV,%.02fA)", update_sp.vout, update_sp.iout);
                self->ctrl_pipe_writer_.store(update_sp);
            }
            else if (req_sp.payload.outstate == ctrl::OutputState::kOff)
            {
                // disable output
                self->ctrl_pipe_writer_.store({
                    .outstate = ctrl::OutputState::kOff,
                    .vout = 0,
                    .iout = 0
                });
                FUMI_LOGI(kTAG, "output disabled");
                break;
            }
        }

        standby_start_tick = xTaskGetTickCount();
    }
}

[[noreturn]] void Janus::task_routine()
{
    FUMI_LOGI(kTAG, "janus task has been started");

    const coro::PollAsync task = task_sequence(coro_frame_buf_.as_store(), this);

    TickType_t last_wake_tick = xTaskGetTickCount();
    while (true)
    {
        // uint32_t st = uxTaskGetStackHighWaterMark(nullptr);
        // FUMI_LOGI(kTAG, "task stack high water mark: %lu", st);

        // TODO(masshiroio): get sensor measurements and compute SOA, applying them
        //                   to the SOA eval function.

        task.poll(xTaskGetTickCount());

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(5));
    }
}

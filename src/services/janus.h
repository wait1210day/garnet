// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_JANUS_H
#define GARNET_JANUS_H

#include "FreeRTOS.h"
#include "task.h"

#include "base/err.h"
#include "base/priorities.h"
#include "base/atomic_snapshot.h"
#include "base/coroutine.h"
#include "drivers/i2c_bus.h"
#include "control/ctrloop.h"
#include "services/hermes_messages.h"

class Janus
{
public:
    constexpr static size_t kStackWords = 256;
    constexpr static float kDefaultVout = 5.0f;
    constexpr static float kDefaultIout = 2.0f;

    struct Config
    {
        Ctrloop::Config ctrl_config;
        Ctrloop *ctrl;
        i2c::Bus *bus;
        uint32_t task_priority;
        GPIO_TypeDef *gdrv_pwren_port;
        uint32_t gdrv_pwren_pin;
    };

    ErrStatus init(const Config& config);

private:
    static coro::PollAsync task_sequence(coro::FrameStore, Janus *self);

    [[noreturn]] void task_routine();

    Config cfg_{};
    Ctrloop *ctrl_{nullptr};
    StaticTask_t task_data_{};
    TaskHandle_t task_handle_{};
    alignas(portBYTE_ALIGNMENT) StackType_t stack_data_[kStackWords]{};

    AtomicSnapshot<CtrlTargetSet> ctrl_pipe_{
        {
            .outstate = CtrlOutputState::kOff,
            .vout = kDefaultVout,
            .iout = kDefaultIout
        }
    };
    using PipeWriter = AtomicSnapshot<CtrlTargetSet>::Writer<Priority::kJanus>;
    using PipeReader = AtomicSnapshot<CtrlTargetSet>::Reader<Priority::kCtrloopIrq>;
    PipeWriter ctrl_pipe_writer_;

    coro::FrameBuffer<256> coro_frame_buf_{};
    hermes::OverwriteQueue<msg::Setpoint::Topic> q_setpoint_{StaticInitOption::kDefer};
};

#endif //GARNET_JANUS_H

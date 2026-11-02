// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_ERR_H
#define GARNET_ERR_H

#include <utility>

enum class ErrStatus
{
    kOk = 0,
    kFail,
    kInvalidArg,
    kUnsupported,
    kNoMem,
    kTimeout,

    kNotInitialized,
    kInvalidContext,
    kBusy,

    kDma,
    kOverrun,

    kI2cNack,
    kI2cBus,
    kI2cArbitrationLost,
    kI2cUnexpectedStop
};

#define GARNET_ERR_STATUS_STR_LIST(V) \
    V(ErrStatus::kOk, "success") \
    V(ErrStatus::kFail, "generic error") \
    V(ErrStatus::kInvalidArg, "invalid argument") \
    V(ErrStatus::kUnsupported, "unsupported operation") \
    V(ErrStatus::kNoMem, "out of memory") \
    V(ErrStatus::kTimeout, "timeout") \
    V(ErrStatus::kNotInitialized, "not initialized") \
    V(ErrStatus::kInvalidContext, "invalid context") \
    V(ErrStatus::kBusy, "device or resource busy") \
    V(ErrStatus::kDma, "dma transfer error") \
    V(ErrStatus::kOverrun, "buffer overrun error") \
    V(ErrStatus::kI2cNack, "i2c device responds NACK") \
    V(ErrStatus::kI2cBus, "i2c bus error") \
    V(ErrStatus::kI2cArbitrationLost, "i2c arbitration lost") \
    V(ErrStatus::kI2cUnexpectedStop, "i2c unexpected stop")

const char *get_error_str(ErrStatus err);

template<typename F>
class ScopeExitGuard
{
public:
    explicit ScopeExitGuard(F&& f) : func_{std::forward<F>(f)} {}
    ~ScopeExitGuard() {
        if (!executed_)
            func_();
    }

    void advance()
    {
        func_();
        executed_ = true;
    }

private:
    bool executed_{false};
    F func_;
};

#endif //GARNET_ERR_H

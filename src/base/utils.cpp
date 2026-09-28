// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include "base/err.h"

const char *get_error_str(ErrStatus err)
{
    switch (err)
    {
        case ErrStatus::kOk:
            return "success";
        case ErrStatus::kInvalidArg:
            return "invalid argument";
        case ErrStatus::kUnsupported:
            return "unsupported operation";
        case ErrStatus::kTimeout:
            return "timed out";
        case ErrStatus::kNoMem:
            return "memory allocation failed";
        case ErrStatus::kFail:
            return "generic error";
        default:
            return "unknown error";
    }
}

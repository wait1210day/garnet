// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_ERR_H
#define GARNET_ERR_H

enum class ErrStatus
{
    kOk = 0,
    kFail,
    kInvalidArg,
    kUnsupported,
    kNoMem,
    kTimeout,
};

const char *get_error_str(ErrStatus err);

#endif //GARNET_ERR_H

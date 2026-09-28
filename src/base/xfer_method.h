// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_XFER_METHOD_H
#define GARNET_XFER_METHOD_H

enum class XferMethod : uint8_t
{
    kIRQ,
    kDMA,
    kPolling
};

#endif //GARNET_XFER_METHOD_H

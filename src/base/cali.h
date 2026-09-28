// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_CALI_H
#define GARNET_CALI_H

struct AdcCaliCoeffs
{
    struct PerMeasure
    {
        float k;
        float b;
    };
    
    PerMeasure v1;
    PerMeasure v2;
    PerMeasure i1;
    PerMeasure i2;
    PerMeasure il_inv;
};

#endif //GARNET_CALI_H

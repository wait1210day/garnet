// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_ADC_OPTIMIZER_H
#define GARNET_ADC_OPTIMIZER_H

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include "base/defs.h"
#include "base/err.h"

struct adc_optimizer_config
{
    uint32_t adc_trig_cmp_min;
    uint32_t adc_trig_cmp_max;
    uint32_t single_sample_win_size;

    uint32_t integral_win_size;
    uint32_t integral_step_size;
};

enum adc_optimizer_state
{
    // TODO(masshiroio): complete this
    ADC_OPTIM_STATE_SCAN,
    ADC_OPTIM_STATE_OBSERVE
};

struct adc_optimizer_context
{
    struct adc_optimizer_config config;
    TaskHandle_t task;

    // FIXME(masshiroio): non-atomic r/w?
#define ADC_OPTIM_TRANSIENT_EDGE_NONE   (-1)
#define ADC_OPTIM_NB_TRANSIENT_EDGES    4
    volatile int transient_edges[ADC_OPTIM_NB_TRANSIENT_EDGES];

    float err_sse;
    float err_sum;

    uint32_t err_sample_cnt;
    volatile uint32_t single_sample_win_size;
    volatile float last_err_sse;
    volatile float last_err_sum;

    volatile uint32_t adc_trig_cmp;

    enum adc_optimizer_state state;
};

GARNET_ALWAYS_INLINE static inline
void adc_optimizer_update_sample(struct adc_optimizer_context *ctx, float err)
{
    ctx->err_sum += err;
    ctx->err_sse += err * err;
    ctx->err_sample_cnt++;
    if (ctx->err_sample_cnt >= ctx->single_sample_win_size)
    {
        ctx->last_err_sum = ctx->err_sum;
        ctx->last_err_sse = ctx->err_sse;
        ctx->err_sum = 0;
        ctx->err_sse = 0;
        ctx->err_sample_cnt = 0;
    }
}

GARNET_ALWAYS_INLINE static inline
uint32_t adc_optimizer_get_adc_trig_cmp(struct adc_optimizer_context *ctx)
{
    return ctx->adc_trig_cmp;
}

ErrStatus adc_optimizer_init(struct adc_optimizer_context *ctx, const struct adc_optimizer_config *config);

#endif //GARNET_ADC_OPTIMIZER_H

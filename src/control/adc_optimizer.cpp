// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstring>
#include <cmath>

#include "base/priorities.h"
#include "control/adc_optimizer.h"
#include "services/fumi.h"

static const char *kTAG = "adc_optimizer";

static float get_current_average_sse(struct adc_optimizer_context *ctx)
{
    float sse_avg = 0;
    for (int i = 0; i < 3; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        sse_avg += ctx->last_err_sse;
    }
    return sse_avg / 3.0f;
}

static float observe_sample_integral_win(struct adc_optimizer_context *ctx, uint32_t start_pos)
{
    const uint32_t end_pos = start_pos + ctx->config.integral_win_size;
    float sum_of_sse = 0;

    uint32_t nb_samples = 0;
    for (uint32_t curr = start_pos; curr < end_pos && curr < ctx->config.adc_trig_cmp_max;
         curr += ctx->config.integral_step_size)
    {
        ctx->adc_trig_cmp = start_pos;
        vTaskDelay(pdMS_TO_TICKS(1));

        const float sse_avg = get_current_average_sse(ctx);
        sum_of_sse += sse_avg;
        nb_samples++;
    }

    return sum_of_sse / (float)nb_samples;
}

static uint32_t scan_global_sse_valley(struct adc_optimizer_context *ctx, float *out_min_sse)
{
    uint32_t min_trig = 0;
    float min_sse = INFINITY;

    const uint32_t winsize = ctx->config.integral_win_size;

    for (uint32_t curr = ctx->config.adc_trig_cmp_min; curr < ctx->config.adc_trig_cmp_max; curr += winsize)
    {
        float sum_sse = observe_sample_integral_win(ctx, curr);
        FUMI_LOGI(kTAG, "%lu %.04f", curr, sum_sse);

        if (sum_sse < min_sse)
        {
            min_sse = sum_sse;
            min_trig = curr;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (out_min_sse)
        *out_min_sse = min_sse;

    FUMI_LOGI(kTAG, "found minimal noise trig in [%lu, %lu] sse=%.04f", min_trig, min_trig + winsize, min_sse);
    return min_trig + winsize / 2;
}

static GARNET_NORETURN void adc_optimizer_task(void *param)
{
    struct adc_optimizer_context *ctx = reinterpret_cast<struct adc_optimizer_context *>(param);

    // TODO(masshiroio): this function needs reimplementation, and here is just a
    //                   placeholder to set the sample point at a fixed position (no actual optimizations).

    float last_min_sse = -10000.0f;
    int need_rescan_cnt = 0;
    float rescan_thresh = 0;
    while (1)
    {
        switch (ctx->state)
        {
            case ADC_OPTIM_STATE_SCAN:
            {
                /*
                const uint32_t trig = scan_global_sse_valley(ctx, &last_min_sse);

                // rescan_threshold = baseline * (1 + K_rel_tol) + C_abs_tol
                rescan_thresh = last_min_sse * (1 + 0.4f) + 0.05f;
                ctx->adc_trig_cmp = trig;
                ctx->state = ADC_OPTIM_STATE_OBSERVE;

                FUMI_LOGI(kTAG, "next rescan threshold is sse > %.04f", rescan_thresh);
                */

                ctx->adc_trig_cmp = 2960;
                break;
            }

            case ADC_OPTIM_STATE_OBSERVE:
            {
                /*
                const float current_sse = get_current_average_sse(ctx);
                if (current_sse > rescan_thresh)
                    need_rescan_cnt += 2;
                else if (need_rescan_cnt > 0)
                    need_rescan_cnt -= 1;

                if (need_rescan_cnt >= 10)
                {
                    FUMI_LOGI(kTAG, "rescan required: sse=%.04f (last min %.04f), rescan_cnt=%d",
                              current_sse, last_min_sse, need_rescan_cnt);

                    need_rescan_cnt = 0;
                    ctx->state = ADC_OPTIM_STATE_SCAN;
                }
                */
                vTaskDelay(pdMS_TO_TICKS(3000));
                ctx->state = ADC_OPTIM_STATE_SCAN;
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


ErrStatus adc_optimizer_init(struct adc_optimizer_context *ctx,
                         const struct adc_optimizer_config *config)
{
    if (config->adc_trig_cmp_min > config->adc_trig_cmp_max)
        return ErrStatus::kInvalidArg;

    memset(ctx, 0, sizeof(*ctx));
    memcpy(&ctx->config, config, sizeof(*config));

    for (int i = 0; i < ADC_OPTIM_NB_TRANSIENT_EDGES; i++)
        ctx->transient_edges[i] = ADC_OPTIM_TRANSIENT_EDGE_NONE;

    ctx->single_sample_win_size = config->single_sample_win_size;
    ctx->adc_trig_cmp = config->adc_trig_cmp_min;
    ctx->state = ADC_OPTIM_STATE_SCAN;

    BaseType_t task_creation_result = xTaskCreate(adc_optimizer_task, "adc_optimizer", 256,
        ctx, PriorityTraits<Priority::kAdcOptimizer>::to_freertos(), &ctx->task);
    if (task_creation_result != pdPASS)
    {
        FUMI_LOGE(kTAG, "failed to create adc triggering optimizer task: out of memory");
        return ErrStatus::kNoMem;
    }

    return ErrStatus::kOk;
}

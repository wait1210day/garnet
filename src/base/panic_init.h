// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_PANIC_INIT_H
#define GARNET_PANIC_INIT_H

#include <cstddef>
#include <cstdint>

#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

#include "base/panic.h"

namespace panic
{

struct Config
{
    USART_TypeDef   *uart;
    GPIO_TypeDef    *led_gpio;
    uint32_t         led_gpio_pin;
    bool             led_active_low;
};

void init_panic_resources(const Config& config);

} // namespace panic

#endif //GARNET_PANIC_INIT_H

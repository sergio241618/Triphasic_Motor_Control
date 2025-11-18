#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    mcpwm_unit_t pwm_unit;        // MCPWM_UNIT_0
    mcpwm_timer_t pwm_timer;      // MCPWM_TIMER_0/1/2
    mcpwm_operator_t pwm_op;      // MCPWM_OPR_A (use A as main)
    gpio_num_t gpio_num;          // GPIO for operator A (high-side)
    float duty_cycle;             // current duty % (0..100)
    float phase_offset_deg;       // 0, 120, 240
    QueueHandle_t queue;          // (Legacy queue, no longer used in this logic)
    const char* name_id;          // Name for logs (e.g. "CH1-Freq")
} pwm_task_config_t;

// HAL functions (implemented inside phases.cpp)
esp_err_t pwm_hal_init(void);
esp_err_t pwm_hal_configure_timer_and_pin(const pwm_task_config_t* cfg, gpio_num_t gpio_op_a, gpio_num_t gpio_op_b);
esp_err_t pwm_hal_set_deadtime(mcpwm_unit_t unit, mcpwm_timer_t timer, uint32_t red_ticks, uint32_t fed_ticks);
esp_err_t pwm_hal_set_duty_percent(const pwm_task_config_t* cfg, float duty_percent);

#ifdef __cplusplus
}
#endif
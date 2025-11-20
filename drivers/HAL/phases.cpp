/**
 * phases.cpp  -- MCPWM + theta management + PWM HAL (integrated)
 * (Compatible with ESP-IDF v4.4)
 * (Compatible with app_main notification architecture)
 */

#include "phases.hpp"
#include "pwm.h"
#include "esp_log.h"
#include "driver/mcpwm.h" 
#include "driver/gpio.h"
#include "esp_timer.h"
#include <cmath>

using namespace phases;

static const char LOG_TAG[] = "phases_hal";

// ==============================
//  INTERNAL VARIABLES
// ==============================
static bool init_ok = false;
bool phases::init_phases_ok(void) { return init_ok; }

static esp_timer_handle_t sine_generator_timer_handle = nullptr;

static volatile uint32_t _A_theta_int = 0;
static volatile uint32_t _angular_speed_int = 0;

static float g_amplitude = 1.0f;
static uint32_t div_fact = PWM_MAX_VAL;

static uint32_t MAX_ANGULAR_SPEED_int = 0;
static float MAX_ANGULAR_SPEED_rads = 0.0f;
static float MAX_FREQUENCY_hz = 0.0f;

static constexpr uint32_t SINE_STEPS = SINE_LUT_IDX_RESOLUTION;

// --- Notification function prototype ---
// This function must exist in app_main.cpp
extern void phases_notify_pwm_tasks_from_isr(void);

// ==============================
//  FORWARD UTILS
// ==============================
uint32_t phases::hz_to_delta_theta_int(float frequency_hz) {
    return (uint32_t)std::ceil((double)frequency_hz * SINE_WAVE_SAMPLE_TIMEs * (double)MAX_THETA_INT);
}
uint32_t phases::w_to_delta_theta_int(float angular_speed_rads) {
    return (uint32_t)std::ceil((double)angular_speed_rads * SINE_WAVE_SAMPLE_TIMEs * (double)MAX_THETA_INT / (double)M_TAU);
}
uint32_t phases::rad_to_theta_int(float x) {
    while (x > M_TAU) x -= M_TAU;
    while (x <  0.0f) x += M_TAU;
    return (uint32_t)(x * (double)MAX_THETA_INT / (double)M_TAU);
}

uint32_t phases::phases_get_theta_int(void) {
    return _A_theta_int;
}

// ==============================
//  TIMER ISR: increments theta AND NOTIFIES
// ==============================
void IRAM_ATTR phases::phase_output_intr(void* args) {
    (void)args;
    // 1. Increment Theta
    _A_theta_int += _angular_speed_int;
    
    // 2. Notify PWM tasks that there is a new value
    phases_notify_pwm_tasks_from_isr();
}

// ==============================
//  INIT PHASES
// ==============================
bool phases::init_phases(void) {
    esp_err_t err;

    // power pin
    gpio_config_t enable_pin_config = {
        .pin_bit_mask = (1ULL << POWER_ON_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    err = gpio_config(&enable_pin_config);
    if (err != ESP_OK) {
        ESP_LOGE(LOG_TAG, "gpio_config error: %s", esp_err_to_name(err));
        return false;
    }
    gpio_set_level(POWER_ON_GPIO, 0);

    MAX_ANGULAR_SPEED_rads = (M_TAU / SINE_WAVE_SAMPLE_TIMEs) * 0.9f;
    MAX_FREQUENCY_hz = 0.9f / SINE_WAVE_SAMPLE_TIMEs;
    MAX_ANGULAR_SPEED_int = w_to_delta_theta_int(MAX_ANGULAR_SPEED_rads);

    // Create esp_timer for theta stepping
    esp_timer_create_args_t sine_generator_timer_cfg = {
        .callback = &phases::phase_output_intr, // <- Calls the ISR (which now notifies)
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_ISR,
        .name = "theta_step",
        .skip_unhandled_events = false
    };
    err = esp_timer_create(&sine_generator_timer_cfg, &sine_generator_timer_handle);
    if (err != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Error creating timer: %s", esp_err_to_name(err));
        return false;
    }

    init_ok = true;
    ESP_LOGI(LOG_TAG, "phases init ok");
    return true;
}

void phases::start_phases(void) {
    if (!init_ok) {
        ESP_LOGE(LOG_TAG, "phases not initialized");
        return;
    }
    // IMPORTANT! The timer is started AFTER creating the tasks
    esp_err_t err = esp_timer_start_periodic(sine_generator_timer_handle, SINE_WAVE_SAMPLE_TIMEus);
    if (err != ESP_OK) {
        ESP_LOGE(LOG_TAG, "esp_timer_start_periodic failed: %s", esp_err_to_name(err));
        return;
    }
    gpio_set_level(POWER_ON_GPIO, 1);
}

void phases::stop_phases(void) {
    if (sine_generator_timer_handle) {
        esp_timer_stop(sine_generator_timer_handle);
    }
    gpio_set_level(POWER_ON_GPIO, 0);
}

void phases::kill_phases(void) {
    stop_phases();
    g_amplitude = 0.0f;
}

bool phases::is_active_phases(void) {
    if (!sine_generator_timer_handle) return false;
    return esp_timer_is_active(sine_generator_timer_handle);
}

// amplitude / frequency
void phases::set_amplitude(const float amplitude) {
    if (amplitude < 0.0f) {
        ESP_LOGE(LOG_TAG, "amplitude negative clipped");
        g_amplitude = 0.0f;
        return;
    }
    if (amplitude > 1.0f) {
        ESP_LOGW(LOG_TAG, "amplitude clipped to 1");
        g_amplitude = 1.0f;
    } else {
        g_amplitude = amplitude;
    }
    div_fact = (g_amplitude == 0.0f) ? PWM_MAX_VAL : (uint32_t)std::ceil(1.0f / (double)g_amplitude);
}
float phases::get_amplitude(void) { return g_amplitude; }

void phases::set_frequency(const float frequency_hz) {
    if (frequency_hz < 0.0f) {
        ESP_LOGE(LOG_TAG, "Invalid frequency negative");
        _angular_speed_int = 0;
        return;
    }
    if (frequency_hz > MAX_FREQUENCY_hz) {
        ESP_LOGW(LOG_TAG, "frequency clipped to max");
        _angular_speed_int = MAX_ANGULAR_SPEED_int;
        return;
    }
    _angular_speed_int = hz_to_delta_theta_int(frequency_hz);
}

void phases::set_angular_speed(const float angular_speed_rads) {
    if (angular_speed_rads < 0.0f) {
        ESP_LOGE(LOG_TAG, "Invalid angular speed negative");
        _angular_speed_int = 0;
        return;
    }
    if (angular_speed_rads > MAX_ANGULAR_SPEED_rads) {
        ESP_LOGW(LOG_TAG, "angular speed clipped");
        _angular_speed_int = MAX_ANGULAR_SPEED_int;
        return;
    }
    _angular_speed_int = w_to_delta_theta_int(angular_speed_rads);
}

float phases::get_angular_speed(void) {
    return M_TAU * (double)_angular_speed_int / (SINE_WAVE_SAMPLE_TIMEs * (double)MAX_THETA_INT);
}
float phases::get_frequency(void) {
    return (double)_angular_speed_int / (SINE_WAVE_SAMPLE_TIMEs * (double)MAX_THETA_INT);
}

// ============================================================================
//   PWM HAL
// ============================================================================
static const char *PWM_HAL_TAG = "pwm_hal_impl";

esp_err_t pwm_hal_init(void) {
    ESP_LOGI(PWM_HAL_TAG, "pwm_hal_init ok");
    return ESP_OK;
}

esp_err_t pwm_hal_configure_timer_and_pin(const pwm_task_config_t* cfg, gpio_num_t gpio_op_a, gpio_num_t gpio_op_b)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    // map GPIOs per timer
    if (cfg->pwm_timer == PWM_TIMER_ID0) {
        mcpwm_gpio_init(cfg->pwm_unit, MCPWM0A, gpio_op_a);
        mcpwm_gpio_init(cfg->pwm_unit, MCPWM0B, gpio_op_b);
    } else if (cfg->pwm_timer == PWM_TIMER_ID1) {
        mcpwm_gpio_init(cfg->pwm_unit, MCPWM1A, gpio_op_a);
        mcpwm_gpio_init(cfg->pwm_unit, MCPWM1B, gpio_op_b);
    } else {
        mcpwm_gpio_init(cfg->pwm_unit, MCPWM2A, gpio_op_a);
        mcpwm_gpio_init(cfg->pwm_unit, MCPWM2B, gpio_op_b);
    }

    // configure timer
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY_Hz;
    pwm_config.cmpr_a = 0.0;
    pwm_config.cmpr_b = 0.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    esp_err_t err = mcpwm_init(cfg->pwm_unit, cfg->pwm_timer, &pwm_config);
    if (err != ESP_OK) {
        ESP_LOGE(PWM_HAL_TAG, "mcpwm_init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(PWM_HAL_TAG, "Configured MCPWM unit %d timer %d pins %d/%d",
             (int)cfg->pwm_unit, (int)cfg->pwm_timer, (int)gpio_op_a, (int)gpio_op_b);

    return ESP_OK;
}

esp_err_t pwm_hal_set_deadtime(mcpwm_unit_t unit, mcpwm_timer_t timer, uint32_t red_us, uint32_t fed_us)
{
    uint32_t factor = 19; 

    uint32_t red_ticks = red_us * factor;
    uint32_t fed_ticks = fed_us * factor;

    esp_err_t err = mcpwm_deadtime_enable(unit, timer, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, red_ticks, fed_ticks);
    
    if (err != ESP_OK) {
        ESP_LOGE(PWM_HAL_TAG, "mcpwm_deadtime_enable failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(PWM_HAL_TAG, "deadtime set: %u us -> %u ticks (Factor: %u)", (unsigned)red_us, (unsigned)red_ticks, (unsigned)factor);
    }
    return err;
}

esp_err_t pwm_hal_set_duty_percent(const pwm_task_config_t* cfg, float duty_percent)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    mcpwm_set_duty(cfg->pwm_unit, cfg->pwm_timer, cfg->pwm_op, duty_percent);
    mcpwm_set_duty_type(cfg->pwm_unit, cfg->pwm_timer, cfg->pwm_op, MCPWM_DUTY_MODE_0);

    return ESP_OK;
}
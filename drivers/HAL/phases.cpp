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
static volatile uint32_t _target_angular_speed_int = 0;  // Target frequency for ramping
static volatile uint32_t _slew_rate_int = 0;              // Slew rate in int units per ISR tick

static float g_amplitude = 1.0f;
static float g_slew_rate_hz_per_s = 50.0f;                // Default: 50 Hz/s
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
    
    // 1. Apply slew rate limiting (frequency ramp)
    if (_angular_speed_int < _target_angular_speed_int) {
        // Accelerating
        uint32_t diff = _target_angular_speed_int - _angular_speed_int;
        if (diff > _slew_rate_int) {
            _angular_speed_int += _slew_rate_int;
        } else {
            _angular_speed_int = _target_angular_speed_int;
        }
    } else if (_angular_speed_int > _target_angular_speed_int) {
        // Decelerating
        uint32_t diff = _angular_speed_int - _target_angular_speed_int;
        if (diff > _slew_rate_int) {
            _angular_speed_int -= _slew_rate_int;
        } else {
            _angular_speed_int = _target_angular_speed_int;
        }
    }
    
    // 2. Increment Theta
    _A_theta_int += _angular_speed_int;
    
    // 3. Notify PWM tasks that there is a new value
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
    
    // Initialize slew rate (default 50 Hz/s)
    _slew_rate_int = hz_to_delta_theta_int(g_slew_rate_hz_per_s * SINE_WAVE_SAMPLE_TIMEs);

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
    // Motor operating range limits: 20 Hz to 60 Hz
    const float MIN_MOTOR_FREQ = 20.0f;
    const float MAX_MOTOR_FREQ = 60.0f;
    
    float limited_freq = frequency_hz;
    
    // Apply motor operating range limits
    if (limited_freq < MIN_MOTOR_FREQ) {
        ESP_LOGW(LOG_TAG, "Frequency %.2f Hz below minimum, clamping to %.2f Hz", frequency_hz, MIN_MOTOR_FREQ);
        limited_freq = MIN_MOTOR_FREQ;
    } else if (limited_freq > MAX_MOTOR_FREQ) {
        ESP_LOGW(LOG_TAG, "Frequency %.2f Hz above maximum, clamping to %.2f Hz", frequency_hz, MAX_MOTOR_FREQ);
        limited_freq = MAX_MOTOR_FREQ;
    }
    
    // Additional check against hardware maximum
    if (limited_freq > MAX_FREQUENCY_hz) {
        ESP_LOGW(LOG_TAG, "Frequency clipped to hardware max");
        limited_freq = MAX_FREQUENCY_hz;
    }
    
    uint32_t new_target = hz_to_delta_theta_int(limited_freq);
    
    // If current speed is 0, initialize it to avoid ramping from 0
    if (_angular_speed_int == 0 && new_target > 0) {
        _angular_speed_int = new_target;
    }
    
    // Set target frequency - ISR will ramp to it smoothly
    _target_angular_speed_int = new_target;
}

void phases::set_angular_speed(const float angular_speed_rads) {
    if (angular_speed_rads < 0.0f) {
        ESP_LOGE(LOG_TAG, "Invalid angular speed negative");
        _target_angular_speed_int = 0;
        return;
    }
    if (angular_speed_rads > MAX_ANGULAR_SPEED_rads) {
        ESP_LOGW(LOG_TAG, "angular speed clipped");
        _target_angular_speed_int = MAX_ANGULAR_SPEED_int;
        return;
    }
    // Set target angular speed - ISR will ramp to it smoothly
    _target_angular_speed_int = w_to_delta_theta_int(angular_speed_rads);
}

float phases::get_angular_speed(void) {
    return M_TAU * (double)_angular_speed_int / (SINE_WAVE_SAMPLE_TIMEs * (double)MAX_THETA_INT);
}
float phases::get_frequency(void) {
    return (double)_angular_speed_int / (SINE_WAVE_SAMPLE_TIMEs * (double)MAX_THETA_INT);
}

// ==============================
//  FREQUENCY SLEW RATE LIMITER
// ==============================
void phases::set_frequency_slew_rate(float hz_per_second) {
    if (hz_per_second < 0.0f) {
        ESP_LOGE(LOG_TAG, "Invalid slew rate negative");
        return;
    }
    
    // Limit to reasonable maximum (e.g., 1000 Hz/s)
    if (hz_per_second > 1000.0f) {
        ESP_LOGW(LOG_TAG, "Slew rate clipped to 1000 Hz/s");
        hz_per_second = 1000.0f;
    }
    
    g_slew_rate_hz_per_s = hz_per_second;
    
    // Calculate slew rate in int units per ISR tick
    // Each tick, we can change by: hz_per_second * SINE_WAVE_SAMPLE_TIMEs Hz
    float delta_hz_per_tick = hz_per_second * SINE_WAVE_SAMPLE_TIMEs;
    _slew_rate_int = hz_to_delta_theta_int(delta_hz_per_tick);
    
    ESP_LOGI(LOG_TAG, "Slew rate set to %.2f Hz/s", hz_per_second);
}

float phases::get_frequency_slew_rate(void) {
    return g_slew_rate_hz_per_s;
}

float phases::get_target_frequency(void) {
    return (double)_target_angular_speed_int / (SINE_WAVE_SAMPLE_TIMEs * (double)MAX_THETA_INT);
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

esp_err_t pwm_hal_set_deadtime(mcpwm_unit_t unit, mcpwm_timer_t timer, 
                                uint32_t red_ns, uint32_t fed_ns)
{
    const uint32_t PWM_FREQ = PWM_FREQUENCY_Hz; 
    const uint32_t TICKS_PER_PERIOD = 1000;
    
    // ns per tick
    uint32_t ns_per_tick = (1000000000UL / PWM_FREQ) / TICKS_PER_PERIOD;
    
    // Convert nanoseconds to ticks
    uint32_t red_ticks = (red_ns + ns_per_tick - 1) / ns_per_tick;  // Round up
    uint32_t fed_ticks = (fed_ns + ns_per_tick - 1) / ns_per_tick;
    
    ESP_LOGI(PWM_HAL_TAG, "Deadtime: %u ns -> %u ticks (rising)", red_ns, red_ticks);
    ESP_LOGI(PWM_HAL_TAG, "Deadtime: %u ns -> %u ticks (falling)", fed_ns, fed_ticks);
    
    esp_err_t err = mcpwm_deadtime_enable(unit, timer, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 
                                          red_ticks, fed_ticks);
    
    if (err != ESP_OK) {
        ESP_LOGE(PWM_HAL_TAG, "mcpwm_deadtime_enable failed: %s", esp_err_to_name(err));
    }
    
    return err;
}

esp_err_t pwm_hal_set_duty_percent(const pwm_task_config_t* cfg, float duty_percent)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    // Only set duty - duty_type is already configured during initialization
    // Calling mcpwm_set_duty_type() repeatedly can cause glitches
    mcpwm_set_duty(cfg->pwm_unit, cfg->pwm_timer, cfg->pwm_op, duty_percent);

    return ESP_OK;
}
#pragma once
/**
 * phases.hpp
 * Constants and public API for the phases (SPWM) module - ESP-IDF v4.4 compatible
 */

#include <cstdint>
#include "math.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "../sine_LUT.hpp"
#include "pwm.h"

namespace phases {

// constants and mappings
constexpr float M_TAU = M_PI * 2.0f;
constexpr uint32_t MAX_THETA_INT = UINT32_MAX;
constexpr uint32_t SINE_WAVE_SAMPLE_TIMEus = CONFIG_SINE_WAVE_SAMPLE_TIMEus;
constexpr float    SINE_WAVE_SAMPLE_TIMEs  = SINE_WAVE_SAMPLE_TIMEus * 1e-6f;
constexpr uint32_t DEAD_TIME_nsX100 = 10u;

// MCPWM mapping - legacy
constexpr mcpwm_unit_t MCPWM_UNIT_USED = MCPWM_UNIT_0;
constexpr mcpwm_timer_t PWM_TIMER_ID0 = MCPWM_TIMER_0;
constexpr mcpwm_timer_t PWM_TIMER_ID1 = MCPWM_TIMER_1;
constexpr mcpwm_timer_t PWM_TIMER_ID2 = MCPWM_TIMER_2;
constexpr uint32_t PWM_FREQUENCY_Hz = 20000u;

// pins
constexpr gpio_num_t A_HIGH_GPIO = (gpio_num_t)5;
constexpr gpio_num_t A_LOW_GPIO  = (gpio_num_t)26;
constexpr gpio_num_t B_HIGH_GPIO = (gpio_num_t)18;
constexpr gpio_num_t B_LOW_GPIO  = (gpio_num_t)25;
constexpr gpio_num_t C_HIGH_GPIO = (gpio_num_t)19;
constexpr gpio_num_t C_LOW_GPIO  = (gpio_num_t)33;

constexpr int POWER_ON_PIN = 23;
constexpr gpio_num_t POWER_ON_GPIO = (gpio_num_t)POWER_ON_PIN;

// Public API
bool init_phases(void);
bool init_phases_ok(void);

void start_phases(void);
void stop_phases(void);
void kill_phases(void);
bool is_active_phases(void);

void set_amplitude(const float amplitude);
float get_amplitude(void);

void set_frequency(const float frequency_hz);
void set_angular_speed(const float angular_speed_rads);
float get_angular_speed(void);
float get_frequency(void);

// Frequency slew rate limiter (rampa de frecuencia)
void set_frequency_slew_rate(float hz_per_second);
float get_frequency_slew_rate(void);
float get_target_frequency(void);

// utilities
uint32_t hz_to_delta_theta_int(float frequency_hz);
uint32_t w_to_delta_theta_int(float angular_speed_rads);
uint32_t rad_to_theta_int(float x);

// atomic read of the current theta
uint32_t phases_get_theta_int(void);

// timer callback
void IRAM_ATTR phase_output_intr(void* args);

} // namespace phases
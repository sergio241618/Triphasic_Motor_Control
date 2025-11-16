/**
 * @file phases.cpp
 * @author ACMAX (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-02-21
 * * @copyright Copyright (c) 2025
 * */
#include "phases.hpp"

#ifndef MCK_PHASE_MODULE

#include <algorithm>

#include "esp_log.h"

#include "pwm.h"

using phases::M_TAU;
using phases::MAX_THETA_INT;
using phases::SINE_WAVE_SAMPLE_TIMEus;
using phases::SINE_WAVE_SAMPLE_TIMEs;
using phases::DEAD_TIME_nsX100;
using phases::PWM_TIMER_ID;
using phases::PWM_FREQUENCY_Hz;

using phases::A_HIGH_CHANNEL;
using phases::A_LOW_CHANNEL;
using phases::B_HIGH_CHANNEL;
using phases::B_LOW_CHANNEL;
using phases::C_HIGH_CHANNEL;
using phases::C_LOW_CHANNEL;

using phases::A_HIGH_GPIO;
using phases::A_LOW_GPIO;
using phases::B_HIGH_GPIO;
using phases::B_LOW_GPIO;
using phases::C_HIGH_GPIO;
using phases::C_LOW_GPIO;

using phases::POWER_ON_PIN;
using phases::POWER_ON_GPIO;
static constexpr uint32_t GPIO_HIGH = 1;
static constexpr uint32_t GPIO_LOW  = 0;

static const char LOG_TAG[] = "phases";

static constexpr uint32_t SECOND_us     = 1000000;
static constexpr uint32_t SECOND_ns     = SECOND_us*1000;
static constexpr uint32_t SECOND_nsX100 = SECOND_ns/100;
constexpr uint32_t DEAD_TIME = 2.0*DEAD_TIME_nsX100*PWM_FREQUENCY_Hz*PWM_MAX_VAL/SECOND_nsX100;
constexpr uint32_t DEAD_TIME_2 = DEAD_TIME / 2;

constexpr int DUTYCYCLE_OFFSET = 16;
constexpr uint32_t DUTYCYCLE_MASK_LOW  = 0xFFFF;
constexpr uint32_t DUTYCYCLE_MASK_HIGH = DUTYCYCLE_MASK_LOW<<DUTYCYCLE_OFFSET;
constexpr uint32_t PLS_M_TAU_3_INT = MAX_THETA_INT/3;
constexpr uint32_t MNS_M_TAU_3_INT = (~PLS_M_TAU_3_INT) + 1;

static bool init_ok = false;
bool phases::init_phases_ok(void) {return init_ok;}

static esp_timer_handle_t sine_generator_timer_handle;

static float MAX_ANGULAR_SPEED_rads = 0.0f;
static float MAX_FREQUENCY_hz = 0.0f;
static uint32_t MAX_ANGULAR_SPEED_int = 0;
static volatile uint32_t _angular_speed_int = 0;

static volatile uint32_t div_fact  = PWM_MAX_VAL;

enum PhaseSelector {A=0, B, C};
inline void set_phase_dutycycle(PhaseSelector phase, uint32_t value);

void phases::phase_output_intr(void* args) {
    static uint32_t A_theta = 0;
    uint32_t angular_speed = _angular_speed_int;

    A_theta += angular_speed;

    uint32_t A_dutycycle = sin_lut(A_theta                );
    uint32_t B_dutycycle = sin_lut(A_theta+PLS_M_TAU_3_INT);
    uint32_t C_dutycycle = sin_lut(A_theta+MNS_M_TAU_3_INT);

    set_phase_dutycycle(A, A_dutycycle);
    set_phase_dutycycle(B, B_dutycycle);
    set_phase_dutycycle(C, C_dutycycle);
}

inline void set_phase_dutycycle(PhaseSelector phase, uint32_t value) {
    ledc_channel_t phase_component_h = A_HIGH_CHANNEL;
    ledc_channel_t phase_component_l = A_LOW_CHANNEL;
    switch (phase) {
        case A:
            phase_component_h = A_HIGH_CHANNEL;
            phase_component_l = A_LOW_CHANNEL;
            break;
        case B:
            phase_component_h = B_HIGH_CHANNEL;
            phase_component_l = B_LOW_CHANNEL;
            break;
        case C:
            phase_component_h = C_HIGH_CHANNEL;
            phase_component_l = C_LOW_CHANNEL;
            break;
    }
    uint32_t dutycycle_h = (value&DUTYCYCLE_MASK_HIGH)>>DUTYCYCLE_OFFSET;
    uint32_t dutycycle_l = value&DUTYCYCLE_MASK_LOW;

    dutycycle_h = dutycycle_h/div_fact;
    dutycycle_l = dutycycle_l/div_fact;
    ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE,
        phase_component_h, std::min(dutycycle_h+DEAD_TIME_2, PWM_MAX_VAL), 0
        // phase_component_h, dutycycle_h, 0
    );
    ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE,
        phase_component_l, (dutycycle_l<DEAD_TIME_2)?0:(dutycycle_l-DEAD_TIME_2), DEAD_TIME/2
        // phase_component_l, (dutycycle_l<DEAD_TIME)?0:(dutycycle_l-DEAD_TIME), DEAD_TIME/2
        // phase_component_l, dutycycle_l, 0
    );
}

void phases::set_amplitude(const float amplitude) {
    if (amplitude > 1.0f || amplitude < 0.0f) {
        ESP_LOGE(LOG_TAG, "Invalid amplitude, out of range! Clipping");
    }

    div_fact  = std::ceil(1/amplitude);
}
float phases::get_amplitude(void) {
    return 1/(float)div_fact;
}

void phases::set_frequency(const float frequency_hz) {
    if (frequency_hz < 0.0f) {
        ESP_LOGE(LOG_TAG, "Invalid frequency, negative! Clipping");
        _angular_speed_int = 0;
    }
    if (frequency_hz > (MAX_ANGULAR_SPEED_rads/M_TAU)) {
        ESP_LOGE(LOG_TAG, "Invalid frequency, too high! Clipping");
        _angular_speed_int = MAX_ANGULAR_SPEED_int;
        return;
    }
    _angular_speed_int = hz_to_delta_theta_int(frequency_hz);
}
void phases::set_angular_speed(const float angular_speed_rads) {
    if (angular_speed_rads < 0.0f) {
        ESP_LOGE(LOG_TAG, "Invalid angular speed, negative! Clipping");
        _angular_speed_int = 0;
        return;
    }
    if (angular_speed_rads > MAX_ANGULAR_SPEED_rads) {
        ESP_LOGE(LOG_TAG, "Invalid angular speed, too high! Clipping");
        _angular_speed_int = MAX_ANGULAR_SPEED_int;
        return;
    }
    _angular_speed_int = w_to_delta_theta_int(angular_speed_rads);
}
float phases::get_angular_speed(void) {
    return M_TAU*_angular_speed_int/(SINE_WAVE_SAMPLE_TIMEs*MAX_THETA_INT);
}
float phases::get_frequency(void) {
    return _angular_speed_int/(SINE_WAVE_SAMPLE_TIMEs*MAX_THETA_INT);
}

void phases::start_phases(void) {
    if (!init_ok) {
        ESP_LOGE(LOG_TAG, "error on initialization, cannot start!");
        return;
    }
    esp_err_t error_code = ESP_OK;
    ESP_ERROR_CHECK_WITHOUT_ABORT( esp_timer_start_periodic(
        sine_generator_timer_handle, SINE_WAVE_SAMPLE_TIMEus
    ));
    if (error_code != ESP_OK) {
        ESP_LOGE( LOG_TAG,
            "error starting phases: %s",
            esp_err_to_name(error_code)
        );
        return;
    }
    gpio_set_level(POWER_ON_GPIO, GPIO_HIGH);
}

void phases::stop_phases(void) {
    esp_err_t error_code = ESP_OK;
    gpio_set_level(POWER_ON_GPIO, GPIO_LOW);
    error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(sine_generator_timer_handle));
    if (error_code != ESP_OK) {
        ESP_LOGE( LOG_TAG,
            "error stopping phases: %s",
            esp_err_to_name(error_code)
        );
    }
}

void phases::kill_phases(void) {
    esp_err_t error_code = ESP_OK;
    gpio_set_level(POWER_ON_GPIO, GPIO_LOW);
    error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(sine_generator_timer_handle));
    if (error_code != ESP_OK) {
        ESP_LOGE( LOG_TAG,
            "error stopping phases: %s",
            esp_err_to_name(error_code)
        );
    }
    set_phase_dutycycle(A, 0);
    set_phase_dutycycle(B, 0);
    set_phase_dutycycle(C, 0);
    set_amplitude(0);
}

bool phases::is_active_phases(void) {
    return esp_timer_is_active(sine_generator_timer_handle);
}

static const char INIT_LOG_TAG[] = "phase_init";

static inline bool init_phase_channel(
    const PhaseSelector phase,
    const ledc_channel_config_t channel_base_config
) {
    ESP_LOGI(INIT_LOG_TAG, "Configuring phase %c...", 'A'+phase);
    ledc_channel_config_t channel_high_config = channel_base_config;
    ledc_channel_config_t channel_low_config  = channel_base_config;

    switch (phase) {
        case A:
            channel_high_config.gpio_num = A_HIGH_GPIO;
            channel_high_config.channel  = A_HIGH_CHANNEL;
            channel_low_config.gpio_num  = A_LOW_GPIO;
            channel_low_config.channel   = A_LOW_CHANNEL;
            break;
        case B:
            channel_high_config.gpio_num = B_HIGH_GPIO;
            channel_high_config.channel  = B_HIGH_CHANNEL;
            channel_low_config.gpio_num  = B_LOW_GPIO;
            channel_low_config.channel   = B_LOW_CHANNEL;
            break;
        case C:
            channel_high_config.gpio_num = C_HIGH_GPIO;
            channel_high_config.channel  = C_HIGH_CHANNEL;
            channel_low_config.gpio_num  = C_LOW_GPIO;
            channel_low_config.channel   = C_LOW_CHANNEL;
            break;
    }
    channel_high_config.flags.output_invert = 1;

    esp_err_t error_code = ESP_OK;
    error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_channel_config(&channel_high_config));
    if (error_code != ESP_OK) {
        ESP_LOGE( INIT_LOG_TAG,
            "Error configuring phase %c high, ERRCODE:\n%s",
            phase, esp_err_to_name(error_code));
        return false;
    }
    error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_channel_config(&channel_low_config));
    if (error_code != ESP_OK) {
        ESP_LOGE( INIT_LOG_TAG,
            "Error configuring phase %c low, ERRCODE:\n%s",
            phase, esp_err_to_name(error_code));
        return false;
    }
    ESP_LOGI(INIT_LOG_TAG, "Phase %c configured!", 'A'+phase);

    init_ok = true;
    return true;
}

bool phases::init_phases(void) {
    esp_err_t error_code = ESP_OK;
    gpio_config_t enable_pin_config = {
        .pin_bit_mask = 1<<POWER_ON_PIN,
        .mode         = gpio_mode_t::GPIO_MODE_OUTPUT,
        .pull_up_en   = gpio_pullup_t::GPIO_PULLUP_DISABLE,
        .pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE,
        .intr_type    = gpio_int_type_t::GPIO_INTR_DISABLE
    };
    error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&enable_pin_config));
    gpio_set_level(POWER_ON_GPIO, GPIO_LOW);

    ESP_LOGI(INIT_LOG_TAG, "Creating sine sampler...");

    esp_timer_create_args_t sine_generator_timer_cfg {
        .callback              = phase_output_intr,
        .arg                   = (void*)nullptr,
        .dispatch_method       = ESP_TIMER_TASK, // <-- ARREGLO 1 (Línea 289)
        .name                  = "SINE GEN",
        .skip_unhandled_events = false
    };
    error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_create(
        &sine_generator_timer_cfg,
        &sine_generator_timer_handle
    )); if (error_code != ESP_OK) return false;
    ESP_LOGI(INIT_LOG_TAG, "Sine sampler created!");

    MAX_ANGULAR_SPEED_rads = (M_TAU/SINE_WAVE_SAMPLE_TIMEs)*0.9;
    MAX_FREQUENCY_hz       = 0.9/SINE_WAVE_SAMPLE_TIMEs;
    MAX_ANGULAR_SPEED_int  = w_to_delta_theta_int(MAX_ANGULAR_SPEED_rads);
    ESP_LOGI(INIT_LOG_TAG, "Maximum angular speed: %.3erad/s", MAX_ANGULAR_SPEED_rads);
    ESP_LOGI(INIT_LOG_TAG, "Maximum frequency    : %.3eHz", MAX_FREQUENCY_hz);
    ESP_LOGI(INIT_LOG_TAG, "Configured dead time(us)   : %.3f", DEAD_TIME_nsX100/10.0);
    ESP_LOGI(INIT_LOG_TAG, "Configured dead time(pwmdc): %ld", DEAD_TIME);
    
    ESP_LOGI(INIT_LOG_TAG, "Configuring PWM timer...");
    ledc_timer_config_t pwm_timer_config = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION,
        .timer_num       = PWM_TIMER_ID,
        .freq_hz         = PWM_FREQUENCY_Hz,
        .clk_cfg         = LEDC_USE_APB_CLK,
        .deconfigure     = false
    };
    uint32_t suitable_res = ledc_find_suitable_duty_resolution(APB_CLK_FREQ, PWM_FREQUENCY_Hz);
    error_code = ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_timer_config(&pwm_timer_config));
    if (suitable_res > PWM_RESOLUTION) {
        ESP_LOGW(INIT_LOG_TAG, "You can increase the resolution to %ld", suitable_res);
    }
    if (error_code != ESP_OK) {
        ESP_LOGW(INIT_LOG_TAG, "With a frequency of %ldHz, a resolution of %ld is needed", PWM_FREQUENCY_Hz, suitable_res);
        return false;
    }
    ESP_LOGI(INIT_LOG_TAG, "PWM timer configured!");

    ledc_channel_config_t channel_base_config = {
        .gpio_num   = 0,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = (ledc_channel_t)0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = PWM_TIMER_ID,
        .duty       = 0x0F,
        .hpoint     = 0,
        .flags = {.output_invert = 0}
    };

    ESP_LOGI(INIT_LOG_TAG, "Configuring phase channels...");

    bool init_ok = false;
    init_ok = init_phase_channel(A, channel_base_config);
    if (!init_ok) return false;
    init_ok = init_phase_channel(B, channel_base_config);
    if (!init_ok) return false;
    init_ok = init_phase_channel(C, channel_base_config);
    if (!init_ok) return false;


    // --- ARREGLO 2 (Línea ~368) ---
    // La función ledc_set_duty_and_update() requiere que el 
    // servicio "fade" esté instalado, incluso si no lo usamos para desvanecer.
    esp_err_t fade_err = ledc_fade_func_install(0);
    if (fade_err != ESP_OK) {
        ESP_LOGE(INIT_LOG_TAG, "Error installing fade service: %s", esp_err_to_name(fade_err));
        return false;
    }
    // --- FIN DEL ARREGLO ---


    return true;
}

#endif // MCK_PHASE_MODULE

uint32_t phases::hz_to_delta_theta_int(float frequency_hz) {
    using phases::SINE_WAVE_SAMPLE_TIMEs;
    using phases::MAX_THETA_INT;

    return std::ceil(frequency_hz*SINE_WAVE_SAMPLE_TIMEs*MAX_THETA_INT);
}
uint32_t phases::w_to_delta_theta_int(float angular_speed_rads) {
    using phases::SINE_WAVE_SAMPLE_TIMEs;
    using phases::MAX_THETA_INT;
    using phases::M_TAU;

    return std::ceil(angular_speed_rads*SINE_WAVE_SAMPLE_TIMEs*MAX_THETA_INT/M_TAU);
}
uint32_t phases::rad_to_theta_int(float x) {
    using phases::M_TAU;
    using phases::MAX_THETA_INT;

    while (x > M_TAU) x -= M_TAU;
    while (x <  0.0f) x += M_TAU;
    return (uint32_t)(x*MAX_THETA_INT/M_TAU);
}
/** main/app_main.cpp
 * PRODUCTION VERSION - PID Control via UART
 * - Frequency control (CH1)
 * - Amplitude control (CH2)
 * - RPM Reference (CH3)
 */

#include <stdio.h>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/gpio_struct.h"  // For direct GPIO register access (GPIO.in)
#include "phases.hpp"
#include "pwm.h"

// ========== MODE SELECTION ==========
// Set to 1 for Python debug mode (text commands: "SET CH1 20")
// Set to 0 for Simulink mode (binary: [rpm_ref, freq] as uint16)
#define PYTHON_DEBUG 1

#define TAG "MOTOR_AC_UART"

// Encoder pins
#define PIN_ENC_A GPIO_NUM_13
#define PIN_ENC_B GPIO_NUM_15

// UART
#define UART_PORT UART_NUM_0
#define UART_BAUD 115200

// Encoder constants
#define PULSES_PER_REV 199
#define SAMPLE_MS 10
#define ALPHA 0.1f

// Sequence pulse interval
#define T_PULSE_MS 250
#define MAX_SEQ_LEN 16

typedef struct
{
    uint8_t len;
    uint32_t vals[MAX_SEQ_LEN];
} seq_t;

// Globals
static volatile int32_t g_pulse_count = 0;
static volatile uint32_t g_isr_count = 0;  // Debug: count ISR calls
static volatile uint32_t g_last_isr_us = 0;  // For debouncing
static volatile uint8_t old_AB = 0;
static const int8_t QEM[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
static volatile uint16_t g_rpm_ref = 0;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

static pwm_task_config_t pwmA;
static pwm_task_config_t pwmB;
static pwm_task_config_t pwmC;

// Task handles
static TaskHandle_t pwmA_task_handle = NULL;
static TaskHandle_t pwmB_task_handle = NULL;
static TaskHandle_t pwmC_task_handle = NULL;

// Command queues
static QueueHandle_t cmdQueueA = NULL;
static QueueHandle_t cmdQueueB = NULL;
static QueueHandle_t cmdQueueC = NULL;

// ========== PHASE SHIFT CONSTANTS ==========
constexpr uint32_t PLS_M_TAU_3_INT = phases::MAX_THETA_INT / 3u; // +120°
constexpr uint32_t MNS_M_TAU_3_INT = (~PLS_M_TAU_3_INT) + 1u;    // +240°

// ========== Encoder ISR (Arduino-style: single interrupt on PIN_A) ==========
static void IRAM_ATTR encoder_isr(void *arg)
{
    // Debounce: ignore interrupts too close together (3us minimum)
    const uint32_t DEBOUNCE_US = 3;
    uint32_t now_us = esp_timer_get_time();
    
    if ((now_us - g_last_isr_us) < DEBOUNCE_US) {
        return;  // Too soon, ignore
    }
    g_last_isr_us = now_us;
    
    portENTER_CRITICAL_ISR(&spinlock);
    
    g_isr_count++;  // Debug: count ISR calls
    
    // Fast GPIO read - read PIN_B state
    uint32_t gpio_in = GPIO.in;
    uint8_t state_B = (gpio_in >> PIN_ENC_B) & 1;
    
    // Increment or decrement based on PIN_B state
    if (state_B == 0) {
        g_pulse_count++;
    } else {
        g_pulse_count--;
    }
    
    portEXIT_CRITICAL_ISR(&spinlock);
}

// ========== Notification from ISR ==========
void phases_notify_pwm_tasks_from_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (pwmA_task_handle)
    {
        vTaskNotifyGiveFromISR(pwmA_task_handle, &xHigherPriorityTaskWoken);
    }
    if (pwmB_task_handle)
    {
        vTaskNotifyGiveFromISR(pwmB_task_handle, &xHigherPriorityTaskWoken);
    }
    if (pwmC_task_handle)
    {
        vTaskNotifyGiveFromISR(pwmC_task_handle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ========== UART Init ==========
static void uart_init(void)
{
    uart_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.baud_rate = UART_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_param_config(UART_PORT, &cfg);
    uart_driver_install(UART_PORT, 1024, 1024, 0, NULL, 0);
}

// ========== Encoder Init ==========
static void encoder_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_ENC_A) | (1ULL << PIN_ENC_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE};
    
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Encoder GPIO config failed: %s", esp_err_to_name(err));
        return;
    }
    
    old_AB = ((gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B));
    
    // Install ISR service if not already installed
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(err));
        return;
    }
    
    // Add ISR handlers
    err = gpio_isr_handler_add(PIN_ENC_A, encoder_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for PIN_A: %s", esp_err_to_name(err));
        return;
    }
    
    err = gpio_isr_handler_add(PIN_ENC_B, encoder_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for PIN_B: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Encoder initialized: PIN_A=%d, PIN_B=%d, initial_state=%d", 
             PIN_ENC_A, PIN_ENC_B, old_AB);
}

// ========== PWM TASK ==========
static void pwm_task(void *pv)
{
    pwm_task_config_t *cfg = (pwm_task_config_t *)pv;

    // Determine complementary pin
    gpio_num_t op_a = cfg->gpio_num;
    gpio_num_t op_b = phases::A_LOW_GPIO;
    if (cfg->pwm_timer == phases::PWM_TIMER_ID0)
        op_b = phases::A_LOW_GPIO;
    else if (cfg->pwm_timer == phases::PWM_TIMER_ID1)
        op_b = phases::B_LOW_GPIO;
    else
        op_b = phases::C_LOW_GPIO;

    pwm_hal_configure_timer_and_pin(cfg, op_a, op_b);
    pwm_hal_set_deadtime(cfg->pwm_unit, cfg->pwm_timer, 500, 500);

    seq_t local_seq;
    local_seq.len = 0;
    QueueHandle_t my_cmd_queue = NULL;

    if (cfg->pwm_timer == phases::PWM_TIMER_ID0)
        my_cmd_queue = cmdQueueA;
    else if (cfg->pwm_timer == phases::PWM_TIMER_ID1)
        my_cmd_queue = cmdQueueB;
    else if (cfg->pwm_timer == phases::PWM_TIMER_ID2)
        my_cmd_queue = cmdQueueC;

    // Calculate phase shift
    uint32_t shift_int = 0;
    if (cfg->phase_offset_deg == 120.0f)
    {
        shift_int = PLS_M_TAU_3_INT;
    }
    else if (cfg->phase_offset_deg == 240.0f)
    {
        shift_int = MNS_M_TAU_3_INT;
    }

    float duty_cmd = 0.0f;

    for (;;)
    {
        // 1) Check for command sequence
        if (my_cmd_queue != NULL && xQueueReceive(my_cmd_queue, &local_seq, 0) == pdTRUE)
        {

            for (uint8_t i = 0; i < local_seq.len; ++i)
            {
                uint32_t value = local_seq.vals[i];

                if (cfg->pwm_timer == phases::PWM_TIMER_ID0)
                {
                    // CH1 = Frequency
                    float freq_hz = (float)value;
                    phases::set_frequency(freq_hz);
                    ESP_LOGI(TAG, "Frequency: %.2f Hz", freq_hz);
                }
                else if (cfg->pwm_timer == phases::PWM_TIMER_ID1)
                {
                    // CH2 = Amplitude
                    float amp = (float)value / 100.0f;
                    if (amp > 1.0f)
                        amp = 1.0f;
                    if (amp < 0.0f)
                        amp = 0.0f;
                    phases::set_amplitude(amp);
                    ESP_LOGI(TAG, "Amplitude: %.0f%%", amp * 100.0f);
                }
                else
                {
                    // CH3 = RPM Reference
                    g_rpm_ref = (uint16_t)value;
                    ESP_LOGI(TAG, "RPM Ref: %u", (unsigned)g_rpm_ref);
                }

                // Don't delay here - continue PWM generation immediately
                // to maintain continuous sinusoidal waveform
            }
            local_seq.len = 0;
        }

        // 2) Check legacy float queue
        if (cfg->queue && xQueueReceive(cfg->queue, &duty_cmd, 0) == pdTRUE)
        {
            if (duty_cmd < 0.0f)
                duty_cmd = 0.0f;
            if (duty_cmd > 100.0f)
                duty_cmd = 100.0f;
            cfg->duty_cycle = duty_cmd;
            pwm_hal_set_duty_percent(cfg, duty_cmd);

            while (cfg->queue && xQueueReceive(cfg->queue, &duty_cmd, 0) == pdTRUE)
            {
                if (duty_cmd < 0.0f)
                    duty_cmd = 0.0f;
                if (duty_cmd > 100.0f)
                    duty_cmd = 100.0f;
                cfg->duty_cycle = duty_cmd;
                pwm_hal_set_duty_percent(cfg, duty_cmd);
            }
            vTaskDelay(pdMS_TO_TICKS(T_PULSE_MS));
            continue;
        }

        // 3) SPWM mode
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t theta = phases::phases_get_theta_int();
        uint32_t theta_shifted = theta + shift_int;

        uint32_t raw = sin_lut(theta_shifted);
        const uint32_t DUTYCYCLE_OFFSET = 16;
        const uint32_t DUTYCYCLE_MASK_HIGH = 0xFFFF << DUTYCYCLE_OFFSET;
        uint32_t duty_counts_high = (raw & DUTYCYCLE_MASK_HIGH) >> DUTYCYCLE_OFFSET;

        float pct = ((float)duty_counts_high / (float)PWM_MAX_VAL) * 100.0f;
        pct *= phases::get_amplitude();

        if (pct < 0.0f)
            pct = 0.0f;
        if (pct > 100.0f)
            pct = 100.0f;

        cfg->duty_cycle = pct;
        pwm_hal_set_duty_percent(cfg, pct);
    }
}

// ========== UART RX Task ==========
static void uart_rx_task(void *arg)
{
    const int RX_BUF = 512;
    uint8_t *data = (uint8_t *)malloc(RX_BUF);
    std::string line_buffer;

    auto trim = [](std::string &str)
    {
        while (!str.empty() && isspace((unsigned char)str.front()))
            str.erase(0, 1);
        while (!str.empty() && isspace((unsigned char)str.back()))
            str.pop_back();
    };

    while (1)
    {
        int len = uart_read_bytes(UART_PORT, data, RX_BUF - 1, pdMS_TO_TICKS(200));
        if (len > 0)
        {
            data[len] = '\0';
            line_buffer += (char *)data;

            size_t newline_pos;
            while ((newline_pos = line_buffer.find('\n')) != std::string::npos)
            {
                std::string cmd_line = line_buffer.substr(0, newline_pos);
                line_buffer.erase(0, newline_pos + 1);

                trim(cmd_line);
                if (cmd_line.empty())
                    continue;

                size_t space1 = cmd_line.find(' ');
                if (space1 == std::string::npos)
                    continue;
                std::string token1 = cmd_line.substr(0, space1);

                size_t space2 = cmd_line.find(' ', space1 + 1);
                if (space2 == std::string::npos)
                    continue;
                std::string token2 = cmd_line.substr(space1 + 1, space2 - space1 - 1);
                std::string token3 = cmd_line.substr(space2 + 1);

                trim(token1);
                trim(token2);
                trim(token3);

                if (token1 != "SET")
                    continue;

                QueueHandle_t target_cmd_queue = NULL;
                if (token2 == "CH1" || token2 == "A")
                    target_cmd_queue = cmdQueueA;
                else if (token2 == "CH2" || token2 == "B")
                    target_cmd_queue = cmdQueueB;
                else if (token2 == "CH3" || token2 == "C")
                    target_cmd_queue = cmdQueueC;

                if (target_cmd_queue == NULL)
                    continue;

                seq_t seq;
                seq.len = 0;

                size_t start = 0;
                while (start < token3.size() && seq.len < MAX_SEQ_LEN)
                {
                    size_t end = token3.find(',', start);
                    if (end == std::string::npos)
                        end = token3.size();

                    std::string val_str = token3.substr(start, end - start);
                    trim(val_str);

                    if (!val_str.empty())
                    {
                        uint32_t value = strtoul(val_str.c_str(), NULL, 10);
                        seq.vals[seq.len++] = value;
                    }
                    start = end + 1;
                }

                if (seq.len > 0)
                {
                    xQueueOverwrite(target_cmd_queue, &seq);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ========== SIMULINK RX Task (Binary Protocol) ==========
#if PYTHON_DEBUG == 0
static void simulink_rx_task(void *arg)
{
    const int RX_BUF = 128;
    uint8_t *data = (uint8_t *)malloc(RX_BUF);
    uint8_t buffer[4]; // Buffer for 2 x uint16 = 4 bytes
    uint8_t buf_idx = 0;

    ESP_LOGI(TAG, "Simulink RX task started - waiting for [rpm_ref, freq] as uint16");

    while (1)
    {
        int len = uart_read_bytes(UART_PORT, data, RX_BUF, pdMS_TO_TICKS(20));
        
        for (int i = 0; i < len; i++)
        {
            buffer[buf_idx++] = data[i];
            
            // When we have 4 bytes (2 x uint16), process them
            if (buf_idx >= 4)
            {
                // Parse: [rpm_ref_u16, freq_u16]
                uint16_t rpm_ref_rx = (uint16_t)(buffer[0] | (buffer[1] << 8));
                uint16_t freq_rx = (uint16_t)(buffer[2] | (buffer[3] << 8));
                
                // Update RPM reference
                g_rpm_ref = rpm_ref_rx;
                
                // Update frequency (already limited to 20-60 Hz in set_frequency)
                float freq_hz = (float)freq_rx;
                phases::set_frequency(freq_hz);
                
                ESP_LOGI(TAG, "Simulink RX: RPM_ref=%u, Freq=%.1f Hz", rpm_ref_rx, freq_hz);
                
                buf_idx = 0; // Reset buffer
            }
        }
    }
    
    free(data);
}
#endif

// ========== RPM TX Task ==========
static void rpm_tx_task(void *arg)
{
    const double Ts = SAMPLE_MS / 1000.0;
    double rpm_filt = 0.0;
    const double RPM_MAX = 10000.0;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));

        int32_t pulses;
        uint32_t isr_calls;
        taskENTER_CRITICAL(&spinlock);
        pulses = g_pulse_count;
        isr_calls = g_isr_count;
        g_pulse_count = 0;
        g_isr_count = 0;
        taskEXIT_CRITICAL(&spinlock);

        double cycles = (double)pulses / 8.0;
        double revolutions = cycles / (double)PULSES_PER_REV;
        double rpm = fabs((revolutions / Ts) * 60.0);
        if (rpm > RPM_MAX)
            rpm = RPM_MAX;

        rpm_filt = (ALPHA * rpm) + ((1.0 - ALPHA) * rpm_filt);
        uint16_t rpm_measured = (uint16_t)rpm_filt;
        uint16_t rpm_ref_snapshot = g_rpm_ref;

        // Send binary data: [rpm_measured, rpm_ref] as uint16
        uart_write_bytes(UART_PORT, (const char *)&rpm_measured, sizeof(uint16_t));
        uart_write_bytes(UART_PORT, (const char *)&rpm_ref_snapshot, sizeof(uint16_t));
        
        // Log RPM data for monitoring with debug info
        ESP_LOGI(TAG, "Encoder: ISR_calls=%u, pulses=%d, rpm_raw=%.1f, rpm_filt=%.1f, RPM_meas=%u, RPM_ref=%u", 
                 (unsigned)isr_calls, (int)pulses, rpm, rpm_filt, rpm_measured, rpm_ref_snapshot);
    }
}

static void configure_enable_and_unused_pins(void)
{
    gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_23, 1);
    ESP_LOGI(TAG, "ENABLE pin (23) set HIGH");
    ESP_LOGI(TAG, "GPIO configuration complete");
}

// ========== app_main ==========
extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    configure_enable_and_unused_pins();

    uart_init();
    encoder_init();

    ESP_LOGI(TAG, "AC Motor Controller - Ready");

    if (!phases::init_phases())
    {
        ESP_LOGE(TAG, "Initialization failed");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    pwm_hal_init();

    // Configure PWM tasks
    pwmA.pwm_unit = phases::MCPWM_UNIT_USED;
    pwmA.pwm_timer = phases::PWM_TIMER_ID0;
    pwmA.pwm_op = MCPWM_OPR_A;
    pwmA.gpio_num = phases::A_HIGH_GPIO;
    pwmA.phase_offset_deg = 0.0f;
    pwmA.name_id = "PhaseA";
    pwmA.queue = NULL;

    pwmB.pwm_unit = phases::MCPWM_UNIT_USED;
    pwmB.pwm_timer = phases::PWM_TIMER_ID1;
    pwmB.pwm_op = MCPWM_OPR_A;
    pwmB.gpio_num = phases::B_HIGH_GPIO;
    pwmB.phase_offset_deg = 120.0f;
    pwmB.name_id = "PhaseB";
    pwmB.queue = NULL;

    pwmC.pwm_unit = phases::MCPWM_UNIT_USED;
    pwmC.pwm_timer = phases::PWM_TIMER_ID2;
    pwmC.pwm_op = MCPWM_OPR_A;
    pwmC.gpio_num = phases::C_HIGH_GPIO;
    pwmC.phase_offset_deg = 240.0f;
    pwmC.name_id = "PhaseC";
    pwmC.queue = NULL;

    cmdQueueA = xQueueCreate(1, sizeof(seq_t));
    cmdQueueB = xQueueCreate(1, sizeof(seq_t));
    cmdQueueC = xQueueCreate(1, sizeof(seq_t));

    if (!cmdQueueA || !cmdQueueB || !cmdQueueC)
    {
        ESP_LOGE(TAG, "Queue creation failed");
        while (1)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Initialize motor
    phases::set_amplitude(1.0f);
    phases::set_frequency(20.0f);
    
    // Configure slew rate for smooth frequency transitions
    //   - Lower (10-20 Hz/s): Smoother, better for plant modeling
    //   - Higher (50-100 Hz/s): Faster response, more dynamic
    phases::set_frequency_slew_rate(30.0f);

    // Create tasks
#if PYTHON_DEBUG == 1
    // Python debug mode: text commands "SET CH1 20"
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "Python debug mode: Waiting for text commands (CH1=Freq, CH2=Amp, CH3=RPM)");
#else
    // Simulink mode: binary protocol [rpm_ref, freq] as uint16
    xTaskCreatePinnedToCore(simulink_rx_task, "simulink_rx", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "Simulink mode: Waiting for binary data [rpm_ref, freq]");
#endif
    
    xTaskCreatePinnedToCore(rpm_tx_task, "rpm_tx", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(pwm_task, "pwmC", 4096, &pwmC, 5, &pwmC_task_handle, 0);
    xTaskCreatePinnedToCore(pwm_task, "pwmA", 4096, &pwmA, 5, &pwmA_task_handle, 1);
    xTaskCreatePinnedToCore(pwm_task, "pwmB", 4096, &pwmB, 5, &pwmB_task_handle, 1);

    // Start timer
    phases::start_phases();
}
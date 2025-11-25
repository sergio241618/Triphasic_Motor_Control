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
#define PYTHON_DEBUG 0

// Set to 1 for plant modeling mode (logs: Time, Frequency, RPM)
// Set to 0 for normal operation
#define PLANT_MODELING 0

// Set to 1 to generate internal ramp (no Simulink needed)
// Set to 0 to receive frequency from Simulink
#define INTERNAL_RAMP 0

#define TAG "MOTOR_AC_UART" 

// Encoder pins
#define PIN_ENC_A GPIO_NUM_16
#define PIN_ENC_B GPIO_NUM_17

// UART
#define UART_PORT UART_NUM_0
#define UART_BAUD 115200

// Encoder constants (4x decoding - QEM method)
#define PPR 199.0f                    // Pulses per revolution

#if PLANT_MODELING == 1
    #define SAMPLE_MS 100             // Sample period in ms (plant modeling: 100ms for cleaner data)
#else
    #define SAMPLE_MS 10              // Sample period in ms (normal operation: 10ms)
#endif

#define ALPHA 0.1f                    // Low-pass filter coefficient (EMA)
#define CYCLE_ADJUSTMENT 8.0f         // 4x decoding adjustment factor
#define CONVERSION_TO_RPM 60000.0f    // Convert rev/ms to RPM

// Internal ramp parameters (for plant modeling)
#if INTERNAL_RAMP == 1
    #define RAMP_START_HZ 20.0f       // Starting frequency (Hz)
    #define RAMP_END_HZ 50.0f         // Ending frequency (Hz)
    #define RAMP_DURATION_S 60.0f     // Ramp duration (seconds)
    #define RAMP_SLOPE ((RAMP_END_HZ - RAMP_START_HZ) / RAMP_DURATION_S)  // Hz/s
#endif

// Sequence pulse interval
#define T_PULSE_MS 250
#define MAX_SEQ_LEN 16

typedef struct
{
    uint8_t len;
    uint32_t vals[MAX_SEQ_LEN];
} seq_t;

// Globals
static volatile long g_pulse_count = 0;
static volatile uint8_t g_old_AB = 0;  // Encoder state for QEM
static volatile int16_t g_rpm_ref = 0;
static volatile float g_current_freq = 0.0f;  // Current frequency (Hz) for plant modeling
static float g_filtered_rpm = 0.0f;    // EMA filter state
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// QEM (Quadrature Encoder Method) lookup table for 4x decoding
static const int8_t QEM[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

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

// ========== Encoder ISR (4x Decoding - QEM Method) ==========
static void IRAM_ATTR encoder_isr(void *arg)
{
    // Read both encoder pins directly from GPIO register
    uint32_t gpio_in = GPIO.in;
    
    // Shift old state and read new state
    g_old_AB <<= 2;
    uint8_t state_A = (gpio_in >> PIN_ENC_A) & 1;
    uint8_t state_B = (gpio_in >> PIN_ENC_B) & 1;
    uint8_t new_AB = (state_A << 1) | state_B;
    g_old_AB |= new_AB;
    
    // Use QEM lookup table to determine direction and count
    g_pulse_count += QEM[g_old_AB & 0x0f];
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

// ========== Encoder Init (4x Decoding - QEM Method) ==========
static void encoder_init(void)
{
    // Reset pins to ensure clean state
    gpio_reset_pin(PIN_ENC_A);
    gpio_reset_pin(PIN_ENC_B);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure BOTH pins with interrupts on ANY edge (4x decoding)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_ENC_A) | (1ULL << PIN_ENC_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE  // Interrupt on BOTH edges
    };
    
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Encoder GPIO config failed: %s", esp_err_to_name(err));
        return;
    }
    
    // Read initial state of encoder
    g_old_AB = ((gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B));
    
    // Install ISR service if not already installed
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(err));
        return;
    }
    
    // Add ISR handlers for BOTH pins (critical for 4x decoding)
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
    
    ESP_LOGI(TAG, "Encoder initialized (4x decoding): PIN_A=%d, PIN_B=%d", 
             PIN_ENC_A, PIN_ENC_B);
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
                    g_rpm_ref = (int16_t)value;
                    ESP_LOGI(TAG, "RPM Ref: %d", (int)g_rpm_ref);
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

    ESP_LOGI(TAG, "Simulink RX task started - waiting for [rpm_ref, freq] as int16");

    while (1)
    {
        int len = uart_read_bytes(UART_PORT, data, RX_BUF, pdMS_TO_TICKS(20));
        
        for (int i = 0; i < len; i++)
        {
            buffer[buf_idx++] = data[i];
            
            // When we have 4 bytes (2 x uint16), process them
            if (buf_idx >= 4)
            {
                // Parse: [rpm_ref_i16, freq_i16]
                int16_t rpm_ref_rx = (int16_t)(buffer[0] | (buffer[1] << 8));
                int16_t freq_rx = (int16_t)(buffer[2] | (buffer[3] << 8));
                
                // Update RPM reference (ignored in plant modeling mode)
                g_rpm_ref = rpm_ref_rx;
                
                // Update frequency (already limited to 20-60 Hz in set_frequency)
                float freq_hz = (float)freq_rx;
                g_current_freq = freq_hz;  // Store for plant modeling
                phases::set_frequency(freq_hz);
                
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
    const double RPM_MAX = 10000.0;
    uint32_t time_ms = 0;  // Simulation time counter

    #if PLANT_MODELING == 1
        // Print CSV header for plant modeling
        printf("Time(s),Frequency(Hz),RPM\n");
        fflush(stdout);  // Force immediate output
    #endif

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));

        // Get pulse count (critical section to avoid race condition)
        long pulses;
        portDISABLE_INTERRUPTS();
        pulses = g_pulse_count;
        g_pulse_count = 0;
        portENABLE_INTERRUPTS();

        // RPM calculation (4x decoding - QEM method)
        float cycles = (float)pulses / CYCLE_ADJUSTMENT;
        float revolutions = cycles / PPR;
        float raw_rpm = (revolutions / (float)SAMPLE_MS) * CONVERSION_TO_RPM;

        // Absolute value and clamp
        if (raw_rpm < 0) raw_rpm = -raw_rpm;
        if (raw_rpm > RPM_MAX) raw_rpm = RPM_MAX;

        // Apply EMA (Exponential Moving Average) filter
        g_filtered_rpm = (ALPHA * raw_rpm) + ((1.0f - ALPHA) * g_filtered_rpm);

        int16_t rpm_measured = (int16_t)g_filtered_rpm;
        int16_t rpm_ref_snapshot = g_rpm_ref;
        
        #if PLANT_MODELING == 1
            // Plant modeling mode: Print CSV format (Time, Frequency, RPM)
            float time_s = time_ms / 1000.0f;
            float freq_snapshot = g_current_freq;
            printf("%.2f,%.2f,%.2f\n", time_s, freq_snapshot, g_filtered_rpm);
            fflush(stdout);  // Force immediate output
        #else
            // Normal operation: Send binary int16 data [rpm_ref, rpm_measured] to Simulink
            uart_write_bytes(UART_PORT, (const char *)&rpm_ref_snapshot, sizeof(int16_t));
            uart_write_bytes(UART_PORT, (const char *)&rpm_measured, sizeof(int16_t));
        #endif
        
        time_ms += SAMPLE_MS;
    }
}

// ========== Internal Ramp Generator Task (for Plant Modeling) ==========
#if INTERNAL_RAMP == 1
static void ramp_generator_task(void *arg)
{
    ESP_LOGI(TAG, "Internal ramp generator ready");
    ESP_LOGI(TAG, "Ramp config: %.1f Hz -> %.1f Hz over %.1f seconds", 
             RAMP_START_HZ, RAMP_END_HZ, RAMP_DURATION_S);
    
    // Wait 3 seconds after boot to ensure system is stable
    ESP_LOGI(TAG, "Waiting 3 seconds before starting ramp...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "Starting ramp NOW!");
    
    float current_freq = RAMP_START_HZ;
    uint32_t time_ms = 0;
    const uint32_t UPDATE_PERIOD_MS = 100;  // Update frequency every 100ms
    
    // Set initial frequency
    g_current_freq = current_freq;
    phases::set_frequency(current_freq);
    
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));
        time_ms += UPDATE_PERIOD_MS;
        
        // Calculate current frequency based on ramp
        float time_s = time_ms / 1000.0f;
        current_freq = RAMP_START_HZ + (RAMP_SLOPE * time_s);
        
        // Clamp to end frequency
        if (current_freq > RAMP_END_HZ) {
            current_freq = RAMP_END_HZ;
        }
        
        // Update frequency
        g_current_freq = current_freq;
        phases::set_frequency(current_freq);
        
        // Stop after ramp duration + 10 seconds (to capture steady state)
        if (time_s > (RAMP_DURATION_S + 10.0f)) {
            ESP_LOGI(TAG, "Ramp completed. Holding at %.1f Hz", current_freq);
            // Keep running at final frequency
            while(1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
}
#endif

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
    phases::set_frequency(1.0f);

    // Create tasks
#if PYTHON_DEBUG == 1
    // Python debug mode: text commands "SET CH1 20"
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "Python debug mode: Waiting for text commands (CH1=Freq, CH2=Amp, CH3=RPM)");
#elif INTERNAL_RAMP == 1
    // Internal ramp mode: Generate ramp internally (no Simulink needed)
    xTaskCreatePinnedToCore(ramp_generator_task, "ramp_gen", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "Internal ramp mode: Generating %.1f Hz -> %.1f Hz over %.1f s", 
             RAMP_START_HZ, RAMP_END_HZ, RAMP_DURATION_S);
#else
    // Simulink mode: binary protocol [rpm_ref, freq] as int16
    xTaskCreatePinnedToCore(simulink_rx_task, "simulink_rx", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "Simulink mode: Waiting for binary data [rpm_ref, freq] as int16");
#endif
    
    xTaskCreatePinnedToCore(rpm_tx_task, "rpm_tx", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(pwm_task, "pwmC", 4096, &pwmC, 5, &pwmC_task_handle, 0);
    xTaskCreatePinnedToCore(pwm_task, "pwmA", 4096, &pwmA, 5, &pwmA_task_handle, 1);
    xTaskCreatePinnedToCore(pwm_task, "pwmB", 4096, &pwmB, 5, &pwmB_task_handle, 1);

    // Start timer
    phases::start_phases();
}
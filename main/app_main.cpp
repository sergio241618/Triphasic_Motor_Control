/**
 * ARCHIVO: main/app_main.cpp
 * * Lógica principal que combina:
 * 1. Controlador de motor AC (drivers/HAL/phases.hpp)
 * 2. Lector de Encoder (para feedback de RPM)
 * 3. Comunicación UART (para recibir comandos y enviar telemetría)
 * * Incluye un interruptor de #define para modo de prueba.
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "soc/gpio_struct.h"
#include "phases.hpp"


#define TAG "MOTOR_AC_UART"

// --- Interruptor de Pruebas ---
// Poner en 1 para activar el MODO PRUEBA (frecuencia fija, para LED)
// Poner en 0 para activar el MODO REAL (control por UART)
#define RUN_TEST_MODE 0


// ================== Pins ==================
#define PIN_ENC_A      GPIO_NUM_14 // Encoder en pines libres
#define PIN_ENC_B      GPIO_NUM_15

// ================== UART ===================
#define UART_PORT      UART_NUM_0
#define UART_BAUD      115200

// ================== Encoder ===================
#define PULSES_PER_REV 199
#define SAMPLE_MS       10
#define ALPHA           0.1f

// ================== Scaling ===================
#define RPM_MAX         10000.0

// ================== Globals ===================
static volatile long g_pulse_count = 0;
static volatile uint8_t old_AB = 0;
static const int8_t QEM[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
static uint16_t g_rpm_ref  = 0;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// ================== Encoder ISR ===================
static void IRAM_ATTR encoder_isr(void *arg)
{
    uint32_t gpio_in = GPIO.in;
    old_AB <<= 2;
    uint8_t state_A = (gpio_in >> PIN_ENC_A) & 1;
    uint8_t state_B = (gpio_in >> PIN_ENC_B) & 1;
    uint8_t new_AB = (state_A << 1) | state_B;
    old_AB |= new_AB;
    g_pulse_count += QEM[old_AB & 0x0F];
}

// ================== UART INIT ===================
static void uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT, &cfg);
    uart_driver_install(UART_PORT, 256, 256, 0, NULL, 0);
}

// ================== Encoder INIT ===================
static void encoder_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_ENC_A) | (1ULL << PIN_ENC_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);
    old_AB = ((gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ENC_A, encoder_isr, NULL);
    gpio_isr_handler_add(PIN_ENC_B, encoder_isr, NULL);
}

// ================== UART RX Task ===================
// Espera [rpm_ref, target_frequency_hz]
static void uart_rx_task(void *arg)
{
    uint16_t rx_buf[2];  // rx_buf[0]=rpm_ref, rx_buf[1]=frecuencia_hz
    while (1) {
        int len = uart_read_bytes(UART_PORT, (uint8_t *)rx_buf,
                                  sizeof(rx_buf), pdMS_TO_TICKS(20));
        if (len == sizeof(rx_buf)) {
            g_rpm_ref = rx_buf[0];
            uint16_t target_freq_hz = rx_buf[1];
            
            // Aplicamos la frecuencia al driver del motor AC
            phases::set_frequency((float)target_freq_hz);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ================== RPM TX Task ===================
static void rpm_tx_task(void *arg)
{
    const double Ts = SAMPLE_MS / 1000.0;
    double rpm_filt = 0.0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));

        long pulses;
        taskENTER_CRITICAL(&spinlock);
        pulses = g_pulse_count;
        g_pulse_count = 0;
        taskEXIT_CRITICAL(&spinlock);

        double cycles = (double)pulses / 8.0;
        double revolutions = cycles / (double)PULSES_PER_REV;
        double rpm = fabs((revolutions / Ts) * 60.0);

        if (rpm > RPM_MAX) rpm = RPM_MAX;

        rpm_filt = (ALPHA * rpm) + ((1.0 - ALPHA) * rpm_filt);

        uint16_t rpm_val = (uint16_t)rpm_filt;
        uart_write_bytes(UART_PORT, (const char *)&rpm_val, sizeof(uint16_t));
        uart_write_bytes(UART_PORT, (const char *)&g_rpm_ref, sizeof(uint16_t));

        // --- ¡AQUÍ LA PRUEBA DE SOFTWARE! ---
        // Imprime la frecuencia que el módulo 'phases' cree que está generando
        ESP_LOGI(TAG, "RPM(tx): %u, Freq(leida): %.2f Hz", rpm_val, phases::get_frequency());
    }
}

// ================== MAIN ===================
extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    
    // --- INICIALIZACIÓN COMÚN ---
    uart_init();
    encoder_init();

    ESP_LOGI(TAG, "Inicializando controlador de motor AC (Phases)...");
    if (phases::init_phases()) {
        ESP_LOGI(TAG, "Controlador AC inicializado.");
        
        phases::set_amplitude(1.0f); // Amplitud al 100%
        phases::start_phases();      // Arranca los timers de PWM
        
    } else {
        ESP_LOGE(TAG, "¡¡¡ ERROR FATAL al inicializar 'phases' !!!");
        while(1) { vTaskDelay(1000); } // Detener
    }

// --- AQUÍ SE SELECCIONA EL MODO DE OPERACIÓN ---
#if (RUN_TEST_MODE == 1)

    // --- MODO DE PRUEBA ---
    ESP_LOGW(TAG, "======== MODO DE PRUEBA ACTIVO ========");
    ESP_LOGW(TAG, "Fijando frecuencia a 1.0 Hz para prueba de LED.");
    ESP_LOGW(TAG, "Conecta un LED + Resistor al GPIO 5 (A_HIGH)");
    phases::set_frequency(0.2f);
    // No se inician las tareas de UART ni RPM, el sistema espera aquí.

#else

    // --- MODO REAL (UART) ---
    ESP_LOGI(TAG, "======== MODO REAL (UART) ACTIVO ========");
    ESP_LOGI(TAG, "Iniciando tareas de control por UART y telemetría de RPM.");
    
    // Arrancamos con motor detenido hasta recibir comando
    phases::set_frequency(0.0f); 
    
    // Creamos las tareas de control
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(rpm_tx_task, "rpm_tx_task", 4096, NULL, 5, NULL, 1);

#endif
}
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#define SERVO_PIN 2
#define SERVO_FREQ 50 // Servo PWM frequency
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER LEDC_TIMER_0

void setServoAngle(int angle) {
    int dutyCycle = (angle * (2500 - 500) / 270) + 500; // Convert angle to duty cycle (500-2500us)
    uint32_t duty = (dutyCycle * 8192) / 20000; // Scale to LEDC resolution (assuming 16-bit)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
}

void app_main() {
    ledc_timer_config_t timerConfig = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = LEDC_TIMER,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timerConfig);

    ledc_channel_config_t channelConfig = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channelConfig);

    while (1) {
        setServoAngle(0); // Move to 0 degrees
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds

        setServoAngle(270); // Move to 270 degrees
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds
    }
}

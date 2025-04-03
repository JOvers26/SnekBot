#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#define SERVO_GPIO 2  // Change to the GPIO pin you are using
#define SERVO_MIN_PULSEWIDTH_US 500   // Minimum pulse width in microseconds (0°)
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microseconds (180°)
#define SERVO_MAX_DEGREE 180          // Maximum degree of rotation

static const char *TAG = "SERVO";

mcpwm_cmpr_handle_t comparator;
mcpwm_timer_handle_t timer;
mcpwm_oper_handle_t operator;

/**
 * @brief Convert an angle in degrees to a pulse width in microseconds
 */
static uint32_t angle_to_pulsewidth(uint32_t angle) {
    return SERVO_MIN_PULSEWIDTH_US + (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / SERVO_MAX_DEGREE);
}

/**
 * @brief Set the servo to a specific angle
 */
static void set_servo_angle(uint32_t angle) {
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    uint32_t pulse_width = angle_to_pulsewidth(angle);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_width));
    ESP_LOGI(TAG, "Servo moved to %d degrees (Pulse Width: %dus)", angle, pulse_width);
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing servo...");

    // Timer configuration
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,  // 1 MHz resolution for microsecond precision
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 20000  // 20ms period (50Hz PWM for servo)
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // Operator configuration
    mcpwm_operator_config_t operator_config = {.group_id = 0};
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator));

    // Connect operator to timer
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator, timer));

    // Comparator configuration
    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez = true};
    ESP_ERROR_CHECK(mcpwm_new_comparator(operator, &comparator_config, &comparator));

    // Generator configuration
    mcpwm_gen_handle_t generator;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_GPIO,
        .flags.invert_pwm = false
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(operator, &generator_config, &generator));

    // Set generator to produce PWM
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    // Start the timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    // Servo movement loop
    while (1) {
        for (int angle = 0; angle <= 180; angle += 5) {
            set_servo_angle(angle);
            vTaskDelay(pdMS_TO_TICKS(100));  // Wait 100ms
        }
        for (int angle = 180; angle >= 0; angle -= 5) {
            set_servo_angle(angle);
            vTaskDelay(pdMS_TO_TICKS(100));  // Wait 100ms
        }
    }
}

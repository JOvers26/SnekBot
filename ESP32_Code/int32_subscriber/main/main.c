#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include <math.h>  // For M_PI constant

#define SERVO_GPIO 2  // Change to the GPIO pin you are using
#define SERVO_MIN_PULSEWIDTH_US 500   // Minimum pulse width (0°)
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width (180°)
#define SERVO_MAX_DEGREE 180          // Maximum degree of rotation
#define PI 3.14159265359               // Define constant PI

mcpwm_cmpr_handle_t comparator;
mcpwm_timer_handle_t timer;
mcpwm_oper_handle_t operator;

// Convert radians to an angle (in degrees)
static uint32_t radians_to_angle(float radians) {
    // Map radians to degrees where -PI -> 0° and PI -> 180°
    float angle = radians * (SERVO_MAX_DEGREE / PI);
    if (angle < 0) angle = 0;       // Ensure the angle stays within 0-180 range
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;  // Clamp to 180 degrees
    return (uint32_t)angle;
}

// Convert an angle (in degrees) to a pulse width in microseconds
static uint32_t angle_to_pulsewidth(uint32_t angle) {
    return SERVO_MIN_PULSEWIDTH_US + (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / SERVO_MAX_DEGREE);
}

// Move servo to a specific angle in degrees
static void set_servo_angle(uint32_t angle) {
    uint32_t pulse_width = angle_to_pulsewidth(angle);
    mcpwm_comparator_set_compare_value(comparator, pulse_width);
}

// Move servo based on radians input
static void set_servo_angle_radians(float radians) {
    uint32_t angle = radians_to_angle(radians);
    set_servo_angle(angle);
}

void app_main(void) {
    // Timer setup
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,  
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 20000  // 50Hz PWM
    };
    mcpwm_new_timer(&timer_config, &timer);

    // Operator setup
    mcpwm_operator_config_t operator_config = {.group_id = 0};
    mcpwm_new_operator(&operator_config, &operator);
    mcpwm_operator_connect_timer(operator, timer);

    // Comparator setup
    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(operator, &comparator_config, &comparator);

    // Generator setup
    mcpwm_gen_handle_t generator;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_GPIO,
        .flags.invert_pwm = false
    };
    mcpwm_new_generator(operator, &generator_config, &generator);

    mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW));

    // Start timer
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    // Servo movement loop for smooth transitions using radians
    while (1) {
        // Move from -PI to PI radians (from 0° to 180°)
        for (float radians = -PI; radians <= PI; radians += 0.05) {  // Increment by small steps for smoothness
            set_servo_angle_radians(radians);
            vTaskDelay(pdMS_TO_TICKS(15));  // Short delay for smoother movement
        }

        // Move back from PI to -PI radians
        for (float radians = PI; radians >= -PI; radians -= 0.05) {
            set_servo_angle_radians(radians);
            vTaskDelay(pdMS_TO_TICKS(15));  // Short delay for smoother movement
        }
    }
}

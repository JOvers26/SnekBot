#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/mcpwm_prelude.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define NUM_JOINTS 7
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); vTaskDelete(NULL);} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);} }

#define JOINT_1 1
#define JOINT_2 2
#define JOINT_3 42
#define JOINT_4 41
#define JOINT_5 40
#define JOINT_6 39
#define JOINT_G 38
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MAX_DEGREE 270
#define PI 3.14159265359

rcl_subscription_t snekbot_joint_state_subscriber;
sensor_msgs__msg__JointState recv_joint_state_msg;

mcpwm_cmpr_handle_t comparator;
mcpwm_timer_handle_t timer;
mcpwm_oper_handle_t operator;

static uint32_t radians_to_angle(float radians) {
    float angle = ((radians + PI) / (2 * PI)) * SERVO_MAX_DEGREE;
    if (angle < 0) angle = 0;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    return (uint32_t)angle;
}

static uint32_t angle_to_pulsewidth(uint32_t angle) {
    return SERVO_MIN_PULSEWIDTH_US + (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / SERVO_MAX_DEGREE);
}

static void set_servo_angle(uint32_t angle) {
    uint32_t pulse_width = angle_to_pulsewidth(angle);
    mcpwm_comparator_set_compare_value(comparator, pulse_width);
}

static void set_servo_angle_radians(float radians) {
    uint32_t angle = radians_to_angle(radians);
    set_servo_angle(angle);
}

static void setup_pwm(void) {
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 20000
    };
    mcpwm_new_timer(&timer_config, &timer);
    
    mcpwm_operator_config_t operator_config = {.group_id = 0};
    mcpwm_new_operator(&operator_config, &operator);
    mcpwm_operator_connect_timer(operator, timer);
    
    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(operator, &comparator_config, &comparator);
    
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
}

void snekbot_joint_state_callback(const void * msgin) {
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
    if (!msg) return;
    
    for (size_t i = 0; i < msg->position.size; i++) {
        uint32_t angle = radians_to_angle(msg->position.data[i]);
        set_servo_angle(angle);
    }
}

void micro_ros_task(void * arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "snekbot_joint_state_subscriber_rclc", "", &support));
    RCCHECK(rclc_subscription_init_default(
        &snekbot_joint_state_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "snekbot/test_joint_states"));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &snekbot_joint_state_subscriber, &recv_joint_state_msg, &snekbot_joint_state_callback, ON_NEW_DATA));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        usleep(10000);
    }

    RCCHECK(rcl_subscription_fini(&snekbot_joint_state_subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void) {
    setup_pwm();
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif
    xTaskCreate(micro_ros_task, "uros_task", 8 * 1024, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>  // For malloc

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "driver/mcpwm_prelude.h"
#include <math.h>  // For M_PI constant

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define NUM_JOINTS 7  // Updated to 7 to include the gripper
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); vTaskDelete(NULL);} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);} }

// Subscription variable
rcl_subscription_t snekbot_joint_state_subscriber;
sensor_msgs__msg__JointState recv_joint_state_msg;

#define SERVO_GPIO 2  // Change to the GPIO pin you are using
#define SERVO_MIN_PULSEWIDTH_US 500   // Minimum pulse width (0°)
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width (180°)
#define SERVO_MAX_DEGREE 180          // Maximum degree of rotation
#define PI 3.14159265359               // Define constant PI

mcpwm_cmpr_handle_t comparator;
mcpwm_timer_handle_t timer;
mcpwm_oper_handle_t operator;
mcpwm_gen_handle_t generator;

static uint32_t radians_to_angle(float radians) {
    // Map radians to degrees where -PI -> 0° and PI -> 180°
    float angle = ((radians + M_PI) / (2 * M_PI)) * SERVO_MAX_DEGREE;
    
    // Ensure the angle stays within 0-180 range
    if (angle < 0) angle = 0;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    
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

// Setup the MCPWM for servo control
static void setup_pwm(void) {
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
}

// Subscription callback function
void snekbot_joint_state_callback(const void * msgin)
{
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;

    if (msg == NULL) {
        printf("[ERROR] Received NULL message!\n");
        return;
    }

    printf("Received Joint States:\n");
    for (size_t i = 0; i < msg->position.size; i++) {
        printf("  Joint %zu (%s): %f\n", i, msg->name.data[i].data, msg->position.data[i]);
        if (strcmp(msg->name.data[i].data, "gripper") == 0) {
            printf("  fuck ye %zu (%s): %f\n", i, msg->name.data[i].data, msg->position.data[i]);
            float gripper_angle = msg->position.data[i];
            set_servo_angle_radians(gripper_angle);
        }
    }
}

// Micro-ROS task to handle subscriptions and execution loop
void micro_ros_task(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Initialize RCL options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    // Initialize RCL support
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create the ROS node
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "snekbot_joint_state_subscriber_rclc", "", &support));

    // Initialize subscription for SnekBot joint states
    RCCHECK(rclc_subscription_init_default(
        &snekbot_joint_state_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "snekbot/test_joint_states"));

    // Allocate memory for received message
    recv_joint_state_msg.position.capacity = NUM_JOINTS;  // Updated to 7 joints
    recv_joint_state_msg.position.size = 0;
    recv_joint_state_msg.position.data = malloc(NUM_JOINTS * sizeof(double));
    
    recv_joint_state_msg.name.capacity = NUM_JOINTS;  // Updated to 7 joints
    recv_joint_state_msg.name.size = 0;
    recv_joint_state_msg.name.data = malloc(NUM_JOINTS * sizeof(rosidl_runtime_c__String));
    
    for (size_t i = 0; i < NUM_JOINTS; i++) {
        recv_joint_state_msg.name.data[i].data = malloc(20);
        recv_joint_state_msg.name.data[i].size = 0;
        recv_joint_state_msg.name.data[i].capacity = 20;
    }

    if (!recv_joint_state_msg.position.data || !recv_joint_state_msg.name.data) {
        printf("[ERROR] Failed to allocate memory for received message\n");
        return;
    }

    // Create executor for the subscription
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // Add subscription to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &snekbot_joint_state_subscriber, &recv_joint_state_msg, &snekbot_joint_state_callback, ON_NEW_DATA));

    printf("Micro-ROS subscriber is running...\n");

    // Spin to handle callbacks
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        usleep(10000);
    }

    // Clean up
    RCCHECK(rcl_subscription_fini(&snekbot_joint_state_subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    free(recv_joint_state_msg.position.data);
    for (size_t i = 0; i < NUM_JOINTS; i++) {
        free(recv_joint_state_msg.name.data[i].data);
    }
    free(recv_joint_state_msg.name.data);

    vTaskDelete(NULL);
}

void app_main(void)
{
    setup_pwm();
    // Initialize network interface
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // Create the micro-ROS task with increased stack size
    xTaskCreate(micro_ros_task,
                "uros_task",
                8 * 1024,  // Increased stack size
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}

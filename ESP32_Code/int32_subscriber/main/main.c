#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float64.h>
#include <sensor_msgs/msg/joint_state.h>  // Use JointState message type
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);} }

rcl_subscription_t gripper_subscriber;
rcl_subscription_t joint_states_subscriber;
std_msgs__msg__Float64 recv_gripper_msg;
sensor_msgs__msg__JointState recv_joint_states_msg;  // Update to JointState message type

// Gripper subscription callback function
void gripper_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
    printf("Received gripper position: %.2f\n", msg->data);
}

// Joint states subscription callback function
void joint_states_subscription_callback(const void * msgin)
{
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
    printf("Received joint states:\n");
    for (int i = 0; i < msg->position.size; i++) {
        printf("Joint %d position: %.2f\n", i, msg->position.data[i]);
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
    RCCHECK(rclc_node_init_default(&node, "gripper_joint_states_subscriber_rclc", "", &support));

    // Initialize subscription for gripper position
    RCCHECK(rclc_subscription_init_default(
        &gripper_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "snekbot/gripper_position"));

    // Initialize subscription for joint states
    RCCHECK(rclc_subscription_init_default(
        &joint_states_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),  // Use JointState
        "snekbot/test_joint_states"));

    // Create executor for the subscriptions
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    // Add subscriptions to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &gripper_subscriber, &recv_gripper_msg, &gripper_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &joint_states_subscriber, &recv_joint_states_msg, &joint_states_subscription_callback, ON_NEW_DATA));

    // Spin to handle callbacks
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        usleep(10000);
    }

    // Clean up
    RCCHECK(rcl_subscription_fini(&gripper_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&joint_states_subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
    // Initialize network interface
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // Create the micro-ROS task
    xTaskCreate(micro_ros_task,
                "uros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}

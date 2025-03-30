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
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);} }

rcl_publisher_t received_publisher;  // Publisher for the 'received' topic
rcl_subscription_t joint_state_subscriber;
rcl_subscription_t gripper_state_subscriber;
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float64 gripper_state_msg;

void joint_state_subscription_callback(const void * msgin)
{
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
    printf("Received joint states:\n");
    for (size_t i = 0; i < msg->name.size; i++) {
        printf("%s: %.3f\n", msg->name.data[i], msg->position.data[i]);
    }
    
    // Publish the received joint state data to 'received' topic
    char received_data[256];
    snprintf(received_data, sizeof(received_data), "Received joint states: ");
    for (size_t i = 0; i < msg->name.size; i++) {
        snprintf(received_data + strlen(received_data), sizeof(received_data) - strlen(received_data), "%s: %.3f, ", msg->name.data[i], msg->position.data[i]);
    }
    // Remove the trailing comma and space
    received_data[strlen(received_data) - 2] = '\0';
    
    std_msgs__msg__String received_msg;
    received_msg.data.data = (char*)received_data;
    received_msg.data.size = strlen(received_data);
    RCSOFTCHECK(rcl_publish(&received_publisher, &received_msg, NULL));
    printf("Sent to 'received' topic: %s\n", received_data);
}

void gripper_state_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
    printf("Received gripper state: %.3f\n", msg->data);

    // Publish the received gripper state data to 'received' topic
    char received_data[256];
    snprintf(received_data, sizeof(received_data), "Received gripper state: %.3f", msg->data);

    std_msgs__msg__String received_msg;
    received_msg.data.data = (char*)received_data;
    received_msg.data.size = strlen(received_data);
    RCSOFTCHECK(rcl_publish(&received_publisher, &received_msg, NULL));
    printf("Sent to 'received' topic: %s\n", received_data);
}

void micro_ros_task(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodiscovery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    // Setup support structure.
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node.
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "esp32_subscriber_publisher", "", &support));

    // Create publisher for 'received' topic
    RCCHECK(rclc_publisher_init_default(
        &received_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "received"));

    // Create subscriptions for the joint states and gripper state topics
    RCCHECK(rclc_subscription_init_default(
        &joint_state_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "snekbot/joint_states"));
    
    RCCHECK(rclc_subscription_init_default(
        &gripper_state_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "snekbot/gripper_position"));

    // Create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    unsigned int rcl_wait_timeout = 1000;   // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Add subscriptions to the executor
    RCCHECK(rclc_executor_add_subscription(&executor, &joint_state_subscriber, &joint_state_msg, &joint_state_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &gripper_state_subscriber, &gripper_state_msg, &gripper_state_subscription_callback, ON_NEW_DATA));

    // Spin forever.
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // Free resources
    RCCHECK(rcl_subscription_fini(&joint_state_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&gripper_state_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&received_publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
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

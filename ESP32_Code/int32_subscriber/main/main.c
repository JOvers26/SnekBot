#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/string.h>  // Use String message type for JSON data
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <cjson/cJSON.h>  // Include cJSON library for JSON parsing

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);} }

rcl_subscription_t snekbot_data_subscriber;
std_msgs__msg__String recv_snekbot_data_msg;  // String message type for receiving JSON data

// Subscription callback function for receiving the JSON data
void snekbot_data_subscription_callback(const void * msgin)
{
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    
    // Parse the JSON string
    cJSON *json = cJSON_Parse(msg->data.data);
    if (json == NULL) {
        printf("Error parsing JSON\n");
        return;
    }

    // Extract joint states
    cJSON *joint_1 = cJSON_GetObjectItem(json, "joint_1");
    cJSON *joint_2 = cJSON_GetObjectItem(json, "joint_2");
    cJSON *joint_3 = cJSON_GetObjectItem(json, "joint_3");
    cJSON *joint_4 = cJSON_GetObjectItem(json, "joint_4");
    cJSON *joint_5 = cJSON_GetObjectItem(json, "joint_5");
    cJSON *joint_6 = cJSON_GetObjectItem(json, "joint_6");

    // Extract gripper position
    cJSON *gripper = cJSON_GetObjectItem(json, "gripper");

    // Print the data
    if (joint_1 && joint_2 && joint_3 && joint_4 && joint_5 && joint_6) {
        printf("Received joint states:\n");
        printf("Joint 1 position: %.2f\n", joint_1->valuedouble);
        printf("Joint 2 position: %.2f\n", joint_2->valuedouble);
        printf("Joint 3 position: %.2f\n", joint_3->valuedouble);
        printf("Joint 4 position: %.2f\n", joint_4->valuedouble);
        printf("Joint 5 position: %.2f\n", joint_5->valuedouble);
        printf("Joint 6 position: %.2f\n", joint_6->valuedouble);
    }

    if (gripper) {
        printf("Gripper position: %.2f\n", gripper->valuedouble);
    } else {
        printf("Gripper position: None\n");
    }

    // Clean up JSON object
    cJSON_Delete(json);
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
    RCCHECK(rclc_node_init_default(&node, "snekbot_data_subscriber_rclc", "", &support));

    // Initialize subscription for JSON data
    RCCHECK(rclc_subscription_init_default(
        &snekbot_data_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "snekbot/data"));

    // Create executor for the subscriptions
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // Add subscription to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &snekbot_data_subscriber, &recv_snekbot_data_msg, &snekbot_data_subscription_callback, ON_NEW_DATA));

    // Spin to handle callbacks
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        usleep(10000);
    }

    // Clean up
    RCCHECK(rcl_subscription_fini(&snekbot_data_subscriber, &node));
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

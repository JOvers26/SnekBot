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
#include <sensor_msgs/msg/joint_state.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Joint State Subscriber
rcl_subscription_t joint_state_subscriber;
sensor_msgs__msg__JointState recv_joint_state;

void joint_state_callback(const void * msgin)
{
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
    
    printf("Received Joint State:\n");
    for (size_t i = 0; i < msg->position.size; i++) {
        printf(" - %s: %f\n", msg->name.data[i].data, msg->position.data[i]);
    }
}

void micro_ros_task(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Initialize ROS 2
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    // Setup support structure
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "snekbot_joint_state_subscriber", "", &support));

    // Create subscriber for JointState
    RCCHECK(rclc_subscription_init_default(
        &joint_state_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "snekbot/joint_states"));

    // Create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // Add subscriber to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &joint_state_subscriber, &recv_joint_state, &joint_state_callback, ON_NEW_DATA));

    // Spin forever
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // Free resources
    RCCHECK(rcl_subscription_fini(&joint_state_subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // Pin micro-ros task in APP_CPU to make PRO_CPU deal with WiFi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}

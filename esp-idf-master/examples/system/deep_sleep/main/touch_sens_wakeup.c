/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/touch_sens.h"
#include "touch_sens_example_config.h"

static const char *TAG = "touch_wakeup";

#define EXAMPLE_TOUCH_SAMPLE_CFG_NUM         1
#define EXAMPLE_TOUCH_CHANNEL_NUM            3
#define EXAMPLE_TOUCH_CHAN_INIT_SCAN_TIMES   3

// The touch channel IDs that used in this example
// If you want to change the wake-up channels, please make sure the channel GPIOs won't conflict to the EXT wakeup GPIOs
static int s_channel_id[EXAMPLE_TOUCH_CHANNEL_NUM] = {7, 8, 9};

// Active threshold to benchmark ratio. (i.e., touch will be activated when data >= benchmark * (1 + ratio))
static float s_thresh2bm_ratio[EXAMPLE_TOUCH_CHANNEL_NUM] = {
    [0 ... EXAMPLE_TOUCH_CHANNEL_NUM - 1] = 0.02f,  // 2%
};

static bool example_touch_on_active_cb(touch_sensor_handle_t sens_handle, const touch_active_event_data_t *event, void *user_ctx)
{
    ESP_EARLY_LOGW("touch callback", "ch %d active", (int)event->chan_id);
    return false;
}

static bool example_touch_on_inactive_cb(touch_sensor_handle_t sens_handle, const touch_inactive_event_data_t *event, void *user_ctx)
{
    ESP_EARLY_LOGW("touch callback", "ch %d inactive", (int)event->chan_id);
    return false;
}

static void example_touch_do_initial_scanning(touch_sensor_handle_t sens_handle, touch_channel_handle_t chan_handle[])
{
    /* Enable the touch sensor to do the initial scanning, so that to initialize the channel data */
    ESP_ERROR_CHECK(touch_sensor_enable(sens_handle));

    /* Scan the enabled touch channels for several times, to make sure the initial channel data is stable */
    for (int i = 0; i < EXAMPLE_TOUCH_CHAN_INIT_SCAN_TIMES; i++) {
        ESP_ERROR_CHECK(touch_sensor_trigger_oneshot_scanning(sens_handle, 2000));
    }

    /* Disable the touch channel to rollback the state */
    ESP_ERROR_CHECK(touch_sensor_disable(sens_handle));

    /* (Optional) Read the initial channel benchmark and reconfig the channel active threshold accordingly */
    printf("Initial benchmark and new threshold are:\n");
    for (int i = 0; i < EXAMPLE_TOUCH_CHANNEL_NUM; i++) {
        /* Read the initial benchmark of the touch channel */
        uint32_t benchmark[EXAMPLE_TOUCH_SAMPLE_CFG_NUM] = {};
#if SOC_TOUCH_SUPPORT_BENCHMARK
        ESP_ERROR_CHECK(touch_channel_read_data(chan_handle[i], TOUCH_CHAN_DATA_TYPE_BENCHMARK, benchmark));
#else
        /* Read smooth data instead if the hardware does not support benchmark */
        ESP_ERROR_CHECK(touch_channel_read_data(chan_handle[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, benchmark));
#endif  // SOC_TOUCH_SUPPORT_BENCHMARK
        /* Calculate the proper active thresholds regarding the initial benchmark */
        printf("Touch [CH %d]", s_channel_id[i]);
        /* Generate the default channel configuration and then update the active threshold based on the real benchmark */
        touch_channel_config_t chan_cfg = EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();
        for (int j = 0; j < EXAMPLE_TOUCH_SAMPLE_CFG_NUM; j++) {
#if SOC_TOUCH_SENSOR_VERSION == 1
            // Touch V1 (ESP32) uses absolute threshold.
            chan_cfg.abs_active_thresh[j] = (uint32_t)(benchmark[j] * (1 - s_thresh2bm_ratio[i]));
            printf(" %d: %"PRIu32", %"PRIu32"\t", j, benchmark[j], chan_cfg.abs_active_thresh[j]);
#else
            chan_cfg.active_thresh[j] = (uint32_t)(benchmark[j] * s_thresh2bm_ratio[i]);
            printf(" %d: %"PRIu32", %"PRIu32"\t", j, benchmark[j], chan_cfg.active_thresh[j]);
#endif  // SOC_TOUCH_SENSOR_VERSION == 1
        }
        printf("\n");
        /* Update the channel configuration */
        ESP_ERROR_CHECK(touch_sensor_reconfig_channel(chan_handle[i], &chan_cfg));
    }
}

esp_err_t example_deep_sleep_register_touch_wakeup(void)
{
    printf("Enabling touch wakeup\n");

    /* Handles of touch sensor */
    touch_sensor_handle_t sens_handle = NULL;
    touch_channel_handle_t chan_handle[EXAMPLE_TOUCH_CHANNEL_NUM];

    /* Step 1: Create a new touch sensor controller handle with default sample configuration */
    touch_sensor_sample_config_t sample_cfg[TOUCH_SAMPLE_CFG_NUM] = EXAMPLE_TOUCH_SAMPLE_CFG_DEFAULT();
    touch_sensor_config_t sens_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(EXAMPLE_TOUCH_SAMPLE_CFG_NUM, sample_cfg);
    ESP_ERROR_CHECK(touch_sensor_new_controller(&sens_cfg, &sens_handle));

    /* Step 2: Create and enable the new touch channel handles with default configurations */
    touch_channel_config_t chan_cfg = EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();
    for (int i = 0; i < EXAMPLE_TOUCH_CHANNEL_NUM; i++) {
        ESP_ERROR_CHECK(touch_sensor_new_channel(sens_handle, s_channel_id[i], &chan_cfg, &chan_handle[i]));
        /* Display the touch channel corresponding GPIO number, you can also know from `touch_sensor_channel.h` */
        touch_chan_info_t chan_info = {};
        ESP_ERROR_CHECK(touch_sensor_get_channel_info(chan_handle[i], &chan_info));
        printf("Touch [CH %d] enabled on GPIO%d\n", s_channel_id[i], chan_info.chan_gpio);
    }

    /* Step 3: Confiture the default filter for the touch sensor (Note: Touch V1 uses software filter) */
    touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    ESP_ERROR_CHECK(touch_sensor_config_filter(sens_handle, &filter_cfg));

    /* (Optional) Do the initial scanning to initialize the touch channel data
     * Without this step, the channel data in the first read will be invalid
     */
    example_touch_do_initial_scanning(sens_handle, chan_handle);

    /* (Optional) Register the callbacks, optional for deep sleep wakeup */
    touch_event_callbacks_t callbacks = {
        .on_active = example_touch_on_active_cb,
        .on_inactive = example_touch_on_inactive_cb,
    };
    ESP_ERROR_CHECK(touch_sensor_register_callbacks(sens_handle, &callbacks, NULL));

    /* Step 4: Enable the deep sleep wake-up with the basic configuration */
#if CONFIG_EXAMPLE_TOUCH_ALLOW_DSLP_PD
    /* Get the channel information to use same active threshold for the sleep channel */
    touch_chan_info_t chan_info = {};
    ESP_ERROR_CHECK(touch_sensor_get_channel_info(chan_handle[0], &chan_info));

    touch_sleep_config_t deep_slp_cfg = TOUCH_SENSOR_DEFAULT_DSLP_PD_CONFIG(chan_handle[0],
                                                                            chan_info.active_thresh[0],
#if SOC_TOUCH_SENSOR_VERSION == 3
                                                                            chan_info.active_thresh[1],
                                                                            chan_info.active_thresh[2],
#endif
                                                                        );
    printf("Touch channel %d (GPIO%d) is selected as deep sleep wakeup channel\n", chan_info.chan_id, chan_info.chan_gpio);
#else
    touch_sleep_config_t deep_slp_cfg = TOUCH_SENSOR_DEFAULT_DSLP_CONFIG();
#endif
    /* Enable deep sleep wake up for touch sensor */
    ESP_ERROR_CHECK(touch_sensor_config_sleep_wakeup(sens_handle, &deep_slp_cfg));

    /* Step 5: Enable touch sensor controller and start continuous scanning before entering deep sleep */
    ESP_ERROR_CHECK(touch_sensor_enable(sens_handle));
    ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(sens_handle));

    ESP_LOGI(TAG, "touch wakeup source is ready");
    return ESP_OK;
}

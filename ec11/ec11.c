/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <stdatomic.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

#include "ec11.h"

static const char *TAG = "ec11";

#define EXAMPLE_PCNT_HIGH_LIMIT 4
#define EXAMPLE_PCNT_LOW_LIMIT  -4

#define EXAMPLE_EC11_GPIO_A 14
#define EXAMPLE_EC11_GPIO_B 12

int ec11_pcnt_init(ec11_t *ec11)
{
    if(!ec11){
        ESP_LOGE(TAG, "func[%s] args err", __func__);
        return -1;
    }
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
        .level_gpio_num = EXAMPLE_EC11_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
        .level_gpio_num = EXAMPLE_EC11_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    // int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
    int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        // .on_reach = example_pcnt_on_reach,
        .on_reach = ec11->ec11_rotary_handle,
    };
    // QueueHandle_t ec11_evt_queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, ec11->ec11_evt_queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    return 0;
}

#define EC11_KEY_GPIO 39
#define EC11_GPIO_INPUT_PIN_SEL (1ULL<<EC11_KEY_GPIO)

static TaskHandle_t key_task_handle = NULL;

static uint32_t last_time = 0;
uint32_t cnt_test = 0;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    uint32_t now = xTaskGetTickCountFromISR();
    if (now - last_time > pdMS_TO_TICKS(50)) {  // 消抖
        if (gpio_get_level(EC11_KEY_GPIO) == 0) {  // 确认是按下状态
            last_time = now;
            cnt_test++;
            vTaskNotifyGiveFromISR(key_task_handle, &xHigherPriorityTaskWoken);
        }
    }
    
    if(xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void key_task(void* arg)
{
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("EC11 Key pressed!\n");
        printf("cnt_test: %ld\n", cnt_test);
    }
}
    
int ec11_key_init(ec11_t *ec11)
{
    xTaskCreate(key_task, "key_task", 2048, NULL, 10, &key_task_handle);

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = EC11_GPIO_INPUT_PIN_SEL;
    // io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_isr_handler_add(EC11_KEY_GPIO, gpio_isr_handler, NULL);

    return 0;
}

int ec11_init(ec11_t *ec11)
{
    if(!ec11){
        ESP_LOGE(TAG, "func[%s] args err", __func__);
        return -1;
    }
    if(ec11_pcnt_init(ec11)){
        ESP_LOGE(TAG, "func[%s] ec11_pcnt_init err", __func__);
        return -1;
    }
    if(ec11_key_init(ec11)){
        ESP_LOGE(TAG, "func[%s] ec11_key_init err", __func__);
        return -1;
    }
    return 0;
}

ec11_t *ec11_create(ec11_rotary_handle_t ec11_rotary_handle, QueueHandle_t ec11_evt_queue)
{
    if(!ec11_rotary_handle){
        ESP_LOGE(TAG, "func[%s] args err", __func__);
        return NULL;
    }

    ec11_t *ec11 = calloc(1, sizeof(ec11_t));
    if(!ec11){
        ESP_LOGE(TAG, "func[%s] calloc err", __func__);
        return NULL;
    }

    ec11->ec11_rotary_handle = ec11_rotary_handle;
    ec11->ec11_evt_queue = ec11_evt_queue;

    return ec11;
}

int ec11_destroy(ec11_t *ec11)
{
    if(!ec11){
        ESP_LOGE(TAG, "func[%s] args err", __func__);
        return -1;
    }

    free(ec11);
    ec11 = NULL;

    return 0;
}
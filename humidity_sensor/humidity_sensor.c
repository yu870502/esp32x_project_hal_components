/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_filter.h"
#include "driver/gpio.h"
#include "soc/adc_periph.h"
#include "soc/soc_caps.h"

#include "unity.h"
#include "driver/rtc_io.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "humidity_sensor.h"

#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_12
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN                    256

typedef struct humidity_obj_t{
    struct humidity_obj_t *humidity_obj;
    get_humidity_process_t humidity_process;
    float procss_arg; 
}humidity_t;
static humidity_t *humiObj = NULL;

static adc_channel_t channel[2] = {ADC_CHANNEL_4, ADC_CHANNEL_5};

static TaskHandle_t s_task_handle;
static const char *TAG = "HUMI";

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

#if SOC_ADC_DIG_IIR_FILTER_SUPPORTED
adc_digi_iir_filter_coeff_t g_test_filter_coeff[sizeof(channel) / sizeof(adc_channel_t)] = {
    ADC_DIGI_IIR_FILTER_COEFF_2,
    ADC_DIGI_IIR_FILTER_COEFF_4,
    ADC_DIGI_IIR_FILTER_COEFF_8,
    ADC_DIGI_IIR_FILTER_COEFF_16,
    ADC_DIGI_IIR_FILTER_COEFF_64,
};
#endif

#define ADC_GET_IO_NUM(unit, channel) (adc_channel_io_map[unit][channel])

void test_adc_set_io_middle(adc_unit_t unit, adc_channel_t channel)
{
    // TEST_ASSERT(channel < SOC_ADC_CHANNEL_NUM(unit) && "invalid channel");

    uint32_t io_num = ADC_GET_IO_NUM(unit, channel);
    TEST_ESP_OK(gpio_set_pull_mode(io_num, GPIO_PULLUP_PULLDOWN));
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
    if (rtc_gpio_is_valid_gpio(io_num)) {
        TEST_ESP_OK(rtc_gpio_pullup_en(io_num));
        // TEST_ESP_OK(rtc_gpio_pulldown_en(io_num));
    }
#endif
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

adc_cali_handle_t cali_handle = NULL;
static int voltage = 0;
static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_cali_scheme_ver_t scheme_mask;
    esp_err_t ret = adc_cali_check_scheme(&scheme_mask);
    if(ESP_OK != ret){
        ESP_LOGE(TAG, "adc_cali_check_scheme failed");
    }
    ESP_LOGI(TAG, "cali scheme mask:%d\n", scheme_mask);
    if(ADC_CALI_SCHEME_VER_LINE_FITTING == scheme_mask){
        ESP_LOGI(TAG, "cali scheme is ADC_CALI_SCHEME_VER_LINE_FITTING");
    }
    else if(ADC_CALI_SCHEME_VER_CURVE_FITTING == scheme_mask){
        ESP_LOGI(TAG, "cali scheme is ADC_CALI_SCHEME_VER_CURVE_FITTING");
    }
    else{
        ESP_LOGI(TAG, "cali scheme is UNKNOW");
    }

    adc_cali_line_fitting_efuse_val_t cali_val;
    ret = adc_cali_scheme_line_fitting_check_efuse(&cali_val);
    if(ESP_OK != ret){
        ESP_LOGE(TAG, "adc_cali_scheme_line_fitting_check_efuse failed");
    }
    ESP_LOGI(TAG, " eFuse val:%d\n", cali_val);

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = EXAMPLE_ADC_UNIT,
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = EXAMPLE_ADC_BIT_WIDTH,
        .default_vref = 3300,
    };    
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "calibration success", cali_val);
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "calibration fail due to lack of eFuse bits");
    } else {
        TEST_ASSERT(false);
    }

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
        .flags.flush_pool = 1,
    };
    adc_continuous_handle_t handle = NULL;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        // ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        // ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
        // ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);

#if SOC_ADC_DIG_IIR_FILTER_SUPPORTED
        adc_iir_filter_handle_t filter_hdl = NULL;
        adc_continuous_iir_filter_config_t filter_config = {
            .unit = EXAMPLE_ADC_UNIT,
            .channel = adc_pattern[i].channel,
            .coeff = g_test_filter_coeff[i],
        };
        TEST_ESP_OK(adc_new_continuous_iir_filter(handle, &filter_config, &filter_hdl));
        TEST_ESP_OK(adc_continuous_iir_filter_enable(filter_hdl));

        test_adc_set_io_middle(EXAMPLE_ADC_UNIT, adc_pattern[i].channel);
        ESP_LOGI("TEST_ADC", "Test with filter coeff: %d", g_test_filter_coeff[i]);
#endif
    // TEST_ESP_OK(gpio_set_pull_mode(32, GPIO_PULLUP_ONLY));
    // TEST_ESP_OK(gpio_set_pull_mode(33, GPIO_PULLUP_ONLY));

    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));   

    *out_handle = handle;
}

void continuous_read_task(void *)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while (1) {

        /**
         * This is to show you the way to use the ADC continuous mode driver event callback.
         * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
         * However in this example, the data processing (print) is slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still just call
         * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

        while (1) {
            // ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
            ret = adc_continuous_read(handle, result, 4, &ret_num, 0);
            if (ret == ESP_OK) {
                ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    // printf("reslut size:%d\n", sizeof(adc_digi_output_data_t));
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                    adc_cali_raw_to_voltage(cali_handle, data, &voltage);
                    /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                    if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                        ESP_LOGI(TAG, "Raw data:%#x, data:%d, voltage:%dmv, Channel: %"PRIu32, p->val, data, voltage, chan_num);
                        float h = (float)(voltage * 244) / 10000.0;
                        ESP_LOGI(TAG, "hum:%f", h);
                        if(!humiObj){
                            ESP_LOGW(TAG, "humiObj is none");
                        }
                        else{
                            humiObj->humidity_process(h);
                        }
                    } else {
                        ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                    }
                }
                /**
                 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                 * usually you don't need this delay (as this task will block for a while).
                 */
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}

int start_humidity_sensor(get_humidity_process_t humidity_process)
{
    humiObj = calloc(1, sizeof(humidity_t));
    if(!humiObj){
        ESP_LOGW(TAG, "Create humidity sensor failed\n");
        return -1;
    }
    humiObj->humidity_process = humidity_process;
    xTaskCreate(continuous_read_task, "continuous_read_task", 4 * 1024, NULL, 5, NULL);
    return 0; 
}

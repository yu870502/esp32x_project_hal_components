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


#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_12
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH


humidity_t *humiObj = NULL;

static const char *TAG = "HUMI";

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

adc_cali_handle_t cali_handle;
void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
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

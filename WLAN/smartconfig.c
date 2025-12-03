/* Esptouch example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_eap_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "esp_mac.h"

#include "smartconfig_monitor.h"
#include "network_info.h"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
static const char *TAG = "smartconfig";
static const char *TAG_FAST_CONN = "FAST_CONN";

static void smartconfig_task(void * parm);

#define KEY_SSID "ssid"
#define KEY_PW "pw"
#define KEY_CFNET_STATUS "configNetStatus"
#define NVS_NAMESPACE_CONFIGNET "configNet"
#define CONFIGNET_OK 0
#define CONFIGNET_NONE (-1)

#define MAX_SSID_LEN 32
#define MAX_PASSWORD_LEN 64

int8_t configNetStatus = CONFIGNET_NONE;

//key_evt_cb_t
int force_configNet(void *arg)
{
    ESP_LOGW(TAG, "Start force_configNet!!!");

    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs_handle;
    if(ESP_OK != (ret = nvs_open(NVS_NAMESPACE_CONFIGNET, NVS_READWRITE, &nvs_handle))){
        ESP_LOGW(TAG, "nvs_open failed error (%s) opening NVS handle!", esp_err_to_name(ret));
        return -1;
    }    

    configNetStatus = CONFIGNET_NONE;
    ESP_LOGW(TAG, "force configNetStatus = %d", configNetStatus);
    if(ESP_OK != (ret = nvs_set_u8(nvs_handle, KEY_CFNET_STATUS, (uint8_t)configNetStatus))){
        ESP_LOGW(TAG, "nvs_set_u8 configNetStatus error (%s)", esp_err_to_name(ret));
        return -1;  
    }
    if(ESP_OK != (ret = nvs_commit(nvs_handle))){
        ESP_LOGW(TAG, "nvs_senvs_committ_str error (%s)", esp_err_to_name(ret));
        return -1;  
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp_restart();
}

static int store_configNet_info(void)
{
    char ssid[MAX_SSID_LEN + 1] = { 0 };
    char password[MAX_PASSWORD_LEN + 1] = { 0 };

    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs_handle;
    if(ESP_OK != (ret = nvs_open(NVS_NAMESPACE_CONFIGNET, NVS_READWRITE, &nvs_handle))){
        ESP_LOGW(TAG, "nvs_open failed error (%s) opening NVS handle!", esp_err_to_name(ret));
        return -1;
    }    

    ESP_LOGI(TAG, "configNetStatus = %d", configNetStatus);
    if(ESP_OK != (ret = nvs_set_u8(nvs_handle, KEY_CFNET_STATUS, (uint8_t)configNetStatus))){
        ESP_LOGW(TAG, "nvs_set_u8 configNetStatus error (%s)", esp_err_to_name(ret));
        return -1;  
    }
    if(ESP_OK != (ret = nvs_commit(nvs_handle))){
        ESP_LOGW(TAG, "nvs_senvs_committ_str error (%s)", esp_err_to_name(ret));
        return -1;  
    }

    // if(ESP_OK != (ret = nvs_get_u8(nvs_handle, KEY_CFNET_STATUS, (uint8_t *)&configNetStatus))){
    //     ESP_LOGE(TAG, "nvs_get_u8 configNetStatus error (%s) opening NVS handle!", esp_err_to_name(ret));
    // }
    // ESP_LOGI(TAG, "read configNetStatus = %d", configNetStatus);    

    wifi_config_t wifi_config;
    bzero(&wifi_config, sizeof(wifi_config_t));
    if(ESP_OK != (ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_config))){
        ESP_LOGW(TAG, "esp_wifi_get_config error (%s)", esp_err_to_name(ret));
        return -1;
    }
    memcpy(ssid, wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid));
    memcpy(password, wifi_config.sta.password, sizeof(wifi_config.sta.password));
    // ESP_LOGW(TAG, "get config ssid = %s", ssid);
    // ESP_LOGW(TAG, "get config password = %s", password);

    if(ESP_OK != (ret = nvs_set_str(nvs_handle, KEY_SSID, ssid))){
        ESP_LOGE(TAG, "nvs_set_str ssid error (%s)", esp_err_to_name(ret));
        return -1;
    }
    if(ESP_OK != (ret = nvs_commit(nvs_handle))){
        ESP_LOGW(TAG, "nvs_senvs_committ_str ssd error (%s)", esp_err_to_name(ret));
        return -1;  
    }
    // size_t length = MAX_SSID_LEN;
    // memset(ssid, 0, MAX_SSID_LEN + 1);
    // if(ESP_OK != (ret = nvs_get_str(nvs_handle, KEY_SSID, ssid, &length))){
    //     ESP_LOGW(TAG, "nvs_get_str ssid error (%s)", esp_err_to_name(ret));
    //     ESP_LOGW(TAG, "length = %d", length);
    //     return -1;  
    // }
    // ESP_LOGW(TAG, "read ssid = %s", ssid);

    if(ESP_OK != (ret = nvs_set_str(nvs_handle, KEY_PW, password))){
        ESP_LOGW(TAG, "nvs_set_str password error (%s)", esp_err_to_name(ret));
        return -1;  
    }
    if(ESP_OK != (ret = nvs_commit(nvs_handle))){
        ESP_LOGW(TAG, "nvs_senvs_committ_str password error (%s)", esp_err_to_name(ret));
        return -1;  
    }
    // length = MAX_PASSWORD_LEN;
    // memset(password, 0, MAX_PASSWORD_LEN + 1);
    // if(ESP_OK != (ret = nvs_get_str(nvs_handle, KEY_PW, password, &length))){
    //     ESP_LOGW(TAG_FAST_CONN, "nvs_get_str password error (%s)", esp_err_to_name(ret));
    //     return -1;  
    // }
    // ESP_LOGW(TAG, "read password = %s", password);

    if(nvs_handle){
        nvs_close(nvs_handle);
        nvs_handle = 0;
    }

    return 0;
}

static void fast_conn_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG_FAST_CONN, "WIFI start");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG_FAST_CONN, "WIFI disconn, start reconnecting");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
        ESP_LOGI(TAG_FAST_CONN, "WIFI got ip");
    }
}

static int start_fast_conn(nvs_handle_t nvs_handle)
{
    wifi_config_t wifi_config;
    char ssid[MAX_SSID_LEN + 1] = { 0 };      //SSID：1-32字节，几乎包含所有可打印字符
    char password[MAX_PASSWORD_LEN + 1] = { 0 };  //Password：8-63字节，通常只使用可打印ASCII字符（32-126）
    size_t length = 0;

    esp_err_t ret = ESP_OK;
    if(!nvs_handle){
        if(ESP_OK != (ret = nvs_open(NVS_NAMESPACE_CONFIGNET, NVS_READWRITE, &nvs_handle))){
            ESP_LOGW(TAG_FAST_CONN, "nvs_open error (%s)", esp_err_to_name(ret));
            return -1;
        }
    }
    
    length = MAX_SSID_LEN;
    if(ESP_OK != (ret = nvs_get_str(nvs_handle, KEY_SSID, ssid, &length))){
        ESP_LOGW(TAG_FAST_CONN, "nvs_get_str ssid error (%s)", esp_err_to_name(ret));
        return -1;  
    }
    length = MAX_PASSWORD_LEN;
    if(ESP_OK != (ret = nvs_get_str(nvs_handle, KEY_PW, password, &length))){
        ESP_LOGW(TAG_FAST_CONN, "nvs_get_str password error (%s)", esp_err_to_name(ret));
        return -1;  
    }

    if(nvs_handle){
        nvs_close(nvs_handle);
        nvs_handle = 0;
    }

    bzero(&wifi_config, sizeof(wifi_config_t));
    memcpy(wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    ESP_LOGI(TAG, "SSID:%s", ssid);
    ESP_LOGI(TAG, "PASSWORD:%s", password);

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_connect() );
    return 0;
}

static void smartconfig_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
        //After connecting to WiFi normally, extract SSID and PW, submit them to the required module, or save them directly.
        ESP_LOGI(TAG_FAST_CONN, "Start the store confignet info process");
        configNetStatus = CONFIGNET_OK; 
        store_configNet_info();
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[MAX_SSID_LEN + 1] = { 0 };
        uint8_t password[MAX_PASSWORD_LEN + 1] = { 0 };
        uint8_t rvd_data[33] = { 0 };

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));

#ifdef CONFIG_SET_MAC_ADDRESS_OF_TARGET_AP
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            ESP_LOGI(TAG, "Set MAC address of target AP: "MACSTR" ", MAC2STR(evt->bssid));
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }
#endif

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2) {
            ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
            ESP_LOGI(TAG, "RVD_DATA:");
            for (int i=0; i<33; i++) {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }

        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
        esp_wifi_connect();
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

static void initialise_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs_handle;
    if(ESP_OK != (ret = nvs_open(NVS_NAMESPACE_CONFIGNET, NVS_READWRITE, &nvs_handle))){
        ESP_LOGW(TAG, "nvs_open failed error (%s) opening NVS handle!", esp_err_to_name(ret));
        return;
    }

    if(ESP_OK != (ret = nvs_get_u8(nvs_handle, KEY_CFNET_STATUS, (uint8_t *)&configNetStatus))){
        ESP_LOGW(TAG, "nvs_get_u8 configNetStatus error (%s) opening NVS handle!", esp_err_to_name(ret));
    }
    ESP_LOGW(TAG, "configNetStatus = %d", configNetStatus);

    if(CONFIGNET_OK == configNetStatus){
        ESP_LOGW(TAG_FAST_CONN, "Start the Fast Link process");
        ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &fast_conn_event_handler, NULL) );
        ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &fast_conn_event_handler, NULL) );
        ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &fast_conn_event_handler, NULL) );        
        start_fast_conn(nvs_handle);
    }
    else{
        ESP_LOGW(TAG_FAST_CONN, "Start the config net process");
        ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &smartconfig_event_handler, NULL) );
        ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &smartconfig_event_handler, NULL) );
        ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &smartconfig_event_handler, NULL) );
        
        ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
        ESP_ERROR_CHECK( esp_wifi_start() );
    }
    if(nvs_handle){
        nvs_close(nvs_handle);
        nvs_handle = 0;
    }
}

int start_smartconfig_monitor()
{
    return 0;
}

static void smartconfig_task(void * parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

void smartconfig_run(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // ESP_LOGW(TAG, "force confingnet func = %p", force_configNet);
    smartconfigKey_init((key_evt_cb_t)force_configNet);

    initialise_wifi();
}

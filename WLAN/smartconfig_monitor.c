
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "flexible_button.h"
#include "esp_log.h"

#include "smartconfig_monitor.h"

#define ENUM_TO_STR(e) (#e)

typedef enum{
    USER_BUTTON_0 = 0,
    USER_BUTTON_MAX
} user_button_t;

typedef struct __key_evt_st{
    uint8_t id;
    uint8_t evt;
    key_evt_cb_t cb;
}key_evt_t;

static const char *TAG = "smartconfig_moni";

char *enum_btn_event_string[] = {
    ENUM_TO_STR(BTN_DOWN),
    ENUM_TO_STR(BTN_CLICK),
    ENUM_TO_STR(BTN_DOUBLE_CLICK),
    ENUM_TO_STR(BTN_REPEAT_CLICK),
    ENUM_TO_STR(BTN_SHORT_START),
    ENUM_TO_STR(BTN_SHORT_UP),
    ENUM_TO_STR(BTN_LONG_START),
    ENUM_TO_STR(BTN_LONG_UP),
    ENUM_TO_STR(BTN_LONG_HOLD_START),
    ENUM_TO_STR(BTN_LONG_HOLD_UP),
    ENUM_TO_STR(BTN_MAX),
    ENUM_TO_STR(BTN_NONE),
};


QueueHandle_t Queue_key_evt_handle;
 
flex_button_t keyGrp[USER_BUTTON_MAX];

int key_scan_interval = 1000 / FLEX_BTN_SCAN_FREQ_HZ;
void key_scan_task(void *argument)
{
  for(;;)
  {
    flex_button_scan();
    // osDelay(key_scan_interval);  //注意，这里间隔时间要与FLEX_BTN_SCAN_FREQ_HZ紧密联系，详见 FLEX_BTN_SCAN_FREQ_HZ 解释
    vTaskDelay(key_scan_interval / portTICK_PERIOD_MS);
  }
}

void key_evt_process_task(void *argument)
{
    key_evt_t key_evt;

  for(;;)
  {
    memset((void *)&key_evt, 0, sizeof(key_evt));

	UBaseType_t msgCount = uxQueueMessagesWaiting(Queue_key_evt_handle);
	UBaseType_t freeSpace = uxQueueSpacesAvailable(Queue_key_evt_handle);

	BaseType_t result = xQueueReceive(Queue_key_evt_handle, &key_evt, pdMS_TO_TICKS(100));
	
	if(result != pdTRUE)
		continue;
	
	printf("msgCount: %d, msgFree: %d, id:%d, evt:%d\r\n", (uint16_t)msgCount, (uint16_t)freeSpace, key_evt.id, key_evt.evt);

    switch (key_evt.id)
    {
        case USER_BUTTON_0:
        {            
            switch (key_evt.evt)
            {
                case BTN_CLICK:{
                    printf("key click!\r\n");
                }break;

                case BTN_DOUBLE_CLICK:{
                    printf("double click!\r\n");
                }break;

                case BTN_REPEAT_CLICK:{
                    printf("repeat click!\r\n");
                }break;	

                case BTN_SHORT_UP:{
                    printf("press short!\r\n");
                }break;

                case BTN_LONG_START:{
                    printf("press long up!\r\n");
                    if(!key_evt.cb){
                        ESP_LOGE(TAG, "key_evt.cb is NULL!!!");
                    }
                    else{
                        key_evt.cb(NULL);
                    }    
                }break;

                case BTN_LONG_UP:{
                    printf("press long up!\r\n");                  
                }break;

                case BTN_LONG_HOLD_START:{
                    printf("long hold start!\r\n");
                }break;

                case BTN_LONG_HOLD_UP:{
                    printf("press long up!\r\n");
                }break;

                default:
                    break;
            }
        }
        break;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

static void vKeysEvtCb(void *arg)
{
    key_evt_t key_evt;

    flex_button_t *btn = (flex_button_t *)arg;
    ESP_LOGI(TAG, "button id:[%d]  event:[%d - %20s]  repeat:%d\r\n",
           btn->id, btn->event, enum_btn_event_string[btn->event], btn->click_cnt);

    key_evt.id = btn->id;
    key_evt.evt = btn->event;
    if(USER_BUTTON_0 == btn->id){
        // ESP_LOGW(TAG, "btn->cb_arg = %p", btn->cb_arg);
        if(key_evt.cb != (key_evt_cb_t)(btn->cb_arg)){
            key_evt.cb = (key_evt_cb_t)(btn->cb_arg);
        }
        // ESP_LOGW(TAG, "key_evt.cb = %p", key_evt.cb);
    }
    if(pdPASS != xQueueSendToBack(Queue_key_evt_handle, (void *)&key_evt, pdMS_TO_TICKS(100))){
        printf("send key evt vlaue to queue failed\r\n");
        return;
    }
}

#define SMARTCONFIG_KEY_NUM 25
int smartconfigkey_gpio_init(void)
{
    gpio_config_t io_conf = {};

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL << SMARTCONFIG_KEY_NUM);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;  //input-only pad has no internal PU
    // io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    return 0;
}

static uint8_t u8_smartconfigKeyRead(void *arg)
{
    return (uint8_t)(gpio_get_level(SMARTCONFIG_KEY_NUM));
}

void smartconfigKey_init(key_evt_cb_t force_confignet_cb)
{
    smartconfigkey_gpio_init();

    int i = 0;
    memset(keyGrp, 0, sizeof(keyGrp));
    for (i = 0; i < USER_BUTTON_MAX; i++)
    {
        keyGrp[i].id = i;
        keyGrp[i].usr_button_read = u8_smartconfigKeyRead;
        keyGrp[i].cb = vKeysEvtCb;
        if(USER_BUTTON_0 == i){
            keyGrp[i].cb_arg = (void *)force_confignet_cb;
        }
        keyGrp[i].pressed_logic_level = 0;
        keyGrp[i].debounce_tick = 20;
        keyGrp[i].max_multiple_clicks_interval = FLEX_MS_TO_SCAN_CNT(500); // 单击间隔500ms
        keyGrp[i].short_press_start_tick = FLEX_MS_TO_SCAN_CNT(1500);
        keyGrp[i].long_press_start_tick = FLEX_MS_TO_SCAN_CNT(3000);
        keyGrp[i].long_hold_start_tick = FLEX_MS_TO_SCAN_CNT(5000);
        flex_button_register(&keyGrp[i]);
    }

    Queue_key_evt_handle = xQueueCreate(5, sizeof(key_evt_t));

    //Key event sampling task
    xTaskCreate(key_scan_task, "key_scan", 4096, NULL, 3, NULL);

    //Key event process task
    xTaskCreate(key_evt_process_task, "key_evt_process", 2048, NULL, 3, NULL);

}


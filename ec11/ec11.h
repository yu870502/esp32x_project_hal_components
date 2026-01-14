#include "driver/pulse_cnt.h"

#include "freertos/queue.h"

typedef bool(* ec11_rotary_handle_t)(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *rotary_evt_queue);

typedef struct __ec11_obj_st{
    ec11_rotary_handle_t ec11_rotary_handle;
    QueueHandle_t ec11_evt_queue;
}ec11_t;

ec11_t *ec11_create(ec11_rotary_handle_t ec11_rotary_handle, QueueHandle_t ec11_evt_queue);
int ec11_destroy(ec11_t *ec11);
int ec11_init(ec11_t *ec11);

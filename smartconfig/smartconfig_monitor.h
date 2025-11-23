#ifndef __KEY_H__
#define __KEY_H__

typedef int(*key_evt_cb_t)(void *);

void smartconfigKey_init(key_evt_cb_t);

#endif


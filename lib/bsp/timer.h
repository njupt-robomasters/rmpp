#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void timer_delay(float seconds);
float timer_get_dt(uint32_t last_tick);
float timer_update_dt(uint32_t* last_tick);
#ifdef __cplusplus
}
#endif

#pragma once

#include <stdint.h>
#include <ch.h>

uint64_t uavcan_timesync_get_bus_time_at_systime(systime_t systime);
uint64_t uavcan_timesync_get_bus_time_now(void);
bool uavcan_timesync_get_systime_at_bus_time(uint64_t bustime, systime_t* systime_ret);

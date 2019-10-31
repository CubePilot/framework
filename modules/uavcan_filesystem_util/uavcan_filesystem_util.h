#pragma once

#include <stdint.h>
#include <ch.h>

bool uavcan_filesystem_read_timeout(uint8_t node_id, const char* path, uint64_t ofs, size_t* len, uint8_t* buf, systime_t timeout);

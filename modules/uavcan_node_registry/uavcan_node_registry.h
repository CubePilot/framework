#pragma once

#include <stdint.h>
#include <stdbool.h>

bool uavcan_node_registry_match_uuid_string(uint8_t uavcan_idx, uint8_t node_id, const char* str);

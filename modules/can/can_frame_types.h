#pragma once

#include <ch.h>
#include <stdbool.h>
#include <stdint.h>
#include <modules/pubsub/pubsub.h>

typedef uint32_t can_frame_priority_t;

struct can_frame_s {
    uint8_t RTR:1;
    uint8_t IDE:1;
    uint8_t DLC:4;
    union {
        uint32_t SID:11;
        uint32_t EID:29;
    };
    union {
        uint8_t data[8];
        uint32_t data32[2];
    };
};

enum can_frame_origin_t {
    CAN_FRAME_ORIGIN_CAN_BUS,
    CAN_FRAME_ORIGIN_LOCAL,
    CAN_FRAME_ORIGIN_BRIDGE
};

struct can_rx_frame_s {
    struct can_frame_s content;
    systime_t rx_systime;
    bool on_physical_bus : 1;
    enum can_frame_origin_t origin : 2;
};

struct can_tx_frame_s {
    struct can_frame_s content;
    systime_t creation_systime;
    systime_t tx_timeout;
    struct pubsub_topic_s* completion_topic;
#ifdef CAN_MODULE_ENABLE_BRIDGE_INTERFACE
    bool already_loopedback : 1;
    enum can_frame_origin_t origin : 2;
#endif
    struct can_tx_frame_s* next;
};

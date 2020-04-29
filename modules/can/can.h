#pragma once

#include "can_frame_types.h"
#include <modules/pubsub/pubsub.h>
#include <stdbool.h>
#include <stdint.h>
#include <ch.h>

struct can_instance_s;

struct can_transmit_completion_msg_s {
    systime_t completion_systime;
    bool transmit_success;
};

struct can_instance_s* can_get_instance(uint8_t can_idx);

bool can_iterate_instances(struct can_instance_s** instance_ptr);

void can_add_filter(struct can_instance_s* instance, uint32_t mask, uint32_t match);
void can_set_filtering_enabled(struct can_instance_s* instance, bool filtering_enabled);

void can_start_I(struct can_instance_s* instance, bool silent, bool auto_retransmit, uint32_t baudrate);
void can_start(struct can_instance_s* instance, bool silent, bool auto_retransmit, uint32_t baudrate);

void can_stop_I(struct can_instance_s* instance);
void can_stop(struct can_instance_s* instance);

struct pubsub_topic_s* can_get_rx_topic(struct can_instance_s* instance);

void can_set_silent_mode(struct can_instance_s* instance, bool silent);
void can_set_auto_retransmit_mode(struct can_instance_s* instance, bool auto_retransmit);
void can_set_baudrate(struct can_instance_s* instance, uint32_t baudrate);
uint32_t can_get_baudrate(struct can_instance_s* instance);
bool can_get_baudrate_confirmed(struct can_instance_s* instance);

struct can_tx_frame_s* can_allocate_tx_frame_and_append_I(struct can_instance_s* instance, struct can_tx_frame_s** frame_list);
struct can_tx_frame_s* can_allocate_tx_frame_and_append(struct can_instance_s* instance, struct can_tx_frame_s** frame_list);
struct can_tx_frame_s* can_allocate_tx_frames(struct can_instance_s* instance, size_t num_frames);
void can_enqueue_tx_frames(struct can_instance_s* instance, struct can_tx_frame_s** frame_list, systime_t tx_timeout, struct pubsub_topic_s* completion_topic, enum can_frame_origin_t origin);
void can_free_tx_frames(struct can_instance_s* instance, struct can_tx_frame_s** frame_list);

bool can_send_I(struct can_instance_s* instance, struct can_frame_s* frame, systime_t tx_timeout, struct pubsub_topic_s* completion_topic);
bool can_send(struct can_instance_s* instance, struct can_frame_s* frame, systime_t tx_timeout, struct pubsub_topic_s* completion_topic);

#ifdef CAN_MODULE_ENABLE_BRIDGE_INTERFACE
void can_bridge_transmit(struct can_instance_s* instance, const struct can_frame_s* frame);
#endif

#ifdef CAN_MODULE_ENABLE_BRIDGE_INTERFACE
#define CAN_RX_FRAME_ORIGIN(frame) ((frame).origin)
#else
#define CAN_RX_FRAME_ORIGIN(frame) CAN_FRAME_ORIGIN_CAN_BUS
#endif

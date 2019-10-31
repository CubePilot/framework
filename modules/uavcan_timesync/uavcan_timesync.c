#include <modules/uavcan/uavcan.h>
#include <common/ctor.h>
#include <modules/pubsub/pubsub.h>
#include <modules/worker_thread/worker_thread.h>
#include <uavcan.protocol.GlobalTimeSync.h>
#include <modules/can/can.h>
#include <modules/timing/timing.h>

#ifndef UAVCAN_TIMESYNC_WORKER_THREAD
#error Please define UAVCAN_TIMESYNC_WORKER_THREAD in framework_conf.h.
#endif

#define WT UAVCAN_TIMESYNC_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

// MODE_MASTER_INIT:
// Timesync messages are sent with previous_transmission_timestamp_usec = 0 on a 40ms interval and they fail to transmit after 1100ms
//  Transition to MODE_SLAVE occurs when any of the following conditions are met:
//     - A timesync message is received from a lower node id
//     - A timesync message is received with previous_transmission_timestamp_usec != 0
//         NOTE: This indicates there's an established master that we should sync to before taking over if we have a lower node id.
// Transition to MODE_MASTER occurs when the following condition is met:
//     - A timesync message is successfully transmitted and at least 2200ms has elapsed since we last failed to transmit
//         NOTE: This delay gives us time to find a pre-existing master that we should synchronize to
//
// MODE_MASTER:
// Timesync messages are sent with valid previous_transmission_timestamp_usec on a 40ms interval and they fail to transmit after 1100ms
// Transition to MODE_SLAVE occurs when the following condition is met:
//     - A timesync message is received from a lower node id with previous_transmission_timestamp_usec != 0
// Transition to MODE_MASTER_INIT occurs when the following condition is met:
//     - Transmission fails
//
// MODE_SLAVE:
// Received timesync messages are processed and a time offset is estimated.
// Transition to MODE_MASTER occurs when the following condition is met:
//     - Time synchronization has been achieved and self_node_id < master_node_id
// Transition to MODE_MASTER_INIT occurs when the following condition is met:
//     - 2200ms has elapsed without receiving a message from the master_node_id

enum timesync_state_t {
    MASTER_INIT,
    MASTER,
    SLAVE
};

static void on_timeout(struct worker_thread_timer_task_s* task);
static struct worker_thread_timer_task_s timer_task;

static struct worker_thread_listener_task_s timesync_listener;
static void timesync_message_handler(size_t msg_size, const void* buf, void* ctx);

static struct worker_thread_listener_task_s timesync_tx_completion_listener;
static void timesync_tx_completion_handler(size_t msg_size, const void* buf, void* ctx);

static struct pubsub_topic_s msg_completion_topic;

static enum timesync_state_t mode;
static uint64_t last_failed_transmit_us64; // set in MASTER, MASTER_INIT, used in MASTER_INIT - initialize to 0 when transitioning from SLAVE to MASTER_INIT
static uint64_t transmit_init_us64; // adequately protected, no need to initialize
static uint64_t last_transmit_time_us64; // set in MASTER_INIT, MASTER, used in MASTER - initialize to 0 when transitioning from SLAVE to MASTER
static uint8_t master_node_id; // used in SLAVE mode - initialize appropriately when transitioning to SLAVE mode
static bool have_valid_systime_offset; // set to false in MASTER_INIT, true in MASTER, false in SLAVE until there is a valid sync
static int64_t systime_offset; // defined as bus_time = micros64()+systime_offset. set in SLAVE mode, used in every mode - never initialize
static uint64_t prev_received_message_us64; // time of last message from master_node_id - initialize when entering SLAVE mode and when changing master_node_id

PUBSUB_TOPIC_GROUP_CREATE(msg_completion_topic_group, 64)

RUN_AFTER(UAVCAN_INIT) {
    pubsub_init_topic(&msg_completion_topic, &msg_completion_topic_group);
    worker_thread_add_timer_task(&WT, &timer_task, on_timeout, NULL, TIME_INFINITE, false);
    struct pubsub_topic_s* timesync_topic = uavcan_get_message_topic(0, &uavcan_protocol_GlobalTimeSync_descriptor);
    worker_thread_add_listener_task(&WT, &timesync_listener, timesync_topic, timesync_message_handler, NULL);
    worker_thread_add_listener_task(&WT, &timesync_tx_completion_listener, &msg_completion_topic, timesync_tx_completion_handler, NULL);

    last_failed_transmit_us64 = micros64();
    worker_thread_timer_task_reschedule(&WT, &timer_task, chTimeMS2I(UAVCAN_PROTOCOL_GLOBALTIMESYNC_MIN_BROADCASTING_PERIOD_MS));
}

static void on_timeout(struct worker_thread_timer_task_s* task) {
    (void)task;

    if (mode == MASTER_INIT) {
        struct uavcan_protocol_GlobalTimeSync_s msg;
        msg.previous_transmission_timestamp_usec = 0;
        if (uavcan_broadcast_with_callback(0, &uavcan_protocol_GlobalTimeSync_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &msg, chTimeMS2I(1100), &msg_completion_topic)) {
            transmit_init_us64 = micros64();
        } else {
            last_failed_transmit_us64 = micros64();
            worker_thread_timer_task_reschedule(&WT, &timer_task, chTimeMS2I(UAVCAN_PROTOCOL_GLOBALTIMESYNC_MIN_BROADCASTING_PERIOD_MS));
        }
    } else if (mode == MASTER) {
        struct uavcan_protocol_GlobalTimeSync_s msg;
        if (micros64()-last_transmit_time_us64 > 1100000) {
            msg.previous_transmission_timestamp_usec = 0;
        } else {
            msg.previous_transmission_timestamp_usec = last_transmit_time_us64;
        }
        last_transmit_time_us64 = 0;
        if (uavcan_broadcast_with_callback(0, &uavcan_protocol_GlobalTimeSync_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &msg, chTimeMS2I(1100), &msg_completion_topic)) {
            transmit_init_us64 = micros64();
        } else {
            last_failed_transmit_us64 = micros64();
            worker_thread_timer_task_reschedule(&WT, &timer_task, chTimeMS2I(UAVCAN_PROTOCOL_GLOBALTIMESYNC_MIN_BROADCASTING_PERIOD_MS));
        }
    } else if (mode == SLAVE) {
        // transition to MASTER_INIT
        have_valid_systime_offset = false;
        last_failed_transmit_us64 = 0;
        mode = MASTER_INIT;
    }
}

static void timesync_tx_completion_handler(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    uint64_t tnow_us64 = micros64();

    const struct can_transmit_completion_msg_s* status = buf;

    if (mode == MASTER_INIT) {
        if (status->transmit_success) {
            last_transmit_time_us64 = micros64_from_systime(status->completion_systime);
            if (last_failed_transmit_us64 != 0 && tnow_us64-last_failed_transmit_us64 > chTimeMS2I(2200)) {
                mode = MASTER;
                have_valid_systime_offset = true;
            }
        } else {
            last_failed_transmit_us64 = tnow_us64;
        }
    } else if (mode == MASTER) {
        if (status->transmit_success) {
            last_transmit_time_us64 = micros64_from_systime(status->completion_systime);
        } else {
            // transition to MASTER_INIT
            mode = MASTER_INIT;
            have_valid_systime_offset = false;
            last_failed_transmit_us64 = tnow_us64;
        }
    }

    if (mode == MASTER_INIT || mode == MASTER) {
        // Reschedule
        systime_t time_elapsed = chTimeUS2I(micros64_from_systime(status->completion_systime)-transmit_init_us64);
        if (time_elapsed >= chTimeMS2I(UAVCAN_PROTOCOL_GLOBALTIMESYNC_MIN_BROADCASTING_PERIOD_MS)) {
            worker_thread_timer_task_reschedule(&WT, &timer_task, TIME_IMMEDIATE);
        } else {
            worker_thread_timer_task_reschedule(&WT, &timer_task, chTimeMS2I(UAVCAN_PROTOCOL_GLOBALTIMESYNC_MIN_BROADCASTING_PERIOD_MS)-time_elapsed);
        }
    }

}

static void timesync_message_handler(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    const struct uavcan_deserialized_message_s* msg_wrapper = buf;
    const struct uavcan_protocol_GlobalTimeSync_s* msg = (const struct uavcan_protocol_GlobalTimeSync_s*)msg_wrapper->msg;

    uint8_t self_node_id = uavcan_get_node_id(0);
    if (mode == MASTER_INIT) {
        if (msg_wrapper->source_node_id < self_node_id || msg->previous_transmission_timestamp_usec != 0) {
            // transition to SLAVE
            mode = SLAVE;
            master_node_id = msg_wrapper->source_node_id;
        }
    } else if (mode == MASTER) {
        if (msg_wrapper->source_node_id < self_node_id && msg->previous_transmission_timestamp_usec != 0) {
            // transition to SLAVE
            mode = SLAVE;
            master_node_id = msg_wrapper->source_node_id;
        }
    } else if (mode == SLAVE) {
        if (msg_wrapper->source_node_id < master_node_id) {
            // reschedule timeout
            worker_thread_timer_task_reschedule(&WT, &timer_task, chTimeMS2I(2200));
            master_node_id = msg_wrapper->source_node_id;
        } else if (msg_wrapper->source_node_id == master_node_id) {
            // reschedule timeout
            worker_thread_timer_task_reschedule(&WT, &timer_task, chTimeMS2I(2200));
            // compute time offset
            if (prev_received_message_us64 != 0 && msg->previous_transmission_timestamp_usec != 0) {
                // TODO implement some kind of jitter filter and potentially scale factor estimation
                systime_offset = msg->previous_transmission_timestamp_usec - prev_received_message_us64;

                have_valid_systime_offset = true;

                if (self_node_id < master_node_id) {
                    // transition to MASTER
                    mode = MASTER;
                    last_transmit_time_us64 = 0;
                }
            }
        }
    }

    prev_received_message_us64 = micros64_from_systime(msg_wrapper->rx_timestamp);
}

uint64_t uavcan_timesync_get_bus_time_at_systime(systime_t systime) {
    if (have_valid_systime_offset) {
        return micros64_from_systime(systime)+systime_offset;
    } else {
        return 0;
    }
}

uint64_t uavcan_timesync_get_bus_time_now(void) {
    return uavcan_timesync_get_bus_time_at_systime(chVTGetSystemTimeX());
}

bool uavcan_timesync_get_systime_at_bus_time(uint64_t bustime, systime_t* systime_ret) {
    if (!have_valid_systime_offset) {
        return false;
    }

    *systime_ret = systime_from_micros64(bustime-systime_offset);
    return true;
}

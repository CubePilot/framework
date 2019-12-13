#include "uavcan_node_registry.h"
#include <common/helpers.h>
#include <modules/uavcan/uavcan.h>
#include <modules/uavcan_debug/uavcan_debug.h>
#include <modules/timing/timing.h>
#include <string.h>
#include <ctype.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.GetNodeInfo.h>

#include <modules/worker_thread/worker_thread.h>
#ifndef UAVCAN_NODE_REGISTRY_WORKER_THREAD
#error Please define UAVCAN_NODE_REGISTRY_WORKER_THREAD in framework_conf.h.
#endif

#define WT UAVCAN_NODE_REGISTRY_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

struct node_reg_entry_s {
    bool valid;
    uint32_t last_seen_us;
    uint32_t last_uptime_sec;
    uint8_t uuid[16];
};

struct node_reg_instance_s {
    uint8_t uavcan_idx;
    struct node_reg_entry_s entries[127]; // indexed by node_id-1 (since 0 is an invalid node id)
    struct worker_thread_listener_task_s nodestatus_listener_task;
    struct worker_thread_listener_task_s nodeinfo_res_listener_task;
    struct worker_thread_timer_task_s cleanup_task;
    struct node_reg_instance_s* next;
};

static void nodestatus_listener_task_func(size_t msg_size, const void* buf, void* ctx);
static void nodeinfo_res_listener_task_func(size_t msg_size, const void* buf, void* ctx);
static void cleanup_task_func(struct worker_thread_timer_task_s* task);

static struct node_reg_instance_s* node_reg_instance_list_head;

RUN_AFTER(UAVCAN_INIT) {
     for (uint8_t i=0; i<uavcan_get_num_instances(); i++) {
        struct node_reg_instance_s* instance = chCoreAlloc(sizeof(struct node_reg_instance_s));

        chDbgCheck(instance != NULL);
        if (!instance) {
            continue;
        }

        instance->uavcan_idx = i;
        memset(instance->entries, 0, sizeof(instance->entries));

        LINKED_LIST_APPEND(struct node_reg_instance_s, node_reg_instance_list_head, instance)

        struct pubsub_topic_s* nodestatus_topic = uavcan_get_message_topic(i, &uavcan_protocol_NodeStatus_descriptor);
        worker_thread_add_listener_task(&WT, &instance->nodestatus_listener_task, nodestatus_topic, nodestatus_listener_task_func, instance);

        struct pubsub_topic_s* nodeinfo_res_topic = uavcan_get_message_topic(i, &uavcan_protocol_GetNodeInfo_res_descriptor);
        worker_thread_add_listener_task(&WT, &instance->nodeinfo_res_listener_task, nodeinfo_res_topic, nodeinfo_res_listener_task_func, instance);

        worker_thread_add_timer_task(&WT, &instance->cleanup_task, cleanup_task_func, instance, chTimeS2I(1), true);
    }
}

static void cleanup_task_func(struct worker_thread_timer_task_s* task) {
    struct node_reg_instance_s* instance = worker_thread_task_get_user_context(task);

    uint32_t tnow_us = micros();
    for (size_t i=0; i < sizeof(instance->entries)/sizeof(instance->entries[0]); i++) {
        if (instance->entries[i].valid && tnow_us-instance->entries[i].last_seen_us > 4000000) {
            chSysLock();
            memset(&instance->entries[i], 0, sizeof(struct node_reg_entry_s));
            chSysUnlock();
        }
    }
}

static void nodestatus_listener_task_func(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    struct node_reg_instance_s* instance = ctx;

    const struct uavcan_deserialized_message_s* wrapper = buf;
    const struct uavcan_protocol_NodeStatus_s* msg = (const struct uavcan_protocol_NodeStatus_s*)wrapper->msg;

    // this should never happen
    if (wrapper->source_node_id <= 0 || wrapper->source_node_id > 127) {
        return;
    }

    struct node_reg_entry_s* entry = &instance->entries[wrapper->source_node_id-1];

    // Check if entry needs to be updated
    uint32_t tnow_us = micros();
    if (!entry->valid || tnow_us-entry->last_seen_us > 10000000 || msg->uptime_sec < entry->last_uptime_sec) {
        chSysLock();
        entry->valid = false;
        memset(entry, 0, sizeof(struct node_reg_entry_s));
        chSysUnlock();
        uavcan_request(instance->uavcan_idx, &uavcan_protocol_GetNodeInfo_req_descriptor, CANARD_TRANSFER_PRIORITY_LOW, wrapper->source_node_id, NULL);
    } else {
        chSysLock();
        entry->last_seen_us = tnow_us;
        entry->last_uptime_sec = msg->uptime_sec;
        chSysUnlock();
    }
}

static void nodeinfo_res_listener_task_func(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    struct node_reg_instance_s* instance = ctx;

    const struct uavcan_deserialized_message_s* wrapper = buf;
    const struct uavcan_protocol_GetNodeInfo_res_s* msg = (const struct uavcan_protocol_GetNodeInfo_res_s*)wrapper->msg;

    // this should never happen
    if (wrapper->source_node_id <= 0 || wrapper->source_node_id > 127) {
        return;
    }

    struct node_reg_entry_s* entry = &instance->entries[wrapper->source_node_id-1];

    uint32_t tnow_us = micros();
    chSysLock();
    memcpy(entry->uuid, msg->hardware_version.unique_id, 16);
    entry->last_seen_us = tnow_us;
    entry->last_uptime_sec = msg->status.uptime_sec;
    entry->valid = true;
    chSysUnlock();
}

static uint8_t hex_to_nibble(char c) {
    const char* hex_chars = "0123456789ABCDEF";
    const char* chrptr = strchr(hex_chars, ascii_toupper(c));
    if (!chrptr) {
        return 255;
    }
    return chrptr-hex_chars;
}

bool uavcan_node_registry_match_uuid_string(uint8_t uavcan_idx, uint8_t node_id, const char* str) {
    if (!node_reg_instance_list_head) {
        return false;
    }

    struct node_reg_instance_s* element = node_reg_instance_list_head;
    while (element != NULL && element->uavcan_idx != uavcan_idx) {
        element = element->next;
    }

    if (!element) {
        return false;
    }

    if (!element->entries[node_id-1].valid) {
        return false;
    }

    uint8_t uuid[16] = {};
    uint8_t characters_read = 0;
    size_t str_len = strnlen(str, 47);
    for (size_t i=0; i<str_len; i++) {
        uint8_t nibble = hex_to_nibble(str[i]);
        if (nibble == 255) {
            continue;
        }

        uuid[characters_read/2] |= nibble << (characters_read%2?0:4);
        characters_read++;
    }

    if (characters_read == 32 && !memcmp(uuid, element->entries[node_id-1].uuid, 16)) {
        return true;
    } else {
        return false;
    }
}

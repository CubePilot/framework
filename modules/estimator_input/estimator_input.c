#include "estimator_input.h"
#include <common/helpers.h>
#include <modules/timing/timing.h>
#include <stdlib.h>
#include <string.h>
#include <modules/uavcan_debug/uavcan_debug.h>


MEMORYPOOL_DECL(input_port_pool, sizeof(struct estimator_input_port_s), PORT_NATURAL_ALIGN, chCoreAllocAlignedI);

static void reschedule_S(struct estimator_input_s* instance) {
    systime_t tnow_ticks = chVTGetSystemTimeX();
    systime_t tdelay = TIME_INFINITE;

    if (instance->oldest_element) {
        uint32_t oldest_meas_age_us = chTimeI2US(tnow_ticks-instance->oldest_element->meas.meas_time);
        if (oldest_meas_age_us > instance->estimator_delay_us) {
            tdelay = TIME_IMMEDIATE;
        } else {
            tdelay = chTimeUS2I(instance->estimator_delay_us-oldest_meas_age_us);
        }
    }

    worker_thread_timer_task_reschedule_S(instance->worker_thread, &instance->process_measurements_task, tdelay);
}

static void process_measurements_task_func(struct worker_thread_timer_task_s* task) {
    struct estimator_input_s* instance = worker_thread_task_get_user_context(task);

    systime_t tnow_ticks = chVTGetSystemTimeX();

    if (!instance->oldest_element) {
        chSysLock();
        reschedule_S(instance);
        chSysUnlock();
        return;
    }

    systime_t oldest_meas_age = tnow_ticks-instance->oldest_element->meas.meas_time;

    if (oldest_meas_age < instance->estimator_delay_us) {
        chSysLock();
        reschedule_S(instance);
        chSysUnlock();
        return;
    }


    instance->cb(instance->oldest_element->port, &instance->oldest_element->meas, chHeapGetSize(instance->oldest_element)-sizeof(struct delay_buffer_element_s)+sizeof(struct sensor_measurement_s));

    chSysLock();
    instance->last_processed_msg_time = instance->oldest_element->meas.meas_time;
    struct delay_buffer_element_s* delete_ptr = instance->oldest_element;
//     instance->oldest_element->port->oldest_element = instance->oldest_element->next_in_port;
    instance->oldest_element = instance->oldest_element->next;
    reschedule_S(instance);
    chSysUnlock();
    chHeapFree(delete_ptr);

}

static bool delete_newest_lower_priority_message(struct estimator_input_s* instance, uint8_t priority) {
    (void)instance;
    (void)priority;
    // TODO
    return false;
}

static void insert_into_main_list_I(struct estimator_input_s* instance, struct delay_buffer_element_s* new_element) {
    struct delay_buffer_element_s** insert_ptr = &instance->oldest_element;
    systime_t tnow_ticks = chVTGetSystemTimeX();
    while (*insert_ptr && tnow_ticks-new_element->meas.meas_time < tnow_ticks-(*insert_ptr)->meas.meas_time) {
        insert_ptr = &((*insert_ptr)->next);
    }

    new_element->next = *insert_ptr;
    *insert_ptr = new_element;
}

static void sensor_msg_listener_func(size_t msg_size, const void* buf, void* ctx) {


    struct estimator_input_port_s* port = ctx;
    struct estimator_input_s* instance = port->instance;
    const struct sensor_measurement_s* meas = buf;

    systime_t tnow_ticks = chVTGetSystemTimeX();

    // Check if this measurement was taken prior to the current time horizon
    // TODO maybe allow for some non-chronological processing if the difference isn't too large
    if (tnow_ticks-meas->meas_time > tnow_ticks-instance->last_processed_msg_time) {
        uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "drop too old");
        port->drop_too_old++;
        return;
    }

//     uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "try allocate %u", (unsigned)(sizeof(struct delay_buffer_element_s)+msg_size-sizeof(struct sensor_measurement_s)));
    struct delay_buffer_element_s* new_element = chHeapAlloc(instance->heap, sizeof(struct delay_buffer_element_s)+msg_size-sizeof(struct sensor_measurement_s));

    while (!new_element) {
        if (!delete_newest_lower_priority_message(instance, port->priority)) {
            break;
        }

        new_element = chHeapAlloc(instance->heap, sizeof(struct delay_buffer_element_s)+msg_size-sizeof(struct sensor_measurement_s));
    }

    if (!new_element) {
        port->drop_alloc_fail++;
        uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "drop alloc fail");
        return;
    }

    new_element->port = port;
    memcpy(&new_element->meas, buf, msg_size);

    chSysLock();
    insert_into_main_list_I(instance, new_element);
//     insert_into_port_list_I(port, new_element);
    reschedule_S(instance);
    chSysUnlock();

}

static uint8_t count_ports(struct estimator_input_s* instance, enum sensor_type_t sensor_type) {
    uint8_t ret = 0;

    struct estimator_input_port_s* port = instance->input_ports_head;
    while (port) {
        if (port->sensor_type == sensor_type) {
            ret++;
        }
        port = port->next;
    }

    return ret;
}

static void try_init_listener(struct estimator_input_port_s* port) {
    if (port->listening_sensor_idx == port->sensor_idx) {
        return;
    }

    if (port->listening_sensor_idx != 255) {
        worker_thread_remove_listener_task(port->instance->worker_thread, &port->listener_task);
    }

    struct pubsub_topic_s* meas_topic = sensor_registry_get_measurement_topic(port->sensor_type, port->sensor_idx, port->measurement_type);
    if (!meas_topic) {
        return;
    }

    port->listening_sensor_idx = port->sensor_idx;
    worker_thread_add_listener_task(port->instance->worker_thread, &port->listener_task, meas_topic, sensor_msg_listener_func, port);
}

static void periodic_task_func(struct worker_thread_timer_task_s* task) {
    struct estimator_input_s* instance = worker_thread_task_get_user_context(task);

    struct estimator_input_port_s* port = instance->input_ports_head;
    while (port) {
        try_init_listener(port);
        port = port->next;
    }
}

void estimator_input_init(struct estimator_input_s* instance, const char* estimator_name, memory_heap_t* heap, uint32_t estimator_delay_us, struct worker_thread_s* worker_thread, estimator_measurement_cb_t cb) {
    memset(instance, 0, sizeof(*instance));

    instance->estimator_name = estimator_name;
    instance->estimator_delay_us = estimator_delay_us;
    instance->heap = heap;
    instance->worker_thread = worker_thread;
    instance->cb = cb;
    worker_thread_add_timer_task(instance->worker_thread, &instance->process_measurements_task, process_measurements_task_func, instance, TIME_INFINITE, false);
    worker_thread_add_timer_task(instance->worker_thread, &instance->periodic_task, periodic_task_func, instance, chTimeMS2I(100), true);
}

struct estimator_input_port_s* estimator_input_add_port(struct estimator_input_s* instance, enum sensor_type_t sensor_type, enum sensor_measurement_type_t measurement_type, uint8_t priority, float max_rate_hz, void* ctx) {
    uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "est_in_add_port %08X %u %u", (unsigned)instance, (unsigned)sensor_type, (unsigned)measurement_type);

    struct estimator_input_port_s* new_port = chPoolAlloc(&input_port_pool);

    if (!new_port) {
        return NULL;
    }

    memset(new_port, 0, sizeof(*new_port));

    new_port->instance = instance;
    new_port->sensor_type = sensor_type;
    new_port->priority = priority;
    new_port->measurement_type = measurement_type;
    new_port->max_rate_hz = max_rate_hz;
    new_port->listening_sensor_idx = 255;
    new_port->ctx = ctx;

    uint8_t port_number = count_ports(instance, sensor_type)+1;
    char port_number_str[4];
    itoa(port_number, port_number_str, 10);

    #define PARAM_NAME_APPEND(dest, str) strncat((dest), (str), 91-strnlen(dest, 92))
    char* idx_param_name = new_port->sensor_idx_param_name;
    PARAM_NAME_APPEND(idx_param_name, instance->estimator_name);
    PARAM_NAME_APPEND(idx_param_name, "_");
    PARAM_NAME_APPEND(idx_param_name, sensor_registry_get_sensor_type_name_str(sensor_type));
    PARAM_NAME_APPEND(idx_param_name, "_INPUT_");
    PARAM_NAME_APPEND(idx_param_name, port_number_str);
    PARAM_NAME_APPEND(idx_param_name, "_IDX");
    new_port->sensor_idx_param_desc = (struct param_descriptor_uint8_s){{PARAM_TYPE_UINT8, 0, new_port->sensor_idx_param_name, &new_port->sensor_idx}, 255, 0, 255};
    param_register((struct param_descriptor_header_s*)&new_port->sensor_idx_param_desc);

    LINKED_LIST_APPEND(struct estimator_input_port_s, instance->input_ports_head, new_port);

    try_init_listener(new_port);

    return new_port;
}

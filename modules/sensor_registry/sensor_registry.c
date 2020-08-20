#include "sensor_registry.h"
#include <common/helpers.h>
#include <ch.h>

static const size_t meas_type_sizes[] = _MEAS_TYPE_SIZES;
static const char* sensor_type_names[] = _SENSOR_TYPE_NAMES;

size_t sensor_registry_get_meas_type_size(enum sensor_measurement_type_t meas_type) {
    return meas_type_sizes[meas_type];
}

const char* sensor_registry_get_sensor_type_name_str(enum sensor_type_t sensor_type) {
    return sensor_type_names[sensor_type];
}

MEMORYPOOL_DECL(sensor_list_pool, sizeof(struct sensor_s), PORT_NATURAL_ALIGN, chCoreAllocAlignedI);
MEMORYPOOL_DECL(sensor_measurement_topic_pool, sizeof(struct sensor_measurement_topic_s), PORT_NATURAL_ALIGN, chCoreAllocAlignedI);

static struct sensor_s* sensor_list_head[NUM_SENSOR_TYPES];

static struct sensor_s* get_sensor(enum sensor_type_t type, uint8_t idx) {
    struct sensor_s* sens = sensor_list_head[type];
    while (sens) {
        if (sens->idx == idx) {
            return sens;
        }
        sens = sens->next;
    }

    return NULL;
}

static struct sensor_s* sensor_registry_register_I(enum sensor_type_t type, uint8_t idx) {
    struct sensor_s* sens = get_sensor(type, idx);
    if (sens != NULL) {
        return sens;
    }

    sens = chPoolAllocI(&sensor_list_pool);
    if (!sens) {
        return NULL;
    }

    sens->idx = idx;
    sens->topic_list_head = NULL;
    sens->next = NULL;

    LINKED_LIST_APPEND(struct sensor_s, sensor_list_head[type], sens);

    return sens;
}

static struct pubsub_topic_s* _get_measurement_topic(enum sensor_type_t sensor_type, uint8_t idx, enum sensor_measurement_type_t meas_type, bool insert_if_not_found) {
    struct sensor_s* sens = get_sensor(sensor_type, idx);

    if (!sens) {
        if (insert_if_not_found) {
            sens = sensor_registry_register_I(sensor_type, idx);
            if (!sens) {
                return NULL;
            }
        } else {
            return NULL;
        }
    }

    struct sensor_measurement_topic_s** insert_ptr = &sens->topic_list_head;

    while (*insert_ptr) {
        if ((*insert_ptr)->meas_type == meas_type) {
            return &(*insert_ptr)->topic;
        }
        insert_ptr = &(*insert_ptr)->next;
    }

    if (!insert_if_not_found) {
        return NULL;
    }

    struct sensor_measurement_topic_s* new_meas_topic = chPoolAllocI(&sensor_measurement_topic_pool);
    new_meas_topic->meas_type = meas_type;
    pubsub_init_topic(&new_meas_topic->topic, NULL);
    new_meas_topic->next = NULL;

    *insert_ptr = new_meas_topic;

    return &new_meas_topic->topic;
}

struct pubsub_topic_s* sensor_registry_get_measurement_topic(enum sensor_type_t sensor_type, uint8_t idx, enum sensor_measurement_type_t meas_type) {
    struct pubsub_topic_s* ret;
    chSysLock();
    ret = _get_measurement_topic(sensor_type, idx, meas_type, true);
    chSysUnlock();
    return ret;
}

struct sensor_writer_func_ctx_s {
    enum sensor_measurement_type_t meas_type;
    systime_t meas_time;
    pubsub_message_writer_func_ptr writer_func;
    void* ctx;
};

#include <modules/uavcan_debug/uavcan_debug.h>

static uint32_t msg_addr;
static uint32_t msg_type;
static uint32_t _msg_size;

static void sensor_writer_func(size_t msg_size, void* msg, void* ctx) {
    struct sensor_writer_func_ctx_s* ctx2 = ctx;
    struct sensor_measurement_s* meas = msg;
    meas->type = ctx2->meas_type;
    meas->meas_time = ctx2->meas_time;
    msg_type = meas->type;
    msg_addr = (uint32_t)msg;
    _msg_size = msg_size-offsetof(struct sensor_measurement_s, msg_body);
    ctx2->writer_func(msg_size-offsetof(struct sensor_measurement_s, msg_body), meas->msg_body, ctx2->ctx);
}

void sensor_registry_publish_sensor_message(enum sensor_type_t sensor_type, uint8_t idx, enum sensor_measurement_type_t meas_type, systime_t meas_time, pubsub_message_writer_func_ptr writer_func, void* ctx) {
    struct pubsub_topic_s* topic = sensor_registry_get_measurement_topic(sensor_type, idx, meas_type);
    if (!topic) {
        return;
    }

    struct sensor_writer_func_ctx_s ctx2 = {meas_type, meas_time, writer_func, ctx};
    pubsub_publish_message(topic, sensor_registry_get_meas_type_size(meas_type)+offsetof(struct sensor_measurement_s, msg_body), sensor_writer_func, &ctx2);
}

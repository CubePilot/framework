#include <modules/driver_profiLED/driver_profiLED.h>
#include <modules/worker_thread/worker_thread.h>
#include <common/ctor.h>
#include <common/helpers.h>
#include <modules/uavcan/uavcan.h>
#include <uavcan.equipment.indication.LightsCommand.h>
#include <hal.h>
#include <math.h>
#include <modules/timing/timing.h>
#include <modules/uavcan_timesync/uavcan_timesync.h>
#include <modules/param/param.h>

static struct spi_device_s led_spi;
static struct worker_thread_timer_task_s timer_task;
static void timer_task_func(struct worker_thread_timer_task_s* task);
struct worker_thread_listener_task_s led_command_task;
static void led_command_handler(size_t msg_size, const void* buf, void* ctx);

static uint8_t board_led_order[] = BOARD_LED_ORDER;

static struct profiLED_color_s led_off(uint32_t led_idx, void* ctx);
static struct profiLED_color_s led_uavcan(uint32_t led_idx, void* ctx);
static struct profiLED_color_s led_redgreen(uint32_t led_idx, void* ctx);
static struct profiLED_color_s led_red(uint32_t led_idx, void* ctx);
static struct profiLED_color_s led_green(uint32_t led_idx, void* ctx);
static struct profiLED_color_s led_rainbow(uint32_t led_idx, void* ctx);
static struct profiLED_color_s led_police(uint32_t led_idx, void* ctx);
static struct profiLED_color_s led_police_left(uint32_t led_idx, void* ctx);
static struct profiLED_color_s led_police_right(uint32_t led_idx, void* ctx);
static struct profiLED_color_s led_sequence(uint32_t led_idx, void* ctx);
static struct profiLED_color_s led_heading(uint32_t led_idx, void* ctx);

static profiLED_color_func led_mode_funcs[] = { led_off, led_uavcan, led_redgreen, led_red, led_green, led_rainbow, led_police, led_police_left, led_police_right, led_sequence, led_heading};
#define NUM_LED_MODES (sizeof(led_mode_funcs)/sizeof(led_mode_funcs[0]))

static const float led_angle_param = 0;
static const uint8_t led_power_mode_param = 1;
static const uint8_t led_mode_param = 5;
static const bool led_strobe = false;
// PARAM_DEFINE_FLOAT32_PARAM_STATIC(led_angle_param, "LED_ANGLE_OFS", 0, -2*M_PI_F, 2*M_PI_F)
// PARAM_DEFINE_UINT8_PARAM_STATIC(led_power_mode_param, "LED_POWER_MODE", 1, 0, 2)
// PARAM_DEFINE_UINT8_PARAM_STATIC(led_mode_param, "LED_MODE", 5, 0, NUM_LED_MODES-1)
// PARAM_DEFINE_BOOL_PARAM_STATIC(led_strobe, "LED_STROBE", false)

WORKER_THREAD_DECLARE_EXTERN(led_thread)

static struct profiLED_color_s uavcan_commanded_color;

RUN_ON(INIT_END) {
    if (profiLED_spi_dev_init(&led_spi, 5, 0, false, 100000)) {
        worker_thread_add_timer_task(&led_thread, &timer_task, timer_task_func, NULL, TIME_IMMEDIATE, false);
    }
    struct pubsub_topic_s* led_command_topic = uavcan_get_message_topic(0, &uavcan_equipment_indication_LightsCommand_descriptor);
    worker_thread_add_listener_task(&led_thread, &led_command_task, led_command_topic, led_command_handler, NULL);
}

static struct profiLED_color_s led_off(uint32_t led_idx, void* ctx) {
    led_idx = board_led_order[led_idx];
    uint64_t tnow_us = *(uint64_t*)ctx;

    struct profiLED_color_s ret;
    uint32_t period_us = 2000000;
    float t_sec = (float)(tnow_us%period_us) * 1e-6;
//     float t_rad = (float)(tnow_us%period_us)/period_us * 2*M_PI;

    ret.r = ret.g = ret.b = 0;

    if (led_strobe && (t_sec < 0.05 || (t_sec > 0.25 && t_sec < 0.3))) {
        ret.r = ret.g = ret.b = 255;
    }

    return ret;
}

static struct profiLED_color_s led_uavcan(uint32_t led_idx, void* ctx) {
    (void)led_idx;
    (void)ctx;
    return uavcan_commanded_color;
}

static struct profiLED_color_s led_rainbow(uint32_t led_idx, void* ctx) {
    led_idx = board_led_order[led_idx];

    uint64_t tnow_us = *(uint64_t*)ctx;
    float pos_rad = wrap_2pi(2*M_PI*(led_idx+0.5)/16.0-led_angle_param);

    struct profiLED_color_s ret;
    uint32_t period_us = 1000000;
    float t_rad = (float)(tnow_us%period_us)/period_us * 2*M_PI;

    ret.r = 25*(1+sinf(t_rad+pos_rad))/2;
    ret.g = 25*(1+sinf(t_rad+pos_rad+2*M_PI/3))/2;
    ret.b = 25*(1+sinf(t_rad+pos_rad+4*M_PI/3))/2;

    return ret;
}

static struct profiLED_color_s led_redgreen(uint32_t led_idx, void* ctx) {
    uint64_t tnow_us = *(uint64_t*)ctx;
    led_idx = board_led_order[led_idx];

    float pos_rad = wrap_2pi(2*M_PI*(led_idx+0.5)/16.0-led_angle_param);

    struct profiLED_color_s ret;
    uint32_t period_us = 2000000;
    float t_sec = (float)(tnow_us%period_us) * 1e-6;
//     float t_rad = (float)(tnow_us%period_us)/period_us * 2*M_PI;

    ret.r = ret.g = ret.b = 0;

    if (led_strobe && (t_sec < 0.05 || (t_sec > 0.25 && t_sec < 0.3))) {
        ret.r = ret.g = ret.b = 255;
    } else if (pos_rad < M_PI) {
        ret.g = 50;
    } else {
        ret.r = 50;
    }

    return ret;
}

static struct profiLED_color_s led_red(uint32_t led_idx, void* ctx) {
    led_idx = board_led_order[led_idx];
    uint64_t tnow_us = *(uint64_t*)ctx;
//     float pos_rad = wrap_2pi(2*M_PI*(led_idx+0.5)/16.0-led_angle_param);

    struct profiLED_color_s ret;
    uint32_t period_us = 2000000;
    float t_sec = (float)(tnow_us%period_us) * 1e-6;
//     float t_rad = (float)(tnow_us%period_us)/period_us * 2*M_PI;

    ret.g = ret.b = 0;
    ret.r = 50;

    if (led_strobe && (t_sec < 0.05 || (t_sec > 0.25 && t_sec < 0.3))) {
        ret.r = ret.g = ret.b = 255;
    }

    return ret;
}

static struct profiLED_color_s led_green(uint32_t led_idx, void* ctx) {
    led_idx = board_led_order[led_idx];
    uint64_t tnow_us = *(uint64_t*)ctx;

    struct profiLED_color_s ret;
    uint32_t period_us = 2000000;
    float t_sec = (float)(tnow_us%period_us) * 1e-6;
//     float t_rad = (float)(tnow_us%period_us)/period_us * 2*M_PI;

    ret.r = ret.b = 0;
    ret.g = 50;

    if (led_strobe && (t_sec < 0.05 || (t_sec > 0.25 && t_sec < 0.3))) {
        ret.r = ret.g = ret.b = 255;
    }

    return ret;
}

static struct profiLED_color_s led_police(uint32_t led_idx, void* ctx) {
    led_idx = board_led_order[led_idx];

    uint64_t tnow_us = *(uint64_t*)ctx;
    float pos_rad = wrap_2pi(2*M_PI*(led_idx+0.5)/16.0-led_angle_param);

    struct profiLED_color_s ret;
    uint32_t period_us = 1000000;

    uint8_t step = 12*(tnow_us%period_us)/period_us;

    ret.r = ret.g = ret.b = 0;

    if (pos_rad < M_PI) {
        switch(step) {
            case 0:
            case 2:
            case 4:
                ret.b = 255;
                break;
            case 6:
            case 8:
            case 10:
                ret.r = ret.b = ret.g = 255;
                break;
        }
    } else {
        switch(step) {
            case 0:
            case 2:
            case 4:
                ret.r = ret.b = ret.g = 255;
                break;
            case 6:
            case 8:
            case 10:
                ret.r = 255;
                break;
        }
    }

    return ret;
}

static struct profiLED_color_s led_police_left(uint32_t led_idx, void* ctx) {
    led_idx = board_led_order[led_idx];

    uint64_t tnow_us = *(uint64_t*)ctx;
    float pos_rad = wrap_2pi(2*M_PI*(led_idx+0.5)/16.0-led_angle_param);

    struct profiLED_color_s ret;
    uint32_t period_us = 1000000;

    uint8_t step = 12*(tnow_us%period_us)/period_us;

    ret.r = ret.g = ret.b = 0;

    switch(step) {
        case 0:
        case 2:
        case 4:
            ret.r = ret.b = ret.g = 255;
            break;
        case 6:
        case 8:
        case 10:
            ret.r = 255;
            break;
    }

    return ret;
}

static struct profiLED_color_s led_police_right(uint32_t led_idx, void* ctx) {
    led_idx = board_led_order[led_idx];

    uint64_t tnow_us = *(uint64_t*)ctx;
    float pos_rad = wrap_2pi(2*M_PI*(led_idx+0.5)/16.0-led_angle_param);

    struct profiLED_color_s ret;
    uint32_t period_us = 1000000;

    uint8_t step = 12*(tnow_us%period_us)/period_us;

    ret.r = ret.g = ret.b = 0;

    switch(step) {
        case 0:
        case 2:
        case 4:
            ret.b = 255;
            break;
        case 6:
        case 8:
        case 10:
            ret.r = ret.b = ret.g = 255;
            break;
    }

    return ret;
}

static struct profiLED_color_s led_sequence(uint32_t led_idx, void* ctx) {
    led_idx = board_led_order[led_idx];

    uint64_t tnow_us = *(uint64_t*)ctx;

    struct profiLED_color_s ret;
    ret.r = ret.g = ret.b = 0;

    uint32_t period_us = 20000000;

    uint8_t step = 20*(tnow_us%period_us)/period_us;
    if (led_idx == step) {
        ret.r = ret.g = ret.b = 255;
    }
    return ret;
}


static struct profiLED_color_s led_heading(uint32_t led_idx, void* ctx) {
//     led_idx = board_led_order[led_idx];
//
//     uint64_t tnow_us = *(uint64_t*)ctx;
//
//     uint32_t period_us = 60000000;
//
//     float pos_rad = wrap_2pi(2*M_PI*(led_idx+0.5)/16.0-led_angle_param);
//
//     if (!heading_valid) {
//         struct profiLED_color_s ret;
//         ret.b = ret.g = 0;
//         ret.r = 10;
//         return ret;
//     }
//
//     float heading = ((tnow_us%period_us) / (float)period_us)*2*M_PI;
//
//     float dist = fabsf(wrap_pi(-heading-pos_rad))/M_PI;
//
//     float intensity = 1-dist*8;
//     if (intensity < 0) {
//         intensity = 0;
//     }
//
    struct profiLED_color_s ret;
    ret.r = ret.g = ret.b = 0;
    return ret;
}

static void timer_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;

    switch(led_power_mode_param) {
        case 1:
            palSetLine(BOARD_PAL_LINE_LED_ENABLE_1);
            palClearLine(BOARD_PAL_LINE_LED_ENABLE_2);
            break;
        case 2:
            palSetLine(BOARD_PAL_LINE_LED_ENABLE_1);
            palSetLine(BOARD_PAL_LINE_LED_ENABLE_2);
            break;
        default:
        case 0:
            palClearLine(BOARD_PAL_LINE_LED_ENABLE_1);
            palClearLine(BOARD_PAL_LINE_LED_ENABLE_2);
            break;
    }

    uint64_t tnow_us = uavcan_timesync_get_bus_time_now();
    if (tnow_us == 0) {
        tnow_us = micros64();
    }

    uint32_t t_begin_us = micros();

    uint8_t led_mode = led_mode_param;

    if (led_mode >= NUM_LED_MODES) {
        led_mode = 0;
    }

    profiLED_output_spi(&led_spi, 16, led_mode_funcs[led_mode], &tnow_us);

    int32_t time_error_us = (int32_t)(tnow_us%16384) - 16384/2;
    uint32_t time_correction = (micros()-t_begin_us) + time_error_us;

    if (time_correction > 16384) {
        time_correction = 16384;
    }

    worker_thread_timer_task_reschedule(&led_thread, &timer_task, chTimeUS2I(16384-time_correction));
}

static void led_command_handler(size_t msg_size, const void* buf, void* ctx)
{
    (void)msg_size;
    (void)ctx;
    const struct uavcan_deserialized_message_s* msg_wrapper = buf;
    const struct uavcan_equipment_indication_LightsCommand_s* msg = (const struct uavcan_equipment_indication_LightsCommand_s*)msg_wrapper->msg;
    uavcan_commanded_color.r = msg->commands[0].color.red*8;
    uavcan_commanded_color.g = msg->commands[0].color.green*4;
    uavcan_commanded_color.b = msg->commands[0].color.blue*8;
}

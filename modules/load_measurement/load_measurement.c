/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <common/ctor.h>
#include <ch.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/uavcan_debug/uavcan_debug.h>

#ifndef LOAD_MEASUREMENT_WORKER_THREAD
#error Please define LOAD_MEASUREMENT_WORKER_THREAD in framework_conf.h.
#endif

#define WT LOAD_MEASUREMENT_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct worker_thread_timer_task_s load_print_task;
static void load_print_task_func(struct worker_thread_timer_task_s* task);

systime_t meas_begin_t;
systime_t idle_enter_t;
systime_t idle_total_ticks;

RUN_AFTER(WORKER_THREADS_INIT) {
    meas_begin_t = chVTGetSystemTimeX();
    idle_total_ticks = 0;
    worker_thread_add_timer_task(&WT, &load_print_task, load_print_task_func, NULL, chTimeMS2I(5000), true);
}

static void load_print_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;

    systime_t tnow = chVTGetSystemTimeX();
    uint32_t load = (uint64_t)10000*idle_total_ticks/(tnow-meas_begin_t);

    uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "load", "idle %u.%02u%% (%u/%u)", load/100, load%100, idle_total_ticks, tnow-meas_begin_t);

    meas_begin_t = tnow;
    idle_total_ticks = 0;
}

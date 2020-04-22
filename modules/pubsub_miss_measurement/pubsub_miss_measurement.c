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

#ifndef PUBSUB_MISS_MEASUREMENT_WORKER_THREAD
#error Please define PUBSUB_MISS_MEASUREMENT_WORKER_THREAD in framework_conf.h.
#endif

#define WT PUBSUB_MISS_MEASUREMENT_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct worker_thread_timer_task_s miss_print_task;
static void miss_print_task_func(struct worker_thread_timer_task_s* task);

RUN_AFTER(WORKER_THREADS_INIT) {
    worker_thread_add_timer_task(&WT, &miss_print_task, miss_print_task_func, NULL, LL_MS2ST(5000), true);
}

static void miss_print_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;

    uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "miss", "%u", pubsub_get_cumulative_misses());
}

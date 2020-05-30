#pragma once

#include <modules/uSD/uSD.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/pubsub/pubsub.h>


struct log_writer_s {
    struct logfile_s* logfile;
    struct worker_thread_listener_task_s listener_task;
    struct log_writer_s* next;
};

struct logfile_s {
    struct log_writer_s* writer_list_head;
    FIL fp;
    FRESULT open_res;
};

void logger_start_log(struct logfile_s* logfile, const char* name);
void logger_subscribe(struct logfile_s* logfile, struct pubsub_topic_s* topic);

#include <modules/worker_thread/worker_thread.h>
#include <modules/uSD/uSD.h>

#ifndef UAVCAN_FILE_SERVER_WORKER_THREAD
#error Please define UAVCAN_FILE_SERVER_WORKER_THREAD in framework_conf.h.
#endif

#define WT UAVCAN_FILE_SERVER_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

#include <string.h>

#include <uavcan.protocol.file.Read.h>

static struct worker_thread_listener_task_s getdirentryinfo_req_listener_task;
static void getdirentryinfo_req_listener_task_func(size_t msg_size, const void* buf, void* ctx);

RUN_AFTER(UAVCAN_INIT) {
    struct pubsub_topic_s* getdirentryinfo_req_topic = uavcan_get_message_topic(0, &uavcan_protocol_file_Read_req_descriptor);
    worker_thread_add_listener_task(&WT, &getdirentryinfo_req_listener_task, getdirentryinfo_req_topic, getdirentryinfo_req_listener_task_func, NULL);
}

static void getdirentryinfo_req_listener_task_func(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;

    const struct uavcan_deserialized_message_s* msg_wrapper = buf;
    const struct uavcan_protocol_file_Read_req_s* req = (struct uavcan_protocol_file_Read_req_s*)msg_wrapper->msg;

    struct uavcan_protocol_file_Read_res_s res;
    memset(&res, 0, sizeof(res));

    FATFS* fs = uSD_get_filesystem();

    if (!fs) {
        res.error.value = UAVCAN_PROTOCOL_FILE_ERROR_UNKNOWN_ERROR;
        uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
        return;
    }

    FIL f;
    char path[sizeof(req->path.path)];

    memcpy(path, req->path.path, req->path.path_len);

    path[req->path.path_len] = '\0';

    if (f_open(&f, path, FA_READ) != FR_OK) {
        res.error.value = UAVCAN_PROTOCOL_FILE_ERROR_UNKNOWN_ERROR;
        uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
        return;
    }

    if(f_lseek(&f, req->offset) != FR_OK) {
        f_close(&f);
        res.error.value = UAVCAN_PROTOCOL_FILE_ERROR_UNKNOWN_ERROR;
        uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
        return;
    }

    UINT br;
    FRESULT fres = f_read(&f, res.data, sizeof(res.data), &br);
    res.data_len = br;
    if (fres != FR_OK) {
        res.error.value = UAVCAN_PROTOCOL_FILE_ERROR_UNKNOWN_ERROR;
    }

    f_close(&f);
    uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
}

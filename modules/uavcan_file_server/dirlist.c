#include <modules/worker_thread/worker_thread.h>
#include <modules/uSD/uSD.h>

#ifndef UAVCAN_FILE_SERVER_WORKER_THREAD
#error Please define UAVCAN_FILE_SERVER_WORKER_THREAD in framework_conf.h.
#endif

#define WT UAVCAN_FILE_SERVER_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

#include <string.h>

#include <uavcan.protocol.file.GetDirectoryEntryInfo.h>

static struct worker_thread_listener_task_s getdirentryinfo_req_listener_task;
static void getdirentryinfo_req_listener_task_func(size_t msg_size, const void* buf, void* ctx);

RUN_AFTER(UAVCAN_INIT) {
    struct pubsub_topic_s* getdirentryinfo_req_topic = uavcan_get_message_topic(0, &uavcan_protocol_file_GetDirectoryEntryInfo_req_descriptor);
    worker_thread_add_listener_task(&WT, &getdirentryinfo_req_listener_task, getdirentryinfo_req_topic, getdirentryinfo_req_listener_task_func, NULL);
}

static void getdirentryinfo_req_listener_task_func(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;

    const struct uavcan_deserialized_message_s* msg_wrapper = buf;
    const struct uavcan_protocol_file_GetDirectoryEntryInfo_req_s* req = (struct uavcan_protocol_file_GetDirectoryEntryInfo_req_s*)msg_wrapper->msg;

    struct uavcan_protocol_file_GetDirectoryEntryInfo_res_s res;
    memset(&res, 0, sizeof(res));

    FATFS* fs = uSD_get_filesystem();

    if (!fs) {
        res.error.value = UAVCAN_PROTOCOL_FILE_ERROR_UNKNOWN_ERROR;
        uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
        return;
    }

    DIR dir;
    char path[201];

    memcpy(path, req->directory_path.path, req->directory_path.path_len);

    path[req->directory_path.path_len] = '\0';

    if (f_opendir(&dir, path) != FR_OK) {
        res.error.value = UAVCAN_PROTOCOL_FILE_ERROR_UNKNOWN_ERROR;
        uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
        return;
    }

    FILINFO finfo;

    for (size_t i=0; i <= req->entry_index; i++) {
        if (f_readdir(&dir, &finfo) != FR_OK) {
            f_closedir(&dir);
            res.error.value = UAVCAN_PROTOCOL_FILE_ERROR_UNKNOWN_ERROR;
            uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
            return;
        }

        if (finfo.fname[0] == 0) {
            f_closedir(&dir);
            res.error.value = UAVCAN_PROTOCOL_FILE_ERROR_NOT_FOUND;
            uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
            return;
        }
    }
    f_closedir(&dir);

    if ((finfo.fattrib & AM_DIR) != 0) {
        res.entry_type.flags = UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_DIRECTORY;
    } else {
        res.entry_type.flags = UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE;
    }

    // construct the full path of the entry
    size_t max_path_len = sizeof(res.entry_full_path.path);
    strncpy((char*)res.entry_full_path.path, path, max_path_len);
    strncat((char*)res.entry_full_path.path, "/", max_path_len-strnlen((char*)res.entry_full_path.path, max_path_len));
    strncat((char*)res.entry_full_path.path, finfo.fname, max_path_len-strnlen((char*)res.entry_full_path.path, max_path_len));
    res.entry_full_path.path_len = strnlen((char*)res.entry_full_path.path, max_path_len);

    uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
}

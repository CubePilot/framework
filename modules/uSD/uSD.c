#include "uSD.h"
#include <common/ctor.h>
#include <hal.h>

static FATFS filesystem;
static bool filesystem_ok;

static bool try_reformat(void) {
    FRESULT res;
    uint8_t workingbuf[1024];
    res = f_mkfs("/", FM_EXFAT, 0, workingbuf, sizeof(workingbuf));
    return res == FR_OK;
}

RUN_AFTER(CH_SYS_INIT) {
    sdcStart(&SDCD1, NULL);
    if (sdcConnect(&SDCD1) != HAL_SUCCESS) {
        return;
    }

    FRESULT res;

    for (uint8_t i=0; i<10; i++) {
        res = f_mount(&filesystem, "/", 1);
        if (res == FR_OK) {
            break;
        }
        chThdSleep(LL_MS2ST(10));
    }

#ifdef MICROSD_MOUNT_FAIL_REFORMAT
    if (res != FR_OK) {
        if (!try_reformat()) {
            return;
        }

        res = f_mount(&filesystem, "/", 1);
        if (res != FR_OK) {
            return;
        }
    }
#else
    if (res != FR_OK) {
        return;
    }
#endif

    filesystem_ok = true;
}


FATFS* uSD_get_filesystem(void) {
    if (!filesystem_ok) {
        return NULL;
    }

    return &filesystem;
}

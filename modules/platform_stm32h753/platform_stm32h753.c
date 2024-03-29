#include "platform_stm32h753.h"
#include <hal.h>
#include <string.h>

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void) {
    SCB_DisableDCache();
    stm32_clock_init();
}

void board_get_unique_id(uint8_t* buf, uint8_t len) {
    uint32_t unique_id_uint32[3];
    unique_id_uint32[0] = ((uint32_t*)0x1FF1E800)[2];
    unique_id_uint32[1] = ((uint32_t*)0x1FF1E800)[1];
    unique_id_uint32[2] = ((uint32_t*)0x1FF1E800)[0];

    if (len>12) {
        memset(buf, 0, len);
        memcpy(buf, unique_id_uint32, 12);
    } else {
        memcpy(buf, unique_id_uint32, len);
    }
}

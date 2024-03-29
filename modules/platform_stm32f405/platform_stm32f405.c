#include "platform_stm32f405.h"
#include <hal.h>
#include <string.h>

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void) {
    stm32_clock_init();
    rccResetAHB1(STM32_GPIO_EN_MASK);
    rccEnableAHB1(STM32_GPIO_EN_MASK, true);
}

void board_get_unique_id(uint8_t* buf, uint8_t len) {
    uint32_t unique_id_uint32[3];
    unique_id_uint32[0] = ((uint32_t*)0x1FFF7A10)[2];
    unique_id_uint32[1] = ((uint32_t*)0x1FFF7A10)[1];
    unique_id_uint32[2] = ((uint32_t*)0x1FFF7A10)[0];

    if (len>12) {
        memset(buf, 0, len);
        memcpy(buf, unique_id_uint32, 12);
    } else {
        memcpy(buf, unique_id_uint32, len);
    }
}

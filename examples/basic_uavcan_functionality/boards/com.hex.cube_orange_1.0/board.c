#include <hal.h>

void boardInit(void) {
    rccResetAHB4(STM32_GPIO_EN_MASK);
    rccEnableAHB4(STM32_GPIO_EN_MASK, true);
    palSetLineMode(BOARD_PAL_LINE_CAN_RX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_MID2);
    palSetLineMode(BOARD_PAL_LINE_CAN_TX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_MID2);
}

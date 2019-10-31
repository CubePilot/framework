#pragma once

#include <modules/platform_stm32h743/platform_stm32h743.h>

#define BOARD_FLASH_SIZE 2048

#define BOARD_CONFIG_HW_NAME "com.hex.cube_orange"
#define BOARD_CONFIG_HW_MAJOR_VER 1
#define BOARD_CONFIG_HW_MINOR_VER 0

#define BOARD_CONFIG_HW_INFO_STRUCTURE { \
    .hw_name = BOARD_CONFIG_HW_NAME, \
    .hw_major_version = BOARD_CONFIG_HW_MAJOR_VER, \
    .hw_minor_version = BOARD_CONFIG_HW_MINOR_VER, \
    .board_desc_fmt = SHARED_HW_INFO_BOARD_DESC_FMT_NONE, \
    .board_desc = 0, \
}

#define BOARD_PAL_LINE_CAN_RX PAL_LINE(GPIOD,0)
#define BOARD_PAL_LINE_CAN_TX PAL_LINE(GPIOD,1)

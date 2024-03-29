#pragma once

#include <modules/platform_stm32f767/platform_stm32f767.h>

#define BOARD_CONFIG_HW_NAME "com.hex.here_pro"
#define BOARD_CONFIG_HW_MAJOR_VER 1
#define BOARD_CONFIG_HW_MINOR_VER 0

#define BOARD_CONFIG_HW_INFO_STRUCTURE { \
    .hw_name = BOARD_CONFIG_HW_NAME, \
    .hw_major_version = BOARD_CONFIG_HW_MAJOR_VER, \
    .hw_minor_version = BOARD_CONFIG_HW_MINOR_VER, \
    .board_desc_fmt = SHARED_HW_INFO_BOARD_DESC_FMT_NONE, \
    .board_desc = 0, \
}

#define BOARD_PAL_LINE_USART3_TX PAL_LINE(GPIOD, 8)
#define BOARD_PAL_LINE_USART3_RX PAL_LINE(GPIOD, 9)

#define BOARD_PAL_LINE_SDMMC_D0 PAL_LINE(GPIOC, 8)
#define BOARD_PAL_LINE_SDMMC_D1 PAL_LINE(GPIOC, 9)
#define BOARD_PAL_LINE_SDMMC_D2 PAL_LINE(GPIOC, 10)
#define BOARD_PAL_LINE_SDMMC_D3 PAL_LINE(GPIOC, 11)
#define BOARD_PAL_LINE_SDMMC_CK PAL_LINE(GPIOC, 12)
#define BOARD_PAL_LINE_SDMMC_CMD PAL_LINE(GPIOD, 2)

#define BOARD_PAL_LINE_SPI5_SCK PAL_LINE(GPIOF, 7)
#define BOARD_PAL_LINE_SPI5_MISO PAL_LINE(GPIOF, 8)
#define BOARD_PAL_LINE_SPI5_MOSI PAL_LINE(GPIOF, 9)

#define BOARD_PAL_LINE_ICM_CS PAL_LINE(GPIOA, 4)
#define BOARD_PAL_LINE_SPI1_SCK PAL_LINE(GPIOA, 5)
#define BOARD_PAL_LINE_SPI1_MISO PAL_LINE(GPIOA, 6)
#define BOARD_PAL_LINE_SPI1_MOSI PAL_LINE(GPIOA, 7)

#define BOARD_PAL_LINE_UBX_NRESET PAL_LINE(GPIOD, 11)
#define BOARD_PAL_LINE_UBX_NSAFEBOOT PAL_LINE(GPIOD, 13)

#define BOARD_PAL_LINE_CAN1_SLEEP PAL_LINE(GPIOC,13)
#define BOARD_PAL_LINE_CAN2_SLEEP PAL_LINE(GPIOG,11)

#define BOARD_PAL_LINE_CAN1_TERMINATE PAL_LINE(GPIOC,14)
#define BOARD_PAL_LINE_CAN2_TERMINATE PAL_LINE(GPIOB,3)

#define BOARD_PAL_LINE_CAN1_RX PAL_LINE(GPIOB,8)
#define BOARD_PAL_LINE_CAN1_TX PAL_LINE(GPIOB,9)
#define BOARD_FLASH_SIZE 2048

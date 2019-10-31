#include <modules/param/param.h>
#include <common/ctor.h>
#include <hal.h>

#ifdef BOARD_PAL_LINE_CAN1_TERMINATE
PARAM_DEFINE_BOOL_PARAM_STATIC(can1_terminator, "CAN1_TERMINATOR", false)
#endif

#ifdef BOARD_PAL_LINE_CAN2_TERMINATE
PARAM_DEFINE_BOOL_PARAM_STATIC(can2_terminator, "CAN2_TERMINATOR", false)
#endif

RUN_ON(INIT_END) {
#ifdef BOARD_PAL_LINE_CAN1_TERMINATE
    palWriteLine(BOARD_PAL_LINE_CAN1_TERMINATE, can1_terminator);
#endif

#ifdef BOARD_PAL_LINE_CAN2_TERMINATE
    palWriteLine(BOARD_PAL_LINE_CAN2_TERMINATE, can2_terminator);
#endif
}

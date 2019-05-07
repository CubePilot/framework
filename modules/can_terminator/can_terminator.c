#include <modules/param/param.h>
#include <common/ctor.h>
#include <hal.h>

PARAM_DEFINE_BOOL_PARAM_STATIC(can1_terminator, "CAN1_TERMINATOR", false)

RUN_ON(INIT_END) {
    palWriteLine(BOARD_PAL_LINE_CAN1_TERMINATE, can1_terminator);
}

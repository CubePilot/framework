MEMORY
{
    bl_lma (rx) :         ORIGIN = 0x08000000,            LENGTH = 32K
    bl (rx) :             ORIGIN = 0x00200000,            LENGTH = 32K
    param1 (rx) :         ORIGIN = 0x08000000+32K,        LENGTH = 32K
    param2 (rx) :         ORIGIN = 0x08000000+64K,        LENGTH = 32K

    app_lma (rx) :        ORIGIN = 0x08000000+96K,        LENGTH = 2M-96K
    app (rx) :            ORIGIN = 0x00200000+96K,        LENGTH = 2M-96K

    ram0    : org = 0x20020000, len = 384k-256      /* SRAM1 + SRAM2 */
    ram1    : org = 0x20020000, len = 368k      /* SRAM1 */
    ram2    : org = 0x2007C000, len = 16k   /* SRAM2 */
    ram3    : org = 0x20000000, len = 128k      /* DTCM-RAM */
    ram4    : org = 0x00000000, len = 16k       /* ITCM-RAM */
    ram5    : org = 0x40024000, len = 4k-256        /* BCKP SRAM */
    ram6    : org = 0x00000000, len = 0
    ram7    : org = 0x00000000, len = 0
    app_bl_shared (rwx) : ORIGIN = 0x20020000+(384k-256), LENGTH = 256

}

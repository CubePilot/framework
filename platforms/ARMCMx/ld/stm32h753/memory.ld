MEMORY
{
    bl (rx) :             ORIGIN = 0x08000000,            LENGTH = 128K
    param1 (rx) :         ORIGIN = 0x08000000+128K,        LENGTH = 128K
    param2 (rx) :         ORIGIN = 0x08000000+256K,        LENGTH = 128K

    app (rx) :            ORIGIN = 0x08000000+384K,        LENGTH = 2M-384K

    ram0    : org = 0x24000000, len = 512k-256              /* AXI SRAM */
    ram1    : org = 0x30000000, len = 256k              /* AHB SRAM1+SRAM2 */
    ram2    : org = 0x30000000, len = 288k              /* AHB SRAM1+SRAM2+SRAM3 */
    ram3    : org = 0x30040000, len = 32k               /* AHB SRAM3 */
    ram4    : org = 0x38000000, len = 64k               /* AHB SRAM4 */
    ram5    : org = 0x20000000, len = 128k              /* DTCM-RAM */
    ram6    : org = 0x00000000, len = 64k               /* ITCM-RAM */
    ram7    : org = 0x38800000, len = 4k                /* BCKP SRAM */
    app_bl_shared (rwx) : ORIGIN = 0x24000000+(512k-256), LENGTH = 256
}

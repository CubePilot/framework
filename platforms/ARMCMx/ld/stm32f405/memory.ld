MEMORY
{
    bl (rx) :             ORIGIN = 0x08000000,            LENGTH = 32K
    param1 (rx) :         ORIGIN = 0x08000000+32K,        LENGTH = 16K
    param2 (rx) :         ORIGIN = 0x08000000+48K,        LENGTH = 16K
    app (rx) :            ORIGIN = 0x08000000+64K,        LENGTH = 1M-64K
    ram (rwx) :           ORIGIN = 0x20000000,            LENGTH = 128K-256
    app_bl_shared (rwx) : ORIGIN = 0x20000000+(128K-256), LENGTH = 256
}

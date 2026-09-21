MEMORY
{
    /* Last 2 KiB page is NVM (θe_off / poles). G431 erase size is 2 KiB. */
    FLASH : ORIGIN = 0x08000000, LENGTH = 126K
    NVM   : ORIGIN = 0x0801F800, LENGTH = 2K
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}

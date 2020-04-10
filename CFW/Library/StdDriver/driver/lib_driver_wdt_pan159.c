/*******************************************************************************
 * @note      Copyright (C) 2018 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_wdt_pan159.c
 * @brief     PAN159 hardware wdtÇý¶¯
 *
 * @history - V1.0, 2018-01-19, huoweibin, first implementation.
*******************************************************************************/
#include "lib_driver_wdt_pan159.h"
/**
 * @brief This function make WDT module start counting with different time-out interval
 * @param[in] u32TimeoutInterval  Time-out interval period of WDT module. Valid values are:
 *                - \ref WDT_TIMEOUT_2POW4 -- 1.6MS
 *                - \ref WDT_TIMEOUT_2POW6 -- 6.4MS
 *                - \ref WDT_TIMEOUT_2POW8 -- 25.6MS
 *                - \ref WDT_TIMEOUT_2POW10 -- 102.4MS
 *                - \ref WDT_TIMEOUT_2POW12 -- 409.6MS
 *                - \ref WDT_TIMEOUT_2POW14 -- 1.6384S
 *                - \ref WDT_TIMEOUT_2POW16 -- 6.5536S
 *                - \ref WDT_TIMEOUT_2POW18 -- 26.214S
*/
void DrvWDT_Init(uint32_t u32TimeoutInterval)
{
    SYS_UnlockReg();
    CLK->CLKSEL1 |= 0x00000003;
    SystemCoreClockUpdate();
    WDT->CTL = u32TimeoutInterval | WDT_CTL_WDTEN_Msk |
               (TRUE << WDT_CTL_RSTEN_Pos) |
               (FALSE << WDT_CTL_WKEN_Pos);
    WDT->ALTCTL = WDT_RESET_DELAY_3CLK;
}

void DrvWDT_Feed(void)
{
    WDT_RESET_COUNTER();
}

void DrvWDT_close(void)
{
    WDT->CTL = 0;
}



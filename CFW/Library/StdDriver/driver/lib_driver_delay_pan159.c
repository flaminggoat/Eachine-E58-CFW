/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_delay_pan159.c
 * @brief     PAN159 delay
 *
 * @history - V1.0, 2017-09-12, xiaoguolin, first implementation.
*******************************************************************************/
#include "lib_driver_delay_pan159.h"

__ASM void __delay_pan159(uint32_t cycle)
{
__delay_pan159_loop
    SUBS    r0,r0,#1
    BCS     __delay_pan159_loop
    BX      lr
}

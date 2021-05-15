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

void __delay_pan159(uint32_t cycle)
{
    __asm volatile(
        "__delay_pan159_loop:\n\t"
        "SUB    %[cycle],%[cycle],#1\n\t"
        "BCS     __delay_pan159_loop\n\t" : [cycle] "+l" (cycle) );
}

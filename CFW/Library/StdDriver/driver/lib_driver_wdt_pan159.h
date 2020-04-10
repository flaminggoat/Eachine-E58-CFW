/*******************************************************************************
 * @note      Copyright (C) 2018 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_wdt_pan159.c
 * @brief     PAN159 hardware wdtÇý¶¯
 *
 * @history - V1.0, 2018-01-19, huoweibin, first implementation.
*******************************************************************************/
#ifndef __LIB_DIRVER_WDT_PAN159_H
#define __LIB_DIRVER_WDT_PAN159_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Mini58Series.h"

void DrvWDT_Init(uint32_t ms);
void DrvWDT_Feed(void);
void DrvWDT_close(void);
#ifdef __cplusplus
}
#endif

#endif // __LIB_DIRVER_WDT_PAN159_H

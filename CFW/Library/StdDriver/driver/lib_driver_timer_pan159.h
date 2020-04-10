/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_timer_pan159.c
 * @brief     ¶¨Ê±Æ÷
 *
 * @history - V1.0, 2017-11-16, xiaoguolin, first implementation.
*******************************************************************************/
#ifndef __LIB_DRIVER_TIMER_PAN159_H
#define __LIB_DRIVER_TIMER_PAN159_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Mini58Series.h"

void timer_pan159_init(uint32_t freq, void(*irq_handler)(void));
void timer1_pan159_init(uint32_t freq, void(*irq_handler)(void));
void DrvTimer0_SetTimer(uint32_t usec);
void DrvTimer1_SetTimer(uint32_t usec);
#ifdef __cplusplus
}
#endif

#endif //__LIB_DRIVER_TIMER_PAN159_H

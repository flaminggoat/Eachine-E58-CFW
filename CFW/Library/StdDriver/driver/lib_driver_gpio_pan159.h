/*******************************************************************************
 * @note      Copyright (C) 2018 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_gpio_pan159.c
 * @brief     PAN159 hardware gpioÇý¶¯
 *
 * @history - V1.0, 2018-01-19, huoweibin, first implementation.
*******************************************************************************/
#ifndef __LIB_DRIVER_GPIO_PAN159_H
#define __LIB_DRIVER_GPIO_PAN159_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Mini58Series.h"
typedef void (*TCallback)(void);
void gpio_ext1_int_init(TCallback cbk);
void gpio_ext0_int_init(TCallback cbk);

#ifdef __cplusplus
}
#endif

#endif //__LIB_DRIVER_GPIO_PAN159_H

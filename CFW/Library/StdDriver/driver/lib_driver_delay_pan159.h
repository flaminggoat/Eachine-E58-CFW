/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_delay_pan159_v1p0.h
 * @brief     PAN159 delay
 *            注意：该延时函数在系统时钟为48MHz时准确
 *
 * @history - V1.0, 2017-09-12, xiaoguolin, first implementation.
*******************************************************************************/
#ifndef __DRIVER_DELAY_PAN159_H
#define __DRIVER_DELAY_PAN159_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Mini58Series.h"

void  __delay_pan159(uint32_t cycle);
#define delay_pan159_us(us)    do{if((us)){__delay_pan159((us)*12);}}while(0)
#define delay_pan159_ms(ms)    do{uint32_t cnt=(ms);while(cnt--){delay_pan159_us(2070);}}while(0)

#define delay_us        delay_pan159_us
#define delay_ms        delay_pan159_ms


#ifdef __cplusplus
}
#endif

#endif // __DRIVER_DELAY_PAN159_H

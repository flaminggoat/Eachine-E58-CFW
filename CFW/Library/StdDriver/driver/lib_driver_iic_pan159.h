/*******************************************************************************
 * @note      Copyright (C) 2018 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_iic_pan159.c
 * @brief     PAN159 hardware IICÇý¶¯
 *
 * @history - V1.0, 2018-01-19, huoweibin, first implementation.
*******************************************************************************/
#ifndef __LIB_DIRVER_IIC_PAN159_H
#define __LIB_DIRVER_IIC_PAN159_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Mini58Series.h"
typedef void (*TCallback)(void);
void iic_pan159_init(void);
void iic_start_send_bytes(uint16_t devAddr,uint8_t regAddr, uint8_t *buf, uint8_t len,TCallback cbk, TCallback fail_cbk);
void iic_start_read_bytes(uint16_t devAddr,uint8_t regAddr, uint8_t *buf, uint8_t len,TCallback cbk, TCallback fail_cbk);
uint8_t is_iic_busy(void);
#ifdef __cplusplus
}
#endif

#endif // __LIB_DIRVER_IIC_PAN159_H

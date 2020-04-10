/*******************************************************************************
 * @note      Copyright (C) 2018 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_uart_pan159.c
 * @brief     PAN159 hardware uartÇý¶¯
 *
 * @history - V1.0, 2018-01-19, huoweibin, first implementation.
*******************************************************************************/
#ifndef __LIB_DIRVER_UART_PAN159_H
#define __LIB_DIRVER_UART_PAN159_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Mini58Series.h"
#define RX_BUF_SIZE 32
#define TX_BUF_SIZE 32
//#define DEBUG_PORT UART1
void uart_init_pan159(void);
void uart_send(uint8_t *buff,uint16_t len);
#ifdef __cplusplus
}
#endif

#endif // __LIB_DIRVER_UART_PAN159_H

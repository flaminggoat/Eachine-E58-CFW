/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_spi.h
 * @brief     SPIÇý¶¯
 *
 * @history - V1.0, 2017-03-06, xiaoguolin, first implementation.
*******************************************************************************/
#ifndef __DRV_SPI_H
#define __DRV_SPI_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Mini58Series.h"

#define  rfspi_pan159_irq()          P52
#define  rfspi_pan159_cs(level)      (P01=(level))
void     rfspi_pan159_init(void);
uint8_t  rfspi_pan159_rwc(uint8_t dat);
uint8_t* rfspi_pan159_rws(uint8_t* p_dat, uint16_t len);
	
#define rfspi_ce(v)  xn297l_write_reg(CE_FSPI_ON|(v), 0)
#define rfspi_irq    rfspi_pan159_irq
#define rfspi_cs     rfspi_pan159_cs
#define rfspi_init   rfspi_pan159_init
#define rfspi_rwc    rfspi_pan159_rwc
#define rfspi_rws    rfspi_pan159_rws

#ifdef __cplusplus
}
#endif

#endif // __DRV_SPI_H

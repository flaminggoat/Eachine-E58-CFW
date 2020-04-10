/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_flash.h
 * @brief     FLASH¶ÁÐ´
 *
 * @history - V1.0, 2017-03-10, xiaoguolin, first implementation.
*******************************************************************************/
#ifndef __DRV_FLASH_H
#define __DRV_FLASH_H

#ifdef __cplusplus
extern "C"{
#endif
#include "Mini58Series.h"

void flash_pan159_erase(uint32_t addr, uint16_t sectors);
void flash_pan159_write(uint32_t addr, void* p_data, uint16_t len);
void flash_pan159_read(uint32_t addr, void* p_data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // __DRV_FLASH_H

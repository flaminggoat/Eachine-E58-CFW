/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_adc_pan159.h
 * @brief     
 *
 *            ADC对应的IO口:
 *            ---------+-----------
 *             ADC_CH0 |  P5.3(5)
 *            ---------+-----------
 *             ADC_CH1 |  P1.0(7)
 *            ---------+-----------
 *             ADC_CH2 |  P1.2(8)
 *            ---------+-----------
 *             ADC_CH3 |  P1.3(9)
 *            ---------+-----------
 *             ADC_CH4 |  P1.4(10)
 *            ---------+-----------
 *             ADC_CH5 |  P1.5(11)
 *            ---------+-----------
 *             ADC_CH6 |  P3.0(13)
 *            ---------+-----------
 *             ADC_CH7 |  P3.1(17)
 *            ---------+-----------
 *
 * @history - V1.0, 2017-09-18, xiaoguolin, first implementation.
*******************************************************************************/
#ifndef __LIB_DIRVER_ADC_PAN159_H
#define __LIB_DIRVER_ADC_PAN159_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Mini58Series.h"

#define ADC_ENABLE_CH0_P53        0
#define ADC_ENABLE_CH1_P10        0
#define ADC_ENABLE_CH2_P12        0
#define ADC_ENABLE_CH3_P13        1
#define ADC_ENABLE_CH4_P14        0
#define ADC_ENABLE_CH5_P15        0
#define ADC_ENABLE_CH6_P30        0
#define ADC_ENABLE_CH7_P31        0

#define ADC_PAN159_CH0_P53        (1<<0)
#define ADC_PAN159_CH1_P10        (1<<1)
#define ADC_PAN159_CH2_P12        (1<<2)
#define ADC_PAN159_CH3_P13        (1<<3)
#define ADC_PAN159_CH4_P14        (1<<4)
#define ADC_PAN159_CH5_P15        (1<<5)
#define ADC_PAN159_CH6_P30        (1<<6)
#define ADC_PAN159_CH7_P31        (1<<7)

void    adc_pan159_init(void);
void    adc_pan159_start(uint8_t chn, uint8_t smplclks);
void adc_pan159_samp2(uint16_t *buf);
void adc_pan159_samp1(uint16_t *buf);
#define adc_pan159_value()               ADC_GET_CONVERSION_DATA(ADC,NULL)
#define adc_pan159_is_busy()             ADC_IS_BUSY(ADC)

#ifdef __cplusplus
}
#endif

#endif //__LIB_DIRVER_ADC_PAN159_H

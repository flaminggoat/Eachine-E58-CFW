/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_adc_pan159.c
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
#include "lib_driver_adc_pan159.h"

const static uint8_t adc_chns_table[] = {
    #if ADC_ENABLE_CH0_P53 != 0
        ADC_PAN159_CH0_P53,
    #endif
    #if ADC_ENABLE_CH1_P10 != 0
        ADC_PAN159_CH1_P10,
    #endif
    #if ADC_ENABLE_CH2_P12 != 0
        ADC_PAN159_CH2_P12,
    #endif
    #if ADC_ENABLE_CH3_P13 != 0
        ADC_PAN159_CH3_P13,
    #endif
    #if ADC_ENABLE_CH4_P14 != 0
        ADC_PAN159_CH4_P14,
    #endif
    #if ADC_ENABLE_CH5_P15 != 0
        ADC_PAN159_CH5_P15,
    #endif
    #if ADC_ENABLE_CH6_P30 != 0
        ADC_PAN159_CH6_P30,
    #endif
    #if ADC_ENABLE_CH7_P31 != 0
        ADC_PAN159_CH7_P31,
    #endif
    0x00
};
#define ADC_CHN_NUM_COUNT            (sizeof(adc_chns_table)-1)
static  int8_t adc_cur_chn_index = 0;

void adc_pan159_init(void)
{
    ADC_POWER_ON(ADC);
    if(ADC_ENABLE_CH0_P53){
        SYS->P5_MFP &= ~SYS_MFP_P53_Msk;
        SYS->P5_MFP |= SYS_MFP_P53_ADC_CH0;
        GPIO_DISABLE_DIGITAL_PATH(P5,BIT3);
    }
    if(ADC_ENABLE_CH1_P10){
        SYS->P1_MFP &= ~SYS_MFP_P10_Msk;
        SYS->P1_MFP |= SYS_MFP_P10_ADC_CH1;
        GPIO_DISABLE_DIGITAL_PATH(P1,BIT0);
    }
    if(ADC_ENABLE_CH2_P12){
        SYS->P1_MFP &= ~SYS_MFP_P12_Msk;
        SYS->P1_MFP |= SYS_MFP_P12_ADC_CH2;
        GPIO_DISABLE_DIGITAL_PATH(P1,BIT2);
    }
    if(ADC_ENABLE_CH3_P13){
        SYS->P1_MFP &= ~SYS_MFP_P13_Msk;
        SYS->P1_MFP |= SYS_MFP_P13_ADC_CH3;
        GPIO_DISABLE_DIGITAL_PATH(P1,BIT3);
    }
    if(ADC_ENABLE_CH4_P14){
        SYS->P1_MFP &= ~SYS_MFP_P14_Msk;
        SYS->P1_MFP |= SYS_MFP_P14_ADC_CH4;
        GPIO_DISABLE_DIGITAL_PATH(P1,BIT4);
    }
    if(ADC_ENABLE_CH5_P15){
        SYS->P1_MFP &= ~SYS_MFP_P15_Msk;
        SYS->P1_MFP |= SYS_MFP_P15_ADC_CH5;
        GPIO_DISABLE_DIGITAL_PATH(P1,BIT5);
    }
    if(ADC_ENABLE_CH6_P30){
        SYS->P3_MFP &= ~SYS_MFP_P30_Msk;
        SYS->P3_MFP |= SYS_MFP_P30_ADC_CH6;
        GPIO_DISABLE_DIGITAL_PATH(P3,BIT0);
    }
    if(ADC_ENABLE_CH7_P31){
        SYS->P3_MFP &= ~SYS_MFP_P31_Msk;
        SYS->P3_MFP |= SYS_MFP_P31_ADC_CH7;
        GPIO_DISABLE_DIGITAL_PATH(P3,BIT1);
    }
}

void adc_pan159_samp2(uint16_t *buf)
{
    /* 获取当前通道 */
    //uint8_t curr_index = adc_cur_chn_index;
    //int16_t curr_chn = adc_chns_table[curr_index];
    
    //adc_pan159_start(adc_chns_table[adc_cur_chn_index],6);
    ADC->CHEN = adc_chns_table[adc_cur_chn_index];
    ADC->EXTSMPT = (ADC->EXTSMPT & ~ADC_EXTSMPT_EXTSMPT_Msk) | 6;
    ADC_START_CONV(ADC);
    while(adc_pan159_is_busy());
    /* 读取ADC转换结果 */
    buf[adc_cur_chn_index] = adc_pan159_value();
    
    /* 计算下一个通道 */
    if(adc_cur_chn_index < ADC_CHN_NUM_COUNT-1){
        adc_cur_chn_index++;
    }
    else{
        adc_cur_chn_index = 0;
    }
}

void adc_pan159_samp1(uint16_t *buf)
{    
    ADC->CHEN = ADC_PAN159_CH3_P13;
    //ADC->CHEN = ADC_PAN159_CH1_P10 ;
    ADC->EXTSMPT = (ADC->EXTSMPT & ~ADC_EXTSMPT_EXTSMPT_Msk) | 6;
    ADC_START_CONV(ADC);
    while(adc_pan159_is_busy());
    /* 读取ADC转换结果 */
    buf[0] = adc_pan159_value();
}

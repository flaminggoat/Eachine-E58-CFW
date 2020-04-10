/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_pwm.c
 * @brief     PWM驱动
 *            ----------+----------+----------+----------
 *             PWM0_CH0 |          | P1.2(8)  | P2.2(24)
 *            ----------+----------+----------+----------
 *             PWM0_CH1 |          | P1.3(9)  | P2.3(25)
 *            ----------+----------+----------+----------
 *             PWM0_CH2 |          |          | P2.4(26)
 *            ----------+----------+----------+----------
 *             PWM0_CH3 |          |          | P2.5(27)
 *            ----------+----------+----------+----------
 *             PWM0_CH4 |          | P1.4(10) | P2.6(28)
 *            ----------+----------+----------+----------
 *             PWM0_CH5 | P0.4(15) |          |
 *            ----------+----------+----------+----------
 *
 * @history - V1.0, 2017-04-20, xiaoguolin, first implementation.
 * @history - V1.1, 2018-01-18, huoweibin, add pwm_pan159_setDuty.
*******************************************************************************/
#include "lib_driver_pwm_pan159.h"

//      <0x50=>PWM0_CH0_P12  (CHIP_PIN_8)
//      <0x90=>PWM0_CH0_P22  (CHIP_PIN_24)
//      <0x59=>PWM0_CH1_P13  (CHIP_PIN_9)
//      <0x99=>PWM0_CH1_P23  (CHIP_PIN_25)
//      <0xA2=>PWM0_CH2_P24  (CHIP_PIN_26)
//      <0xAB=>PWM0_CH3_P25  (CHIP_PIN_27)
//      <0x64=>PWM0_CH4_P14  (CHIP_PIN_10)
//      <0xB4=>PWM0_CH4_P26  (CHIP_PIN_28)
//      <0x25=>PWM0_CH5_P04  (CHIP_PIN_15)
//#define __PWM_PAN159_M0      0xA2  //P24
//#define __PWM_PAN159_M1      0x99  //P23
//#define __PWM_PAN159_M2      0x90  //P22
//#define __PWM_PAN159_M3      0xAB  //P25
/*******************************************************************************
 * @brief      PWM初始化
 * @param[in]  
 * @return     1 - 成功
 *             0 - 失败
 * @history  - V1.0, 2017-09-12, xiaoguolin, first implementation.
*******************************************************************************/
void pwm_pan159_init(uint8_t  psc)
{
    //const uint32_t pwmch_module[] = {PWMCH01_MODULE,PWMCH23_MODULE,PWMCH45_MODULE};
    SYS_UnlockReg();
    CLK_EnableModuleClock(PWMCH01_MODULE);  //moudle clk enable ch01
    CLK_EnableModuleClock(PWMCH23_MODULE);  //moudle clk enable ch23
//        CLK_SetModuleClock(PWMCH01_MODULE,CLK_CLKSEL1_PWMCH01SEL_HCLK,0);//时钟1分频
//        CLK_SetModuleClock(PWMCH23_MODULE,CLK_CLKSEL1_PWMCH23SEL_HCLK,0);//时钟1分频
    //CLK_EnableModuleClock(PWMCH45_MODULE);  //moudle clk enable ch45
//    CLK_EnableModuleClock(pwmch_module[0x03 >> 1]);  //moudle clk enable ch3
    SystemCoreClockUpdate();
    SYS_LockReg();
    //P22
    SYS->P2_MFP &= ~SYS_MFP_P22_Msk;
    SYS->P2_MFP |= SYS_MFP_P22_PWM0_CH0;
    PWM->CTL |= PWM_CTL_CNTEN0_Msk | PWM_CTL_CNTMODE0_Msk;
    //P23
    SYS->P2_MFP &= ~SYS_MFP_P23_Msk;
    SYS->P2_MFP |= SYS_MFP_P23_PWM0_CH1;
    PWM->CTL |= PWM_CTL_CNTEN1_Msk | PWM_CTL_CNTMODE1_Msk;
    //P24
    SYS->P2_MFP &= ~SYS_MFP_P24_Msk;
    SYS->P2_MFP |= SYS_MFP_P24_PWM0_CH2;
    PWM->CTL |= PWM_CTL_CNTEN2_Msk | PWM_CTL_CNTMODE2_Msk;
    //P25
    SYS->P2_MFP &= ~SYS_MFP_P25_Msk;
    SYS->P2_MFP |= SYS_MFP_P25_PWM0_CH3;
    PWM->CTL |= PWM_CTL_CNTEN3_Msk | PWM_CTL_CNTMODE3_Msk;

    PWM_SET_PRESCALER(PWM,0x02,psc);
    PWM_SET_DIVIDER(PWM,0x02,PWM_CLK_DIV_1);
    //PWM_SET_ALIGNED_TYPE(PWM,0x02,0x80000000);//中心对齐
    PWM_SET_ALIGNED_TYPE(PWM,0x02,PWM_EDGE_ALIGNED);//边沿对齐
    PWM_SET_CMR(PWM,0x02,0);
    PWM_SET_CNR(PWM,0x02,1000);//PWM RANGE setting
    //PWM->CTL |= 1 << (PWM_CTL_PINV0_Pos + (((uint32_t)(0x02)) << 2));//输出取反

    PWM_SET_PRESCALER(PWM,0x01,psc);
    PWM_SET_DIVIDER(PWM,0x01,PWM_CLK_DIV_1);
    PWM_SET_ALIGNED_TYPE(PWM,0x01,PWM_EDGE_ALIGNED);
    PWM_SET_CMR(PWM,0x01,0);
    PWM_SET_CNR(PWM,0x01,1000);//PWM RANGE setting
    //PWM->CTL |= 1 << (PWM_CTL_PINV0_Pos + (((uint32_t)(0x01)) << 2));//输出取反
    
    PWM_SET_PRESCALER(PWM,0x00,psc);
    PWM_SET_DIVIDER(PWM,0x00,PWM_CLK_DIV_1);
    PWM_SET_ALIGNED_TYPE(PWM,0x00,PWM_EDGE_ALIGNED);
    PWM_SET_CMR(PWM,0x00,0);
    PWM_SET_CNR(PWM,0x00,1000);//PWM RANGE setting
    //PWM->CTL |= 1 << (PWM_CTL_PINV0_Pos + (((uint32_t)(0x00)) << 2));//输出取反
    
    PWM_SET_PRESCALER(PWM,0x03,psc);
    PWM_SET_DIVIDER(PWM,0x03,PWM_CLK_DIV_1);
    PWM_SET_ALIGNED_TYPE(PWM,0x03,PWM_EDGE_ALIGNED);
    PWM_SET_CMR(PWM,0x03,0);
    PWM_SET_CNR(PWM,0x03,1000);//PWM RANGE setting
    //PWM->CTL |= 1 << (PWM_CTL_PINV0_Pos + (((uint32_t)(0x03)) << 2));//输出取反
    
    /* 开启PWM */
    PWM->CTL |= PWM_CTL_CNTEN0_Msk << (((uint32_t)0x00) << 2);
    PWM->CTL |= PWM_CTL_CNTEN0_Msk << (((uint32_t)0x01) << 2);
    PWM->CTL |= PWM_CTL_CNTEN0_Msk << (((uint32_t)0x02) << 2);
    PWM->CTL |= PWM_CTL_CNTEN0_Msk << (((uint32_t)0x03) << 2);
    
    PWM_PAN159_CHN_OUT(0x00,0);
    PWM_PAN159_CHN_OUT(0x01,0);
    PWM_PAN159_CHN_OUT(0x02,0);
    PWM_PAN159_CHN_OUT(0x03,0);
    //PWM_PAN159_CHN_UNLOCK(1<<chn);
    

//    return pwm_port;
}
/*******************************************************************************
 * @brief      PWM输出
 * @param[in]  
 * @return     
 * @history  - V1.0, 2018-01-18, huoweibin, first implementation.
*******************************************************************************/
void pwm_pan159_setDuty(uint16_t *duty)
{
    register uint8_t lock = 0x00;
    register uint8_t unlock = 0x00;
    /*M1 - CH1 PWM out*/
    PWM_PAN159_CHN_OUT(0x01,duty[0]);
    if(duty[0] == 0){
        lock |= 1 << 0x01;
    }
    else{
        unlock |= 1 << 0x01;
    }
    /*M2 - CH3 PWM out*/
    PWM_PAN159_CHN_OUT(0x03,duty[1]);
    if(duty[1] == 0){
        lock |= 1 << 0x03;
    }
    else{
        unlock |= 1 << 0x03;
    }
    /*M3 - CH2 PWM out*/
    PWM_PAN159_CHN_OUT(0x02,duty[2]);
    if(duty[2] == 0){
        lock |= 1 << 0x02;
    }
    else{
        unlock |= 1 << 0x02;
    }
    /*M4 - CH0 PWM out*/
    PWM_PAN159_CHN_OUT(0x00,duty[3]);
    if(duty[3] == 0){
        lock |= 1 << 0x00;
    }
    else{
        unlock |= 1 << 0x00;
    }
    PWM_PAN159_CHN_LOCK(lock);
    PWM_PAN159_CHN_UNLOCK(unlock);

}



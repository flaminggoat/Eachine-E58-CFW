/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_pwm.h
 * @brief     PWM驱动
 *            机体坐标系(z朝下,x为机头,y为右侧)：
 *
 *            PWM对应的IO口:
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
*******************************************************************************/
#ifndef __LIB_DIRVER_PWM_PAN159_H
#define __LIB_DIRVER_PWM_PAN159_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Mini58Series.h"

#define __PWM_PAN159_PORT(P)              ((P)<<6)
#define __PWM_PAN159_PIN(B)               ((B)<<3)
#define __PWM_PAN159_CHN(C)               ((C)<<0)

#define PWM_PAN159_PWM0_CH0_P12           (__PWM_PAN159_CHN(0)|__PWM_PAN159_PORT(1)|__PWM_PAN159_PIN(2)) /* 01 010 000 = 0x50 */
#define PWM_PAN159_PWM0_CH0_P22           (__PWM_PAN159_CHN(0)|__PWM_PAN159_PORT(2)|__PWM_PAN159_PIN(2)) /* 10 010 000 = 0x90 */
#define PWM_PAN159_PWM0_CH1_P13           (__PWM_PAN159_CHN(1)|__PWM_PAN159_PORT(1)|__PWM_PAN159_PIN(3)) /* 01 011 001 = 0x59 */
#define PWM_PAN159_PWM0_CH1_P23           (__PWM_PAN159_CHN(1)|__PWM_PAN159_PORT(2)|__PWM_PAN159_PIN(3)) /* 10 011 001 = 0x99 */
#define PWM_PAN159_PWM0_CH2_P24           (__PWM_PAN159_CHN(2)|__PWM_PAN159_PORT(2)|__PWM_PAN159_PIN(4)) /* 10 100 010 = 0xA2 */
#define PWM_PAN159_PWM0_CH3_P25           (__PWM_PAN159_CHN(3)|__PWM_PAN159_PORT(2)|__PWM_PAN159_PIN(5)) /* 10 101 011 = 0xAB */
#define PWM_PAN159_PWM0_CH4_P14           (__PWM_PAN159_CHN(4)|__PWM_PAN159_PORT(1)|__PWM_PAN159_PIN(4)) /* 01 100 100 = 0x64 */
#define PWM_PAN159_PWM0_CH4_P26           (__PWM_PAN159_CHN(4)|__PWM_PAN159_PORT(2)|__PWM_PAN159_PIN(6)) /* 10 110 100 = 0xB4 */
#define PWM_PAN159_PWM0_CH5_P04           (__PWM_PAN159_CHN(5)|__PWM_PAN159_PORT(0)|__PWM_PAN159_PIN(4)) /* 00 100 101 = 0x25 */

#define PWM_PAN159_PORT_MSK               __PWM_PAN159_PORT(0x03)
#define PWM_PAN159_PIN_MSK                __PWM_PAN159_PIN(0x07)
#define PWM_PAN159_CHN_MSK                __PWM_PAN159_CHN(0x07)

#define PWM_PAN159_PORT(PWMx)             ((PWMx)>>6)
#define PWM_PAN159_PIN(PWMx)              (((PWMx)&PWM_PAN159_PIN_MSK)>>3)
#define PWM_PAN159_CHN(PWMx)              ((PWMx)&PWM_PAN159_CHN_MSK)

#define PWM_PAN159_CMPDAT                 ((volatile uint32_t*)(0x40040000+0x24))
#define PWM_PAN159_OEN                    (*(volatile uint32_t*)(0x40040000+0x5C))

#define PWM_PAN159_CHN_ISLOCKED(CHN_BITS) (PWM_PAN159_OEN&(CHN_BITS))
#define PWM_PAN159_CHN_LOCK(CHN_BITS)     (PWM_PAN159_OEN&=~(CHN_BITS))
#define PWM_PAN159_CHN_UNLOCK(CHN_BITS)   (PWM_PAN159_OEN|=(CHN_BITS))
#define PWM_PAN159_CHN_OUT(CHN,VAL)       (PWM_PAN159_CMPDAT[(CHN)]=(VAL))

#define pwm_pan159_islocked(pwm_port)     PWM_PAN159_CHN_ISLOCK(1<<PWM_PAN159_CHN((pwm_port)))
#define pwm_pan159_lock(pwm_port)         PWM_PAN159_CHN_LOCK(1<<PWM_PAN159_CHN((pwm_port)))
#define pwm_pan159_unlock(pwm_port)       PWM_PAN159_CHN_UNLOCK(1<<PWM_PAN159_CHN((pwm_port)))
#define pwm_pan159_out(pwm_port,val)      PWM_PAN159_CHN_OUT(PWM_PAN159_CHN((pwm_port)),val)
//uint8_t pwm_pan159_init(uint8_t pwm_port,uint8_t psc,uint8_t div,uint32_t align,uint32_t cnt,uint8_t inv);
void pwm_pan159_init(uint8_t  psc);
void pwm_pan159_setDuty(uint16_t *duty);
#ifdef __cplusplus
}
#endif

#endif // __LIB_DIRVER_PWM_PAN159_H

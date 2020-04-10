/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_timer_pan159.c
 * @brief     ¶¨Ê±Æ÷
 *
 * @history - V1.0, 2017-11-16, xiaoguolin, first implementation.
*******************************************************************************/
#include "lib_driver_timer_pan159.h"

static void (*__s_lib_driver_timer_pan159_irq_handler)(void) = NULL;
static void (*__s_lib_driver_timer1_pan159_irq_handler)(void) = NULL;
/*******************************************************************************
 * @brief      ¶¨Ê±Æ÷0³õÊ¼»¯
 * @param[in]  ÎÞ
 * @param[out] ÎÞ
 * @return     ÎÞ
 * @history  - V1.0, 2017-03-07, xiaoguolin, first implementation.
 ******************************************************************************/
void timer_pan159_init(uint32_t freq, void(*irq_handler)(void))
{
	CLK_EnableModuleClock(TMR0_MODULE);
	TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, freq);
    // Enable timer interrupt
    if(irq_handler){
        TIMER_EnableInt(TIMER0);
        NVIC_EnableIRQ(TMR0_IRQn);
        NVIC_SetPriority(TMR0_IRQn,2);
    }
    __s_lib_driver_timer_pan159_irq_handler = irq_handler;
    // Start Timer 0.
    TIMER_Start(TIMER0);
}

/*******************************************************************************
 * @brief      ¶¨Ê±Æ÷0ÖÐ¶Ï´¦Àíº¯Êý
 * @param[in]  ÎÞ
 * @param[out] ÎÞ
 * @return     ÎÞ
 * @history  - V1.0, 2017-03-07, xiaoguolin, first implementation.
 ******************************************************************************/
void TMR0_IRQHandler(void)
{
	TIMER_ClearIntFlag(TIMER0);
    if(__s_lib_driver_timer_pan159_irq_handler){
        __s_lib_driver_timer_pan159_irq_handler();
    }
}

/*******************************************************************************
 * @brief      ¶¨Ê±Æ1³õÊ¼»¯
 * @param[in]  ÎÞ
 * @param[out] ÎÞ
 * @return     ÎÞ
 * @history  - V1.0, 2018-01-19, huoweibin, first implementation.
 ******************************************************************************/
void timer1_pan159_init(uint32_t freq, void(*irq_handler)(void))
{
	CLK_EnableModuleClock(TMR1_MODULE);
	TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, freq);
    // Enable timer interrupt
    if(irq_handler){
        TIMER_EnableInt(TIMER1);
        NVIC_EnableIRQ(TMR1_IRQn);
        NVIC_SetPriority(TMR1_IRQn,2);
    }
    __s_lib_driver_timer1_pan159_irq_handler = irq_handler;
    // Start Timer 0.
    TIMER_Start(TIMER1);
}
/*******************************************************************************
 * @brief      ¶¨Ê±Æ÷1ÖÐ¶Ï´¦Àíº¯Êý
 * @param[in]  ÎÞ
 * @param[out] ÎÞ
 * @return     ÎÞ
 * @history  - V1.0, 2018-01-19, huoweibin, first implementation.
 ******************************************************************************/
void TMR1_IRQHandler(void)
{
	TIMER_ClearIntFlag(TIMER1);
    if(__s_lib_driver_timer1_pan159_irq_handler){
        __s_lib_driver_timer1_pan159_irq_handler();
    }
}

void DrvTimer0_SetTimer(uint32_t usec)
{
    TIMER_Close(TIMER0);
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, usec);
    TIMER_EnableInt(TIMER0);
    //NVIC_EnableIRQ(TMR0_IRQn);
    //NVIC_SetPriority(TMR0_IRQn,2);
    TIMER_Start(TIMER0);
}

void DrvTimer1_SetTimer(uint32_t usec)
{
    TIMER_Close(TIMER1);
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, usec);
    TIMER_EnableInt(TIMER1);
    //NVIC_EnableIRQ(TMR1_IRQn);
    //NVIC_SetPriority(TMR1_IRQn,2);
    TIMER_Start(TIMER1);
}

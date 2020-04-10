/*******************************************************************************
 * @note      Copyright (C) 2018 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_gpio_pan159.c
 * @brief     PAN159 hardware gpioÇý¶¯
 *
 * @history - V1.0, 2018-01-19, huoweibin, first implementation.
*******************************************************************************/

#include "lib_driver_gpio_pan159.h"
typedef void (*TCallback)(void);
static void (*__s_lib_driver_EINT1_pan159_irq_handler)(void) = NULL;
static void (*__s_lib_driver_EINT0_pan159_irq_handler)(void) = NULL;
void gpio_ext1_int_init(TCallback cbk)
{
    if(cbk){
        GPIO_EnableEINT1(P5, 2, GPIO_INT_FALLING);
        NVIC_EnableIRQ(EINT1_IRQn);
    }
    __s_lib_driver_EINT1_pan159_irq_handler =  cbk;   
}
void gpio_ext0_int_init(TCallback cbk)
{
    GPIO_SetMode(P3, BIT2, GPIO_MODE_INPUT);
    if(cbk){
        GPIO_EnableEINT1(P3, 2, GPIO_INT_FALLING);
        NVIC_EnableIRQ(EINT0_IRQn);
    }
    __s_lib_driver_EINT0_pan159_irq_handler =  cbk;   
}

void EINT0_IRQHandler(void)
{
    /* For P3.2, clear the INT flag */
    P3->INTSRC = BIT2;
    if(__s_lib_driver_EINT0_pan159_irq_handler){
        __s_lib_driver_EINT0_pan159_irq_handler();
    }
    
}
void EINT1_IRQHandler(void)
{
    /* For P5.2, clear the INT flag */
    P5->INTSRC = BIT2;
    if(__s_lib_driver_EINT1_pan159_irq_handler){
        __s_lib_driver_EINT1_pan159_irq_handler();
    }
    

}
/*******************************************************************************
 * @note      Copyright (C) 2018 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_iic_pan159.c
 * @brief     PAN159 hardware IICÇý¶¯
 *
 * @history - V1.0, 2018-01-19, huoweibin, first implementation.
*******************************************************************************/
#include "lib_driver_iic_pan159.h"
//#include <string.h>

uint8_t g_u8DeviceAddr;
uint8_t g_u8RegAddr;
//uint8_t g_au8TxData[64];
//uint8_t g_u8RxData[64];
uint8_t g_u8DataLen;
uint8_t g_u8DataLenMax;
uint8_t __IO g_u8EndFlag = 0;
typedef void (*TCallback)(void);
typedef void (*I2C_FUNC)(uint32_t u32Status,uint8_t *buf,TCallback cbk, TCallback fail_cbk);

I2C_FUNC __IO s_I2CHandlerFn = NULL;
TCallback __IO cbk_success = NULL;
TCallback __IO cbk_failed = NULL;
uint8_t* buf_addr = NULL;
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0)) {
        /* Clear I2C Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    } 
    else {
        if (s_I2CHandlerFn != NULL)
            s_I2CHandlerFn(u32Status,buf_addr,cbk_success,cbk_failed);
    }
}

void iic_pan159_init(void)
{
    //uint32_t u32Div;

    CLK_EnableModuleClock(I2C0_MODULE);
    SYS->P3_MFP &= ~SYS_MFP_P34_Msk;
    SYS->P3_MFP |= SYS_MFP_P34_I2C0_SDA;
    SYS->P3_MFP &= ~SYS_MFP_P35_Msk;
    SYS->P3_MFP |= SYS_MFP_P35_I2C0_SCL;
    
    SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;
    
    I2C_Open(I2C0,400000);
//    u32Div = (uint32_t) (((SystemCoreClock * 10)/(400000 * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */
//    I2C0->CLKDIV = u32Div;
//    I2C0->CTL |= I2C_CTL_I2CEN_Msk;
    
    I2C0->CTL |= I2C_CTL_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
    
}

void iic_master_tx(uint32_t u32Status ,uint8_t *buf, TCallback cbk, TCallback fail_cbk)
{
    if (u32Status == 0x08) {                    /* START has been transmitted */
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } 
    else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C_SET_DATA(I2C0, g_u8RegAddr);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } 
    else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
    } 
    else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8DataLen != g_u8DataLenMax) {
            I2C_SET_DATA(I2C0, buf[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
            g_u8EndFlag = 1;
            if(cbk != NULL)
            {
                cbk();
            }
        }
    } 
    else {
        if(fail_cbk != NULL)
        {
            fail_cbk();
        }
        //I2C_STOP(I2C0);
        //(I2C0)->CTL |= (I2C_CTL_SI_Msk | I2C_CTL_STO_Msk);
        /* TO DO */
//        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}
void iic_master_rx(uint32_t u32Status ,uint8_t *buf,TCallback cbk, TCallback fail_cbk){
    if (u32Status == 0x08) {                    /* START has been transmitted and prepare SLA+W */
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } 
    else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C_SET_DATA(I2C0, g_u8RegAddr);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } 
    else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
    } 
    else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_SI);
    } 
    else if (u32Status == 0x10) {             /* Repeat START has been transmitted and prepare SLA+R */
        I2C_SET_DATA(I2C0, ((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } 
    else if (u32Status == 0x40) {             /* SLA+R has been transmitted and ACK has been received */
        if(g_u8DataLenMax== 1)
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_SI );
        }
        else{
            I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA );
        }
    } 
    else if (u32Status == 0x50) {             /* DATA has been received and ACK has been returned */
        if(g_u8DataLen != g_u8DataLenMax-2){
            buf[g_u8DataLen++] = I2C_GET_DATA(I2C0);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);
        }
        else{
            buf[g_u8DataLen++] = I2C_GET_DATA(I2C0);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI );
        }
    } 
    else if (u32Status == 0x58) {             /* DATA has been received and NACK has been returned */
            buf[g_u8DataLen++] = I2C_GET_DATA(I2C0);
            I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
            g_u8EndFlag = 1;
            if(cbk != NULL)
            {
                cbk();
            }
    } 
    else {
        if(fail_cbk != NULL)
        {
            fail_cbk();
        }
        //(I2C0)->CTL |= (I2C_CTL_SI_Msk | I2C_CTL_STO_Msk);
        //I2C_STOP(I2C0);
        /* TO DO */
        //printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void iic_start_send_bytes(uint16_t devAddr,uint8_t regAddr, uint8_t *buf, uint8_t len,TCallback cbk, TCallback fail_cbk)
{
    I2C0->CTL |= I2C_CTL_I2CEN_Msk;
    
    I2C0->CTL |= I2C_CTL_INTEN_Msk;
    g_u8DataLenMax = len;
    g_u8DeviceAddr = devAddr;
    
    g_u8RegAddr = regAddr;

    g_u8DataLen = 0;
    g_u8EndFlag = 0;
    /* I2C function to write data to slave */
    s_I2CHandlerFn = (I2C_FUNC)iic_master_tx;
    cbk_success = cbk;
    cbk_failed = fail_cbk;
    buf_addr = buf;
    /* I2C as master sends START signal */
    I2C_START(I2C0);
   

}

void iic_start_read_bytes(uint16_t devAddr,uint8_t regAddr, uint8_t *buf, uint8_t len,TCallback cbk, TCallback fail_cbk)
{
    I2C0->CTL |= I2C_CTL_I2CEN_Msk;
    
    I2C0->CTL |= I2C_CTL_INTEN_Msk;
    g_u8DataLenMax = len;
    g_u8DeviceAddr = devAddr;
    g_u8RegAddr = regAddr;
    g_u8DataLen = 0;
    g_u8EndFlag = 0;
    s_I2CHandlerFn = (I2C_FUNC)iic_master_rx;
    cbk_success = cbk;
    cbk_failed = fail_cbk;
    buf_addr = buf;
    //CLK_SysTickDelay(5000);
    /* I2C as master sends START signal */
    I2C_START(I2C0);
     //while (g_u8EndFlag == 0);
}

uint8_t is_iic_busy(void)
{
    if(g_u8EndFlag){
        return 0;
    }
    else{
        return 1;
    }
}

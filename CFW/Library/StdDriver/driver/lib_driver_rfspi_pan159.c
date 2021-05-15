/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_spi.c
 * @brief     SPI?y???
 *
 * @history - V1.0, 2017-03-06, xiaoguolin, first implementation.
*******************************************************************************/
#include "lib_driver_rfspi_pan159.h"

/*******************************************************************************
 * @brief      SPI?????
 * @param[in]  ??
 * @param[out] ??
 * @return     ??
 ******************************************************************************/
void rfspi_pan159_init(void)
{
    // SYS->P0_MFP &= SYS_MFP_P01_Msk;
	SYS->P0_MFP |= SYS_MFP_P01_GPIO;	// P01->RF_CSN
    SYS->P0_MFP |= SYS_MFP_P07_SPI0_CLK|SYS_MFP_P05_SPI0_MOSI|SYS_MFP_P06_SPI0_MISO ;
    SYS->P5_MFP |= SYS_MFP_P52_GPIO;	                //P20->RF_IRQ
    // SYS->P5_MFP &= SYS_MFP_P52_Msk;
    
    CLK_EnableModuleClock(SPI0_MODULE);//???SPI???
	CLK_SetModuleClock(SPI0_MODULE,CLK_CLKSEL1_SPISEL_HCLK,1);//???1???
    
 	GPIO_SetMode(P0, BIT1, GPIO_MODE_OUTPUT);     	//RF_CSN
 	GPIO_SetMode(P5, BIT2, GPIO_MODE_INPUT);		//RF_IRQ
    
	SYS->P0_MFP |= SYS_MFP_P05_SPI0_MOSI | SYS_MFP_P06_SPI0_MISO | SYS_MFP_P07_SPI0_CLK;//????IO???¨΄???
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0,8,1000000);//???υτ????0,8¦Λ?????2M????
	SPI_DisableAutoSS(SPI0);//??????SS??
	SPI_WRITE_TX(SPI0, 0);//???SPI????
}

/*******************************************************************************
 * @brief      SPI????§Υ
 * @param[in]  dat - ??§Υ??????
 * @param[out] ??
 * @return     ??????????
 ******************************************************************************/
uint8_t rfspi_pan159_rwc(uint8_t dat)
{
    SPI_WRITE_TX(SPI0, dat);
    SPI_TRIGGER(SPI0);
    while(SPI_IS_BUSY(SPI0));
    return SPI0->RX;
}

/*******************************************************************************
 * @brief      SPI??§Υ?????
 * @param[in]  p_dat - ???????
 *             len   - ???????
 * @param[out] ??
 * @return     ???????????ο…??p_dat????????
 * @history  - V1.0, 2017-03-06, xiaoguolin, first implementation.
 ******************************************************************************/
uint8_t* rfspi_pan159_rws(uint8_t* p_dat, uint16_t len)
{
    uint8_t i = 0;
    while(i < len)
    {
        SPI_WRITE_TX(SPI0, p_dat[i]);
        SPI_TRIGGER(SPI0);
        while(SPI_IS_BUSY(SPI0));
        p_dat[i] = SPI0->RX;
        i++;
    }
    return p_dat;
}

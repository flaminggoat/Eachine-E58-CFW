/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_spi.c
 * @brief     SPI?y?ˉ
 *
 * @history - V1.0, 2017-03-06, xiaoguolin, first implementation.
*******************************************************************************/
#include "lib_driver_rfspi_pan159.h"

/*******************************************************************************
 * @brief      SPI初始化
 * @param[in]  无
 * @param[out] 无
 * @return     无
 ******************************************************************************/
void rfspi_pan159_init(void)
{
	SYS->P0_MFP |= SYS_MFP_P01_GPIO;	// P01->RF_CSN
    SYS->P0_MFP |= SYS_MFP_P07_SPI0_CLK|SYS_MFP_P05_SPI0_MOSI|SYS_MFP_P06_SPI0_MISO ;
    SYS->P5_MFP |= SYS_MFP_P52_GPIO;	                //P20->RF_IRQ
    
    CLK_EnableModuleClock(SPI0_MODULE);//使能SPI时钟
	CLK_SetModuleClock(SPI0_MODULE,CLK_CLKSEL1_SPISEL_HCLK,1);//时钟1分频
    
 	GPIO_SetMode(P0, BIT1, GPIO_MODE_OUTPUT);     	//RF_CSN
 	GPIO_SetMode(P5, BIT2, GPIO_MODE_INPUT);		//RF_IRQ
    
	SYS->P0_MFP |= SYS_MFP_P05_SPI0_MOSI | SYS_MFP_P06_SPI0_MISO | SYS_MFP_P07_SPI0_CLK;//设置IO复用功能
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0,8,2000000);//主设备，模式0,8位宽度，2M速率
	SPI_DisableAutoSS(SPI0);//禁止硬件SS脚
	SPI_WRITE_TX(SPI0, 0);//清除SPI数据
}

/*******************************************************************************
 * @brief      SPI字节读写
 * @param[in]  dat - 待写入数据
 * @param[out] 无
 * @return     读取到的字节
 ******************************************************************************/
uint8_t rfspi_pan159_rwc(uint8_t dat)
{
    SPI_WRITE_TX(SPI0, dat);
    SPI_TRIGGER(SPI0);
    while(SPI_IS_BUSY(SPI0));
    return SPI0->RX;
}

/*******************************************************************************
 * @brief      SPI读写数据块
 * @param[in]  p_dat - 数据指针
 *             len   - 数据长度
 * @param[out] 无
 * @return     读取到的数据块，与p_dat共缓冲区
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

/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_xn297l.h
 * @brief     XN297L RF驱动
 *
 * @history - V1.0, 2017-03-06, xiaoguolin, first implementation.
*******************************************************************************/
#ifndef __DRV_XN297L_H
#define __DRV_XN297L_H

#ifdef __cplusplus
extern "C"{
#endif

#include "lib_driver_delay_pan159.h"
#include "lib_driver_rfspi_pan159.h"

#define  XN297L_RF_DATA_RATE_1M             0x00
#define  XN297L_RF_DATA_RATE_2M             0x40
#define  XN297L_RF_DATA_RATE_250K           0xC0

#define     USER_RF_POWER                   (CONF_RF_REMOTE_POWER|CONF_RF_REMOTE_RATE)
//#define     MONITOR_RF_POWER                (CONF_RF_CONGLEX_POWER|CONF_RF_DONGLEX_RATE)

#define     TRANS_ENHANCE_MODE      1                               //发送增强模式
#define     TRANS_NORMAL_MODE       2                               //发送普通模式
#define     TRANSMIT_TYPE           TRANS_NORMAL_MODE               //初始设置为普通模式

#define    	RX_DR_FLAG             	0x40                            // 接收中断标志位
#define    	TX_DS_FLAG            	0x20                            // 发送完成中断标志位
#define    	RX_TX_FLAG            	0x60                            // 发送接收完成中断标志位，ack_payload 模式下使用
#define   	MAX_RT_FLAG           	0x10                            // 发送重传超时中断标志位

#define  XN297L_RF_POWER_P_11              0x27  /**  11dBm */
#define  XN297L_RF_POWER_P_10              0x26  /**  10dBm */
#define  XN297L_RF_POWER_P_9               0x15  /**   9dBm */
#define  XN297L_RF_POWER_P_5               0x2c  /**   5dBm */
#define  XN297L_RF_POWER_P_4               0x14  /**   4dBm */
#define  XN297L_RF_POWER_N_1               0x2A  /**  -1dBm */
#define  XN297L_RF_POWER_N_9               0x29  /**  -9dBm */
#define  XN297L_RF_POWER_N_10              0x19  /** -10dBm */
#define  XN297L_RF_POWER_N_23              0x30  /** -23dBm */

#if( (DATA_RATE ==DR_1M) || (DATA_RATE==DR_2M) )

#define         	rf7dBm                         	0X0D  // 7dBm 只能用于1M过安规用
#define         	rf6dBm                         	0X06  // 6dBm 只能用于1M过安规用
#define         	rf3dBm                         	0X0C  // 3dBm 只能用于1M过安规用

#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//XN297L寄存器操作命令
#define     RF_READ_REG             0x00                            //读配置寄存器,低5位为寄存器地址
#define     RF_WRITE_REG            0x20                            //写配置寄存器,低5位为寄存器地址
#define     W_REGISTER		        0x20                            //写配置寄存器,低5位为寄存器地址
#define     R_RX_PAYLOAD            0x61                            //读RX有效数据, 1~32字节
#define     W_TX_PAYLOAD            0xA0                            //写TX有效数据,1~32字节
#define     FLUSH_TX                0xE1                            //清除TX FIFO寄存器.发射模式下用
#define     FLUSH_RX                0xE2                            //清除RX FIFO寄存器.接收模式下用
#define     REUSE_TX_PL             0xE3                            //重新使用上一包数据,CE为高,数据包被不断发送.
#define     NOP                     0xFF                            //空操作,可以用来读状态寄存器


#define		ACTIVATE			    0x50                            //ACTIVATE
#define		DEACTIVATE			    0x50                            //DEACTIVATE
#define		R_RX_PL_WID			    0x60                            //Read width of RX data
#define		W_ACK_PAYLOAD		    0xA8                            //Data with ACK
#define		W_TX_PAYLOAD_NOACK	    0xB0                            //TX Payload no ACK Request
#define		CE_FSPI_ON	            0xFD                            // CE HIGH
#define		CE_FSPI_OFF	            0xFC                            // CE LOW
#define		RST_FSPI	            0x53                            // RESET


//SPI(XN297L)寄存器地址
#define     CONFIG                  0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
//bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define     EN_AA                   0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define     EN_RXADDR               0x02  //接收地址允许,bit0~5,对应通道0~5
#define     SETUP_AW                0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define     SETUP_RETR              0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define     RF_CH                   0x05  //rf通道,bit6:0,工作通道频率;
#define     RF_SETUP                0x06  //rf寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define     RF_STATUS                  0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
//bit5:数据发送完成中断;bit6:接收数据中断;
#define     MAX_TX  		        0x10  //达到最大发送次数中断
#define     TX_OK   		        0x20  //TX发送完成中断
#define     RX_OK   		        0x40  //接收到数据中断

#define     OBSERVE_TX              0x08  //传输状态寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define     CD                      0x09  //载波检测寄存器,bit0,载波检测;
#define     RX_ADDR_P0              0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前
#define     RX_ADDR_P1              0x0B  //数据通道1接收地址,最大长度5个字节,低字节在前
#define     RX_ADDR_P2              0x0C  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define     RX_ADDR_P3              0x0D  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define     RX_ADDR_P4              0x0E  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define     RX_ADDR_P5              0x0F  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define     TX_ADDR                 0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define     RX_PW_P0                0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define     RX_PW_P1                0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define     RX_PW_P2                0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define     RX_PW_P3                0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define     RX_PW_P4                0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define     RX_PW_P5                0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define     FIFO_STATUS             0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
//bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;

#define		DEM_CAL				    0x19
#define     RF_CAL2				    0x1A
#define     DEM_CAL2			    0x1B
#define		DYNPD			        0x1C
#define		FEATURE				    0x1D
#define		RF_CAL				    0x1E
#define		BB_CAL				    0x1F


#define     USER_CHN   	    		CONF_RF_REMOTE_CHN              //通信信道设置
#define     MONITOR_CHN   	    	CONF_RF_DONGLEX_CHN             //监测通信信道设置
#define     PAYLOAD_WIDTH           CONF_RF_REMOTE_RX_PAYLOAD_WIDTH //XN297L发送接收数据宽度定义
#define 	MONITOR_WIDTH 			CONF_RF_DONGLEX_TX_PAYLOAD_WIDTH
//#define     TX_ADR_WIDTH            5   	//5字节的地址宽度
//#define     RX_ADR_WIDTH            5   	//5字节的地址宽度
//#define     TX_PLOAD_WIDTH          32  	//32字节的用户数据宽度
//#define     RX_PLOAD_WIDTH          32  	//32字节的用户数据宽度

#define     CE_SOFTWARE_MODE      	1                             //CE软件模式
#define     CE_HARDWARE_MODE       	2                             //CE硬件模式
#define     CE_TYPE           		CE_SOFTWARE_MODE              //设置为硬件模式

#define IRQ_STATUS  rfspi_irq()
#define CSN_HIGH   P01=1 
#define CSN_LOW   P01=0

//#if 0
////PAN163
//#if(CE_TYPE==CE_HARDWARE_MODE)
//	#define set_CE()   	(P02=1)
//	#define clear_CE()  (P02=0)
//#else
//	#define     set_CE()                (xn297l_write_reg(CE_FSPI_ON, 0))//内部置位CE
//	#define     clear_CE()              (xn297l_write_reg(CE_FSPI_OFF, 0))//内部拉低CE
//#endif
//#endif
////PAN159
//#define     set_CE()                (xn297l_write_reg(CE_FSPI_ON, 0))//内部置位CE
//#define     clear_CE()              (xn297l_write_reg(CE_FSPI_OFF, 0))//内部拉低CE


#if(CE_TYPE==CE_HARDWARE_MODE)
	#define set_CE()   	rfspi_ce(1)
	#define clear_CE()  rfspi_ce(0)
#else
	#define set_CE()    (xn297l_write_reg(CE_FSPI_ON, 0))//内部置位CE
	#define clear_CE()  (xn297l_write_reg(CE_FSPI_OFF, 0))//内部拉低CE
#endif

void    xn297l_write_reg(uint8_t reg,uint8_t data);//rf写寄存器函数
uint8_t xn297l_read_reg(uint8_t reg);//rf读寄存器函数
void    xn297l_write_buf(uint8_t reg,uint8_t *pbuf,uint8_t length);//rf写缓冲函数
void    xn297l_read_buf(uint8_t reg,uint8_t *pbuf,uint8_t length);//rf读缓冲函数
//void    xn297l_init(void);//rf初始化
void    xn297l_init(uint8_t* addr,uint8_t addr_len,uint8_t chn,uint8_t payload,uint8_t rate_power);
void    xn297l_tx_mode(void);//XN297L设置为发送模式
void    xn297l_rx_mode(void);//XN297L设置为接收模式
uint8_t xn297l_get_status(void);//XN297L获取状态信息
void    xn297l_clear_status(void);//XN297L清除状态信息
void    xn297l_clear_fifo(void);//XN297L清除缓冲
void    xn297l_set_channel(uint8_t channel);//XN297L设置通信信道
void    xn297l_tx_data(uint8_t *data, uint8_t length);//XN297L发送数据
uint8_t xn297l_rx_data(uint8_t *data,uint8_t length);//rf接收函数
void    xn297l_set_addr(uint8_t reg,uint8_t *rf_addr_array,uint8_t addr_len);//rf设置通信地址
void rf_init(void);

#ifdef __cplusplus
}
#endif

#endif // __DRV_XN297L_H

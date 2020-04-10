/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_xn297l.c
 * @brief     XN297L RF驱动
 *
 * @history - V1.0, 2017-03-06, xiaoguolin, first implementation.
*******************************************************************************/
#include "lib_driver_xn297l.h"

/******************************************************************************/
//void rf_write_reg(uint8_t reg,uint8_t data);
//rf写寄存器函数
//input:uint8_t reg，寄存器地址；uint8_t data:需要写入寄存器的值
//output:无
/******************************************************************************/
void xn297l_write_reg(uint8_t reg,uint8_t data)
{
    rfspi_cs(0);      //使能片选
    rfspi_rwc(reg);   //写入的寄存器的地址
    rfspi_rwc(data);  //写入的寄存器的值
    rfspi_cs(1);      //禁能片选
}
/******************************************************************************/
//uint8_t rf_read_reg(uint8_t reg);
//rf读寄存器函数
//input:uint8_t reg，需要读取的寄存器
//output:寄存器的值
/******************************************************************************/
uint8_t xn297l_read_reg(uint8_t reg)
{
    uint8_t tmp;
    rfspi_cs(0);              //使能片选
    rfspi_rwc(reg);           //写入的寄存器的地址
    tmp = rfspi_rwc(0x00);    //读取寄存器的值
    rfspi_cs(1);              //禁能片选
    return tmp;
}
/******************************************************************************/
//void rf_write_buf(uint8_t reg,uint8_t *pbuf,uint8_t length);
//rf写缓冲函数
//input:uint8_t reg，写入寄存器的地址;uint8_t *pbuf，写入数组的首地址;uint8_t length，写入数据的长度
//output:无
/******************************************************************************/
void xn297l_write_buf(uint8_t reg,uint8_t *pbuf,uint8_t length)
{
    uint8_t i;
    rfspi_cs(0);                  //使能片选
    rfspi_rwc(reg);
    for(i = 0;i < length; i++)
    {
        rfspi_rwc(pbuf[i]);       //写入数据
    }
    rfspi_cs(1);                  //禁能片选
}

/******************************************************************************/
//void rf_read_buf(uint8_t reg,uint8_t *pbuf,uint8_t length);
//rf读缓冲函数
//input:uint8_t reg，读出寄存器的地址;uint8_t *pbuf，读出缓冲数组的首地址;uint8_t length，读出数据的长度
//output:无
/******************************************************************************/
void xn297l_read_buf(uint8_t reg, uint8_t *pbuf, uint8_t length)
{
    uint8_t i;
    rfspi_cs(0);                      //使能片选
    rfspi_rwc(reg);
    for(i = 0; i < length; i++)
    {
        pbuf[i] = rfspi_rwc(0x00);    //读取数据
    }
    rfspi_cs(1);                      //禁能片选
}

/******************************************************************************/
//void rf_tx_mode(void);
//XN297L设置为发送模式
//input:无
//output:无
/******************************************************************************/
void xn297l_tx_mode(void)
{
//     uint8_t rf_cal_data[]    = {0xF6,0x37,0x5D};    
//     rf_write_buf(W_REGISTER + RF_CAL,rf_cal_data,  sizeof(rf_cal_data));//设置RF_CAL
    xn297l_write_reg(W_REGISTER + CONFIG,0x8E);    // rf设置为发送模式
    clear_CE();
    delay_us(20);
}

/******************************************************************************/
//void rf_rx_mode(void);
//XN297L设置为接收模式
//input:无
//output:无
/******************************************************************************/
void xn297l_rx_mode(void)
{
//     uint8_t rf_cal_data[]    = {0x06,0x37,0x5D};    
//     rf_write_buf(W_REGISTER + RF_CAL,rf_cal_data,  sizeof(rf_cal_data));//设置RF_CAL 
    xn297l_write_reg(W_REGISTER + CONFIG,0X8F);    // rf设置为接收模式
    set_CE();
    //delay_ms(1);
}

/******************************************************************************/
//void rf_get_status(void);
//XN297L获取状态信息
//input:无
//output:状态寄存器的值
/******************************************************************************/
uint8_t xn297l_get_status(void)
{
    return xn297l_read_reg(RF_STATUS)&0x70;            //读取状态值
}

/******************************************************************************/
//void rf_clear_status(void);
//XN297L清除状态信息
//input:无
//output:无
/******************************************************************************/
void xn297l_clear_status(void)
{
    xn297l_write_reg(W_REGISTER + RF_STATUS,0x70);
}

/******************************************************************************/
//void rf_clear_FIFO(void);
//XN297L清除缓冲
//input:无
//output:无
/******************************************************************************/
void xn297l_clear_fifo(void)
{
    xn297l_write_reg(FLUSH_TX, 0);                //清除发送缓冲
    xn297l_write_reg(FLUSH_RX, 0);              //清除接收缓冲
}

/******************************************************************************/
//void rf_set_channel(uint8_t channel);
//XN297L设置通信信道
//input:uint8_t channel，设置的通信信道
//output:无
/******************************************************************************/
void xn297l_set_channel(uint8_t channel)
{
//    current_channel = channel;
    xn297l_write_reg(W_REGISTER + RF_CH, channel);
}
/******************************************************************************/
//void rf_set_addr(uint8_t reg,uint8_t *rf_addr_array);
//XN297L设置通信地址
//input:uint8_t reg,需要设置的寄存器，uint8_t *rf_addr_array，设置的地址
//output:无
/******************************************************************************/
void xn297l_set_addr(uint8_t reg,uint8_t *rf_addr_array,uint8_t addr_len)
{
    xn297l_write_buf(W_REGISTER + reg,rf_addr_array, addr_len);//设置地址
}
/******************************************************************************/
//uint8_t rf_tx_data(uint8_t *data, uint8_t length);
//XN297L发送数据
//input:uint8_t *data，发送数据数组的首地址；uint8_t length发送数据的长度
//output:发送状态值
/******************************************************************************/
void xn297l_tx_data(uint8_t *data, uint8_t length)
{
    set_CE();
    delay_us(100);
    xn297l_write_buf(W_TX_PAYLOAD,data,length);       //发送数据
    while(IRQ_STATUS);
    xn297l_clear_status();          //清除状态
    xn297l_write_reg(FLUSH_TX,0);                         //清除发送缓冲
    clear_CE();
}

/******************************************************************************/
//uint8_t rf_rx_data(uint8_t *data,uint8_t length);
//rf接收函数
//input:uint8_t *data,接收数据数组首地址；uint8_t length:接收数据的长度
//output:接收正确与否标志
/******************************************************************************/
uint8_t xn297l_rx_data(uint8_t *data,uint8_t length)
{
    if(IRQ_STATUS)
    {
      return 0;                                           //等待接收
    }
    xn297l_read_buf(R_RX_PAYLOAD,data,length);              //接收数据
    xn297l_write_reg(FLUSH_RX,0);                             //清除接收缓冲
    xn297l_clear_status();              //清除状态
    return 1;
}

/******************************************************************************/
//void rf_init(void);
//rf初始化函数，执行一些初始化操作
//input:无
//output:无
/******************************************************************************/
void rf_init(void)
{
    uint8_t addr[] = {0x23,0x23,0x23};
    rfspi_init();
    xn297l_init(addr,3,20,13,XN297L_RF_POWER_P_5|XN297L_RF_DATA_RATE_1M);
    xn297l_rx_mode();
}

void xn297l_init(uint8_t* addr,uint8_t addr_len,uint8_t chn,uint8_t payload,uint8_t rate_power)
{
#if ((DATA_RATE==DR_1M) || (DATA_RATE==DR_2M))
	uint8_t BB_cal_data[]    = {0x12,0xED,0x67,0x9C,0x46};
    uint8_t rf_cal_data[]    = {0xF6,0x3F,0x5D};
    uint8_t rf_cal2_data[]   = {0x45,0x21,0xEF,0x2C,0x5A,0x40};
    uint8_t Dem_cal_data[]   = {0x01};
    uint8_t Dem_cal2_data[]  = {0x0B,0xDF,0x02};
#elif (DATA_RATE==DR_250K)
	uint8_t BB_cal_data[]  	 = {0x12,0xEC,0x6F,0xA1,0x46};
    uint8_t rf_cal_data[]    = {0xF6,0x3F,0x5D};
    uint8_t rf_cal2_data[]   = {0xD5,0x21,0xEB,0x2C,0x5A,0x40};
    uint8_t Dem_cal_data[]   = {0x1F};
    uint8_t Dem_cal2_data[]  = {0x0B,0xDF,0x02};
#endif

    xn297l_write_reg(RST_FSPI,0x5A);								        //芯片软复位
    xn297l_write_reg(RST_FSPI,0XA5);
    delay_ms(1);
    xn297l_clear_fifo();//清除缓冲
    xn297l_clear_status();							    //清除状态寄存器
    if(payload < 33)
    {
        #if(CE_TYPE==CE_HARDWARE_MODE)
		xn297l_write_reg(W_REGISTER +FEATURE, 0x00);			//32字节数据，使能硬件CE
		#else
		xn297l_write_reg(W_REGISTER +FEATURE, 0x20);			//32字节数据，使能软件CE
		#endif
    }
    else
    {
        #if(CE_TYPE==CE_HARDWARE_MODE)
		xn297l_write_reg(W_REGISTER +FEATURE, 0x18);			//64字节数据，使能硬件CE
		#else
		xn297l_write_reg(W_REGISTER +FEATURE, 0x38);			//64字节数据，使能软件CE
		#endif
    }

    clear_CE();

    xn297l_write_reg(W_REGISTER + EN_RXADDR,0x03);              //使能接收通道0通道1
    xn297l_write_reg(W_REGISTER + SETUP_AW,addr_len-2);         //[1-3] = [3-5]
    xn297l_write_reg(W_REGISTER + RF_CH,chn);                   //设置接收信道
    xn297l_write_reg(W_REGISTER + RX_PW_P0,payload);            //设置接收数据宽度，即Payload宽度
	xn297l_write_reg(W_REGISTER + RX_PW_P1,payload);            //设置接收数据宽度，即Payload宽度
    xn297l_write_buf(W_REGISTER + TX_ADDR,addr, addr_len);      //设置发送地址
    xn297l_write_buf(W_REGISTER + RX_ADDR_P0,addr, addr_len);   //设置接收地址
    xn297l_write_buf(W_REGISTER + BB_CAL,BB_cal_data,  sizeof(BB_cal_data));    //设置BB_CAL
    xn297l_write_buf(W_REGISTER + RF_CAL2,rf_cal2_data, sizeof(rf_cal2_data));  //设置RF_CAL2
    xn297l_write_buf(W_REGISTER + DEM_CAL,Dem_cal_data, sizeof(Dem_cal_data));  //设置DEM_CAL
    xn297l_write_buf(W_REGISTER + RF_CAL,rf_cal_data,  sizeof(rf_cal_data));    //设置RF_CAL
    xn297l_write_buf(W_REGISTER + DEM_CAL2,Dem_cal2_data,sizeof(Dem_cal2_data));//设置DEM_CAL2
    xn297l_write_reg(W_REGISTER + DYNPD, 0x00);         //禁止动态Payload
    xn297l_write_reg(W_REGISTER + RF_SETUP,rate_power); //设置发射功率和速率
#if(TRANSMIT_TYPE == TRANS_ENHANCE_MODE)                //设置为增强模式
    xn297l_write_reg(W_REGISTER + SETUP_RETR,0x03);     //使能三次重发
    xn297l_write_reg(W_REGISTER + EN_AA,     0x01);     //使能自动应答
#elif(TRANSMIT_TYPE == TRANS_NORMAL_MODE)               //设置为普通模式
    xn297l_write_reg(W_REGISTER + SETUP_RETR,0x00);     //禁止重发
    xn297l_write_reg(W_REGISTER + EN_AA,     0x00);     //禁止自动应答
#endif
}
/******************************************************************************/
//单载波模式
/******************************************************************************/
void RF_Carrier( uint8_t ucChannel_Set)
{
    uint8_t BB_cal_data[]    = {0x0A,0x6D,0x67,0x9C,0x46}; 
    uint8_t RF_cal_data[]    = {0xF6,0x37,0x5D};
    uint8_t RF_cal2_data[]   = {0x45,0x21,0xEF,0xAC,0x5A,0x50};
    uint8_t Dem_cal_data[]   = {0xE1}; 								
    uint8_t Dem_cal2_data[]  = {0x0B,0xDF,0x02};      
    xn297l_write_reg(W_REGISTER + RF_CH,ucChannel_Set); 						
    xn297l_write_reg(W_REGISTER + RF_SETUP, 0x3F);      						//13dbm
    xn297l_write_buf(W_REGISTER + BB_CAL,    BB_cal_data,  sizeof(BB_cal_data));
    xn297l_write_buf(W_REGISTER + RF_CAL2,   RF_cal2_data, sizeof(RF_cal2_data));
    xn297l_write_buf(W_REGISTER + DEM_CAL,   Dem_cal_data, sizeof(Dem_cal_data));
    xn297l_write_buf(W_REGISTER + RF_CAL,    RF_cal_data,  sizeof(RF_cal_data));
    xn297l_write_buf(W_REGISTER + DEM_CAL2,  Dem_cal2_data,sizeof(Dem_cal2_data));
}

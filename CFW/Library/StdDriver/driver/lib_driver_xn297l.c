/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_xn297l.c
 * @brief     XN297L RF����
 *
 * @history - V1.0, 2017-03-06, xiaoguolin, first implementation.
*******************************************************************************/
#include "lib_driver_xn297l.h"

/******************************************************************************/
//void rf_write_reg(uint8_t reg,uint8_t data);
//rfд�Ĵ�������
//input:uint8_t reg���Ĵ�����ַ��uint8_t data:��Ҫд��Ĵ�����ֵ
//output:��
/******************************************************************************/
void xn297l_write_reg(uint8_t reg,uint8_t data)
{
    rfspi_cs(0);      //ʹ��Ƭѡ
    rfspi_rwc(reg);   //д��ļĴ����ĵ�ַ
    rfspi_rwc(data);  //д��ļĴ�����ֵ
    rfspi_cs(1);      //����Ƭѡ
}
/******************************************************************************/
//uint8_t rf_read_reg(uint8_t reg);
//rf���Ĵ�������
//input:uint8_t reg����Ҫ��ȡ�ļĴ���
//output:�Ĵ�����ֵ
/******************************************************************************/
uint8_t xn297l_read_reg(uint8_t reg)
{
    uint8_t tmp;
    rfspi_cs(0);              //ʹ��Ƭѡ
    rfspi_rwc(reg);           //д��ļĴ����ĵ�ַ
    tmp = rfspi_rwc(0x00);    //��ȡ�Ĵ�����ֵ
    rfspi_cs(1);              //����Ƭѡ
    return tmp;
}
/******************************************************************************/
//void rf_write_buf(uint8_t reg,uint8_t *pbuf,uint8_t length);
//rfд���庯��
//input:uint8_t reg��д��Ĵ����ĵ�ַ;uint8_t *pbuf��д��������׵�ַ;uint8_t length��д�����ݵĳ���
//output:��
/******************************************************************************/
void xn297l_write_buf(uint8_t reg,uint8_t *pbuf,uint8_t length)
{
    uint8_t i;
    rfspi_cs(0);                  //ʹ��Ƭѡ
    rfspi_rwc(reg);
    for(i = 0;i < length; i++)
    {
        rfspi_rwc(pbuf[i]);       //д������
    }
    rfspi_cs(1);                  //����Ƭѡ
}

/******************************************************************************/
//void rf_read_buf(uint8_t reg,uint8_t *pbuf,uint8_t length);
//rf�����庯��
//input:uint8_t reg�������Ĵ����ĵ�ַ;uint8_t *pbuf����������������׵�ַ;uint8_t length���������ݵĳ���
//output:��
/******************************************************************************/
void xn297l_read_buf(uint8_t reg, uint8_t *pbuf, uint8_t length)
{
    uint8_t i;
    rfspi_cs(0);                      //ʹ��Ƭѡ
    rfspi_rwc(reg);
    for(i = 0; i < length; i++)
    {
        pbuf[i] = rfspi_rwc(0x00);    //��ȡ����
    }
    rfspi_cs(1);                      //����Ƭѡ
}

/******************************************************************************/
//void rf_tx_mode(void);
//XN297L����Ϊ����ģʽ
//input:��
//output:��
/******************************************************************************/
void xn297l_tx_mode(void)
{
//     uint8_t rf_cal_data[]    = {0xF6,0x37,0x5D};    
//     rf_write_buf(W_REGISTER + RF_CAL,rf_cal_data,  sizeof(rf_cal_data));//����RF_CAL
    xn297l_write_reg(W_REGISTER + CONFIG,0x8E);    // rf����Ϊ����ģʽ
    clear_CE();
    delay_us(20);
}

/******************************************************************************/
//void rf_rx_mode(void);
//XN297L����Ϊ����ģʽ
//input:��
//output:��
/******************************************************************************/
void xn297l_rx_mode(void)
{
//     uint8_t rf_cal_data[]    = {0x06,0x37,0x5D};    
//     rf_write_buf(W_REGISTER + RF_CAL,rf_cal_data,  sizeof(rf_cal_data));//����RF_CAL 
    xn297l_write_reg(W_REGISTER + CONFIG,0X8F);    // rf����Ϊ����ģʽ
    set_CE();
    //delay_ms(1);
}

/******************************************************************************/
//void rf_get_status(void);
//XN297L��ȡ״̬��Ϣ
//input:��
//output:״̬�Ĵ�����ֵ
/******************************************************************************/
uint8_t xn297l_get_status(void)
{
    return xn297l_read_reg(RF_STATUS)&0x70;            //��ȡ״ֵ̬
}

/******************************************************************************/
//void rf_clear_status(void);
//XN297L���״̬��Ϣ
//input:��
//output:��
/******************************************************************************/
void xn297l_clear_status(void)
{
    xn297l_write_reg(W_REGISTER + RF_STATUS,0x70);
}

/******************************************************************************/
//void rf_clear_FIFO(void);
//XN297L�������
//input:��
//output:��
/******************************************************************************/
void xn297l_clear_fifo(void)
{
    xn297l_write_reg(FLUSH_TX, 0);                //������ͻ���
    xn297l_write_reg(FLUSH_RX, 0);              //������ջ���
}

/******************************************************************************/
//void rf_set_channel(uint8_t channel);
//XN297L����ͨ���ŵ�
//input:uint8_t channel�����õ�ͨ���ŵ�
//output:��
/******************************************************************************/
void xn297l_set_channel(uint8_t channel)
{
//    current_channel = channel;
    xn297l_write_reg(W_REGISTER + RF_CH, channel);
}
/******************************************************************************/
//void rf_set_addr(uint8_t reg,uint8_t *rf_addr_array);
//XN297L����ͨ�ŵ�ַ
//input:uint8_t reg,��Ҫ���õļĴ�����uint8_t *rf_addr_array�����õĵ�ַ
//output:��
/******************************************************************************/
void xn297l_set_addr(uint8_t reg,uint8_t *rf_addr_array,uint8_t addr_len)
{
    xn297l_write_buf(W_REGISTER + reg,rf_addr_array, addr_len);//���õ�ַ
}
/******************************************************************************/
//uint8_t rf_tx_data(uint8_t *data, uint8_t length);
//XN297L��������
//input:uint8_t *data����������������׵�ַ��uint8_t length�������ݵĳ���
//output:����״ֵ̬
/******************************************************************************/
void xn297l_tx_data(uint8_t *data, uint8_t length)
{
    set_CE();
    delay_us(100);
    xn297l_write_buf(W_TX_PAYLOAD,data,length);       //��������
    while(IRQ_STATUS);
    xn297l_clear_status();          //���״̬
    xn297l_write_reg(FLUSH_TX,0);                         //������ͻ���
    clear_CE();
}

/******************************************************************************/
//uint8_t rf_rx_data(uint8_t *data,uint8_t length);
//rf���պ���
//input:uint8_t *data,�������������׵�ַ��uint8_t length:�������ݵĳ���
//output:������ȷ����־
/******************************************************************************/
uint8_t xn297l_rx_data(uint8_t *data,uint8_t length)
{
    if(IRQ_STATUS)
    {
      return 0;                                           //�ȴ�����
    }
    xn297l_read_buf(R_RX_PAYLOAD,data,length);              //��������
    xn297l_write_reg(FLUSH_RX,0);                             //������ջ���
    xn297l_clear_status();              //���״̬
    return 1;
}

/******************************************************************************/
//void rf_init(void);
//rf��ʼ��������ִ��һЩ��ʼ������
//input:��
//output:��
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

    xn297l_write_reg(RST_FSPI,0x5A);								        //оƬ��λ
    xn297l_write_reg(RST_FSPI,0XA5);
    delay_ms(1);
    xn297l_clear_fifo();//�������
    xn297l_clear_status();							    //���״̬�Ĵ���
    if(payload < 33)
    {
        #if(CE_TYPE==CE_HARDWARE_MODE)
		xn297l_write_reg(W_REGISTER +FEATURE, 0x00);			//32�ֽ����ݣ�ʹ��Ӳ��CE
		#else
		xn297l_write_reg(W_REGISTER +FEATURE, 0x20);			//32�ֽ����ݣ�ʹ�����CE
		#endif
    }
    else
    {
        #if(CE_TYPE==CE_HARDWARE_MODE)
		xn297l_write_reg(W_REGISTER +FEATURE, 0x18);			//64�ֽ����ݣ�ʹ��Ӳ��CE
		#else
		xn297l_write_reg(W_REGISTER +FEATURE, 0x38);			//64�ֽ����ݣ�ʹ�����CE
		#endif
    }

    clear_CE();

    xn297l_write_reg(W_REGISTER + EN_RXADDR,0x03);              //ʹ�ܽ���ͨ��0ͨ��1
    xn297l_write_reg(W_REGISTER + SETUP_AW,addr_len-2);         //[1-3] = [3-5]
    xn297l_write_reg(W_REGISTER + RF_CH,chn);                   //���ý����ŵ�
    xn297l_write_reg(W_REGISTER + RX_PW_P0,payload);            //���ý������ݿ�ȣ���Payload���
	xn297l_write_reg(W_REGISTER + RX_PW_P1,payload);            //���ý������ݿ�ȣ���Payload���
    xn297l_write_buf(W_REGISTER + TX_ADDR,addr, addr_len);      //���÷��͵�ַ
    xn297l_write_buf(W_REGISTER + RX_ADDR_P0,addr, addr_len);   //���ý��յ�ַ
    xn297l_write_buf(W_REGISTER + BB_CAL,BB_cal_data,  sizeof(BB_cal_data));    //����BB_CAL
    xn297l_write_buf(W_REGISTER + RF_CAL2,rf_cal2_data, sizeof(rf_cal2_data));  //����RF_CAL2
    xn297l_write_buf(W_REGISTER + DEM_CAL,Dem_cal_data, sizeof(Dem_cal_data));  //����DEM_CAL
    xn297l_write_buf(W_REGISTER + RF_CAL,rf_cal_data,  sizeof(rf_cal_data));    //����RF_CAL
    xn297l_write_buf(W_REGISTER + DEM_CAL2,Dem_cal2_data,sizeof(Dem_cal2_data));//����DEM_CAL2
    xn297l_write_reg(W_REGISTER + DYNPD, 0x00);         //��ֹ��̬Payload
    xn297l_write_reg(W_REGISTER + RF_SETUP,rate_power); //���÷��书�ʺ�����
#if(TRANSMIT_TYPE == TRANS_ENHANCE_MODE)                //����Ϊ��ǿģʽ
    xn297l_write_reg(W_REGISTER + SETUP_RETR,0x03);     //ʹ�������ط�
    xn297l_write_reg(W_REGISTER + EN_AA,     0x01);     //ʹ���Զ�Ӧ��
#elif(TRANSMIT_TYPE == TRANS_NORMAL_MODE)               //����Ϊ��ͨģʽ
    xn297l_write_reg(W_REGISTER + SETUP_RETR,0x00);     //��ֹ�ط�
    xn297l_write_reg(W_REGISTER + EN_AA,     0x00);     //��ֹ�Զ�Ӧ��
#endif
}
/******************************************************************************/
//���ز�ģʽ
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

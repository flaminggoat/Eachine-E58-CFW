/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_xn297l.h
 * @brief     XN297L RF����
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

#define     TRANS_ENHANCE_MODE      1                               //������ǿģʽ
#define     TRANS_NORMAL_MODE       2                               //������ͨģʽ
#define     TRANSMIT_TYPE           TRANS_NORMAL_MODE               //��ʼ����Ϊ��ͨģʽ

#define    	RX_DR_FLAG             	0x40                            // �����жϱ�־λ
#define    	TX_DS_FLAG            	0x20                            // ��������жϱ�־λ
#define    	RX_TX_FLAG            	0x60                            // ���ͽ�������жϱ�־λ��ack_payload ģʽ��ʹ��
#define   	MAX_RT_FLAG           	0x10                            // �����ش���ʱ�жϱ�־λ

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

#define         	rf7dBm                         	0X0D  // 7dBm ֻ������1M��������
#define         	rf6dBm                         	0X06  // 6dBm ֻ������1M��������
#define         	rf3dBm                         	0X0C  // 3dBm ֻ������1M��������

#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//XN297L�Ĵ�����������
#define     RF_READ_REG             0x00                            //�����üĴ���,��5λΪ�Ĵ�����ַ
#define     RF_WRITE_REG            0x20                            //д���üĴ���,��5λΪ�Ĵ�����ַ
#define     W_REGISTER		        0x20                            //д���üĴ���,��5λΪ�Ĵ�����ַ
#define     R_RX_PAYLOAD            0x61                            //��RX��Ч����, 1~32�ֽ�
#define     W_TX_PAYLOAD            0xA0                            //дTX��Ч����,1~32�ֽ�
#define     FLUSH_TX                0xE1                            //���TX FIFO�Ĵ���.����ģʽ����
#define     FLUSH_RX                0xE2                            //���RX FIFO�Ĵ���.����ģʽ����
#define     REUSE_TX_PL             0xE3                            //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define     NOP                     0xFF                            //�ղ���,����������״̬�Ĵ���


#define		ACTIVATE			    0x50                            //ACTIVATE
#define		DEACTIVATE			    0x50                            //DEACTIVATE
#define		R_RX_PL_WID			    0x60                            //Read width of RX data
#define		W_ACK_PAYLOAD		    0xA8                            //Data with ACK
#define		W_TX_PAYLOAD_NOACK	    0xB0                            //TX Payload no ACK Request
#define		CE_FSPI_ON	            0xFD                            // CE HIGH
#define		CE_FSPI_OFF	            0xFC                            // CE LOW
#define		RST_FSPI	            0x53                            // RESET


//SPI(XN297L)�Ĵ�����ַ
#define     CONFIG                  0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
//bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define     EN_AA                   0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define     EN_RXADDR               0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define     SETUP_AW                0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define     SETUP_RETR              0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define     RF_CH                   0x05  //rfͨ��,bit6:0,����ͨ��Ƶ��;
#define     RF_SETUP                0x06  //rf�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define     RF_STATUS                  0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
//bit5:���ݷ�������ж�;bit6:���������ж�;
#define     MAX_TX  		        0x10  //�ﵽ����ʹ����ж�
#define     TX_OK   		        0x20  //TX��������ж�
#define     RX_OK   		        0x40  //���յ������ж�

#define     OBSERVE_TX              0x08  //����״̬�Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define     CD                      0x09  //�ز����Ĵ���,bit0,�ز����;
#define     RX_ADDR_P0              0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define     RX_ADDR_P1              0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define     RX_ADDR_P2              0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define     RX_ADDR_P3              0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define     RX_ADDR_P4              0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define     RX_ADDR_P5              0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define     TX_ADDR                 0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define     RX_PW_P0                0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define     RX_PW_P1                0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define     RX_PW_P2                0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define     RX_PW_P3                0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define     RX_PW_P4                0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define     RX_PW_P5                0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define     FIFO_STATUS             0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
//bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;

#define		DEM_CAL				    0x19
#define     RF_CAL2				    0x1A
#define     DEM_CAL2			    0x1B
#define		DYNPD			        0x1C
#define		FEATURE				    0x1D
#define		RF_CAL				    0x1E
#define		BB_CAL				    0x1F


#define     USER_CHN   	    		CONF_RF_REMOTE_CHN              //ͨ���ŵ�����
#define     MONITOR_CHN   	    	CONF_RF_DONGLEX_CHN             //���ͨ���ŵ�����
#define     PAYLOAD_WIDTH           CONF_RF_REMOTE_RX_PAYLOAD_WIDTH //XN297L���ͽ������ݿ�ȶ���
#define 	MONITOR_WIDTH 			CONF_RF_DONGLEX_TX_PAYLOAD_WIDTH
//#define     TX_ADR_WIDTH            5   	//5�ֽڵĵ�ַ���
//#define     RX_ADR_WIDTH            5   	//5�ֽڵĵ�ַ���
//#define     TX_PLOAD_WIDTH          32  	//32�ֽڵ��û����ݿ��
//#define     RX_PLOAD_WIDTH          32  	//32�ֽڵ��û����ݿ��

#define     CE_SOFTWARE_MODE      	1                             //CE���ģʽ
#define     CE_HARDWARE_MODE       	2                             //CEӲ��ģʽ
#define     CE_TYPE           		CE_SOFTWARE_MODE              //����ΪӲ��ģʽ

#define IRQ_STATUS  rfspi_irq()
#define CSN_HIGH   P01=1 
#define CSN_LOW   P01=0

//#if 0
////PAN163
//#if(CE_TYPE==CE_HARDWARE_MODE)
//	#define set_CE()   	(P02=1)
//	#define clear_CE()  (P02=0)
//#else
//	#define     set_CE()                (xn297l_write_reg(CE_FSPI_ON, 0))//�ڲ���λCE
//	#define     clear_CE()              (xn297l_write_reg(CE_FSPI_OFF, 0))//�ڲ�����CE
//#endif
//#endif
////PAN159
//#define     set_CE()                (xn297l_write_reg(CE_FSPI_ON, 0))//�ڲ���λCE
//#define     clear_CE()              (xn297l_write_reg(CE_FSPI_OFF, 0))//�ڲ�����CE


#if(CE_TYPE==CE_HARDWARE_MODE)
	#define set_CE()   	rfspi_ce(1)
	#define clear_CE()  rfspi_ce(0)
#else
	#define set_CE()    (xn297l_write_reg(CE_FSPI_ON, 0))//�ڲ���λCE
	#define clear_CE()  (xn297l_write_reg(CE_FSPI_OFF, 0))//�ڲ�����CE
#endif

void    xn297l_write_reg(uint8_t reg,uint8_t data);//rfд�Ĵ�������
uint8_t xn297l_read_reg(uint8_t reg);//rf���Ĵ�������
void    xn297l_write_buf(uint8_t reg,uint8_t *pbuf,uint8_t length);//rfд���庯��
void    xn297l_read_buf(uint8_t reg,uint8_t *pbuf,uint8_t length);//rf�����庯��
//void    xn297l_init(void);//rf��ʼ��
void    xn297l_init(uint8_t* addr,uint8_t addr_len,uint8_t chn,uint8_t payload,uint8_t rate_power);
void    xn297l_tx_mode(void);//XN297L����Ϊ����ģʽ
void    xn297l_rx_mode(void);//XN297L����Ϊ����ģʽ
uint8_t xn297l_get_status(void);//XN297L��ȡ״̬��Ϣ
void    xn297l_clear_status(void);//XN297L���״̬��Ϣ
void    xn297l_clear_fifo(void);//XN297L�������
void    xn297l_set_channel(uint8_t channel);//XN297L����ͨ���ŵ�
void    xn297l_tx_data(uint8_t *data, uint8_t length);//XN297L��������
uint8_t xn297l_rx_data(uint8_t *data,uint8_t length);//rf���պ���
void    xn297l_set_addr(uint8_t reg,uint8_t *rf_addr_array,uint8_t addr_len);//rf����ͨ�ŵ�ַ
void rf_init(void);

#ifdef __cplusplus
}
#endif

#endif // __DRV_XN297L_H

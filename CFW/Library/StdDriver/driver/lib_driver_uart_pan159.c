/*******************************************************************************
 * @note      Copyright (C) 2018 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      lib_driver_uart_pan159.c
 * @brief     PAN159 hardware uartÇý¶¯
 *
 * @history - V1.0, 2018-01-19, huoweibin, first implementation.
*******************************************************************************/
#include "lib_driver_uart_pan159.h"
#include "string.h"

uint8_t uartRcvBuffer[RX_BUF_SIZE],wptr,u8InChar,uartTxBuffer[TX_BUF_SIZE];
uint16_t  u16Count;
uint16_t  u16TxDataLen;
/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint32_t u32IntSts= UART0->INTSTS;
    if(u32IntSts & UART_INTSTS_RDAINT_Msk) {
        /* Get all the input characters */
        while(UART_IS_RX_READY(UART0)) {
            /* Get the character from UART Buffer */
            if(wptr < RX_BUF_SIZE) {
                uartRcvBuffer[wptr++] = UART_READ(UART0);   
            }
            else{
                wptr = 0;
                uartRcvBuffer[wptr] = UART_READ(UART0);
            }
        }
    }
    if(u32IntSts & UART_INTSTS_THREINT_Msk) {
        if(u16Count < u16TxDataLen && u16TxDataLen != 0){
            u8InChar = uartTxBuffer[u16Count++];
            UART_WRITE(UART0,u8InChar);
            if(u16Count >= u16TxDataLen){
                u16TxDataLen = 0;
                u16Count = 0;
                UART_DISABLE_INT(UART0,UART_INTEN_THREIEN_Msk);
            }
        }
    }
}


void uart_init_pan159(void)
{
    CLK_EnableModuleClock(UART0_MODULE);
//    SYS->P4_MFP = SYS_MFP_P46_UART1_RXD | SYS_MFP_P47_UART1_TXD;
    SYS->P5_MFP = SYS_MFP_P51_UART0_RXD | SYS_MFP_P50_UART0_TXD;
	GPIO_ENABLE_DIGITAL_PATH(P5,(1<<1));	
//    UART_Open(UART1, 115200);
    UART_Open(UART0, 115200);
    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk /*| UART_INTEN_THREIEN_Msk*/));
    NVIC_EnableIRQ(UART0_IRQn);
}

void uart_send(uint8_t *buff,uint16_t len)
{
    u16TxDataLen = len;
    u16Count = 0;
    memcpy(&uartTxBuffer[0],&buff[0],len);
    UART_ENABLE_INT(UART0,UART_INTEN_THREIEN_Msk);
}



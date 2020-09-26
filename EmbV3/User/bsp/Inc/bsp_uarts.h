/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp_uarts.h
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2017-12-20
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2017/12/20 | 0.0.1 | lingping | Create file
*
*/
#ifndef _BSP_UARTS_H_
#define _BSP_UARTS_H_


#define	UART8_FIFO_EN	1

typedef enum
{
	COM1 = 0,	// USART1 //
	COM2 = 1,	// UART4  //
	COM3 = 2,	// USART3 //
	COM4 = 3,	// UART6  //
	COM5 = 4,	// UART5  //
	COM6 = 5,	// USART2 //
	COM7 = 6,	// UART7  //
	COM8 = 7,	// UART8  //
}COM_PORT_E;

#define UART1_BAUD			921600//256000
#define UART2_BAUD			256000
#define UART3_BAUD			256000
#define UART4_BAUD			256000
#define UART5_BAUD			57142//57142  //256000  //250000
#define UART6_BAUD			256000

#define UART7_BAUD			9600//9600//115200
#define UART7_TX_BUF_SIZE	1*512
#define UART7_RX_BUF_SIZE	1*512

#if UART8_FIFO_EN == 1
#define UART8_BAUD			9600
#define UART8_TX_BUF_SIZE	1*512
#define UART8_RX_BUF_SIZE	1*512
#endif

typedef struct
{
	USART_TypeDef *uart;
	u8 *pTxBuf;
	u8 *pRxBuf;
	u16 usTxBufSize;
	u16 usRxBufSize;
	u16 usTxWrite;
	u16 usTxRead;
	u16 usTxCount;

	u16 usRxWrite;
	u16 usRxRead;
	u16 usRxCount;

}UART_T;



void BspUarts_init(void);
void comSendBuf(COM_PORT_E _ucPort, u8 *_ucaBuf, u16 _usLen);
UART_T *ComToUart(COM_PORT_E _ucPort);

#endif

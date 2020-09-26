/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp_uart_dma.h
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
#ifndef _BSP_UART_DMA_H_
#define _BSP_UART_DMA_H_


#define PRINTF_TX_BUFF_CNT 2
#define PRINTF_RX_BUFF_CNT 2

#define UART_TX_BUFF_CNT 2
#define UART_RX_BUFF_CNT 2


#define UART_TX_BUFFER_SIZE    256
#define UART_RX_BUFFER_SIZE    256
#define COM_RX_BUFFER_SIZE     512
#define COM_RX_BUFFER_SINGLE_OP_SIZE 256

#define MAX_DMA_COM_NUMS            6
/*
typedef struct{
    COM_PORT_E port;          //!< COMx
    USART_TypeDef * uartx;
    u32 chbps;           //!< 通道波特率
    u8 uartIrq;          //!< 串口中断号
	u8 subPrio;          //!< 串口中断sub优先级
	u8 * pSend;          //!< dma send buffer pointer
	u8   tbsy;            
	DMA_Stream_TypeDef* ptStream;
	u32 dmatChl;
	IRQn_Type dmatchlirq;
	u8 dmatsubPrio;      //!< dma tx channel sub priority
	u8 * pRecv0;          //!< dma recieve buffer0 pointer	
	u8 * pRecv1;          //!< dma recieve buffer1 pointer
	DMA_Stream_TypeDef* prStream;
	u32 dmarChl;
	IRQn_Type rchlirq;
	u32 dma_ttcflag;
	u32 dma_tteflag;
	u32 dma_rtcflag;
	u32 dma_rteflag;
}COMSENDSTRUCT;
*/




//!< uart printf transfer ctrl struct
typedef struct{
	u8* pSnd[PRINTF_TX_BUFF_CNT];
	u8* pRev[PRINTF_RX_BUFF_CNT];
	u16 SndCnt[PRINTF_TX_BUFF_CNT];
	u8  bsy[PRINTF_TX_BUFF_CNT];                              //!< bit x,channel x        1:busy,0:idle
	u8 CurBuf;
}BUFFERSENDCTRL;

typedef struct{
    COM_PORT_E port;          //!< COMx
    USART_TypeDef * uartx;
    u32 chbps;           //!< 通道波特率
    u8 uartIrq;          //!< 串口中断号
	u8 subPrio;          //!< 串口中断sub优先级
	/////////////////u8 * pSend[UART_TX_BUFF_CNT];          //!< dma send buffer pointer
	//////////////////u8   tbsy[UART_TX_BUFF_CNT];   
	BUFFERSENDCTRL sCtrl;
	DMA_Stream_TypeDef* ptStream;
	u32 dmatChl;
	IRQn_Type dmatchlirq;
	u8 dmatsubPrio;      //!< dma tx channel sub priority
	u8 * pRecv0;          //!< dma recieve buffer0 pointer	
	u8 * pRecv1;          //!< dma recieve buffer1 pointer
	DMA_Stream_TypeDef* prStream;
	u32 dmarChl;
	IRQn_Type rchlirq;
	u32 dma_ttcflag;
	u32 dma_tteflag;
	u32 dma_rtcflag;
	u32 dma_rteflag;
}COMSENDSTRUCT;



typedef struct{
	u8* pBuffer;
    u8* pRxRead;
	u8* pRxWrite;
	u16 RxCount;
}COMRECVSTRUCT;


void BspDmaUart_init(void);
s8 UartDmaTransferConfig(COM_PORT_E port,u8 *pSndBuf,u16 len);



#endif

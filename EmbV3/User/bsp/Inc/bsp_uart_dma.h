/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file bsp_uart_dma.h
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
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
    u32 chbps;           //!< ͨ��������
    u8 uartIrq;          //!< �����жϺ�
	u8 subPrio;          //!< �����ж�sub���ȼ�
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
    u32 chbps;           //!< ͨ��������
    u8 uartIrq;          //!< �����жϺ�
	u8 subPrio;          //!< �����ж�sub���ȼ�
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

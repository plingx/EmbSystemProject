/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp_uart_dma.c
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
#include "bsp.h"

//!< Printf Uart DMA transfer buffer 
u8 UART1_DMA_TX0[UART_TX_BUFFER_SIZE]={0};
u8 UART1_DMA_TX1[UART_TX_BUFFER_SIZE]={0};
u8 UART1_DMA_RX0[UART_RX_BUFFER_SIZE]={0};
u8 UART1_DMA_RX1[UART_RX_BUFFER_SIZE]={0};
u8 UART1_RX[COM_RX_BUFFER_SIZE]={0};

//!< UART X DMA transfer buffer  1280bytes
u8 UART2_DMA_TX0[UART_TX_BUFFER_SIZE]={0};
u8 UART2_DMA_TX1[UART_TX_BUFFER_SIZE]={0};
u8 UART2_DMA_RX0[UART_RX_BUFFER_SIZE]={0};
u8 UART2_DMA_RX1[UART_RX_BUFFER_SIZE]={0};
u8 UART2_RX[COM_RX_BUFFER_SIZE]={0};

u8 UART3_DMA_TX0[UART_TX_BUFFER_SIZE]={0};
u8 UART3_DMA_TX1[UART_TX_BUFFER_SIZE]={0};
u8 UART3_DMA_RX0[UART_RX_BUFFER_SIZE]={0};
u8 UART3_DMA_RX1[UART_RX_BUFFER_SIZE]={0};
u8 UART3_RX[COM_RX_BUFFER_SIZE]={0};

u8 UART4_DMA_TX0[UART_TX_BUFFER_SIZE]={0};
u8 UART4_DMA_TX1[UART_TX_BUFFER_SIZE]={0};
u8 UART4_DMA_RX0[UART_RX_BUFFER_SIZE]={0};
u8 UART4_DMA_RX1[UART_RX_BUFFER_SIZE]={0};
u8 UART4_RX[COM_RX_BUFFER_SIZE]={0};

u8 UART5_DMA_TX0[UART_TX_BUFFER_SIZE]={0};
u8 UART5_DMA_TX1[UART_TX_BUFFER_SIZE]={0};
u8 UART5_DMA_RX0[UART_RX_BUFFER_SIZE]={0};
u8 UART5_DMA_RX1[UART_RX_BUFFER_SIZE]={0};
u8 UART5_RX[COM_RX_BUFFER_SIZE]={0};

u8 UART6_DMA_TX0[UART_TX_BUFFER_SIZE]={0};
u8 UART6_DMA_TX1[UART_TX_BUFFER_SIZE]={0};
u8 UART6_DMA_RX0[UART_RX_BUFFER_SIZE]={0};
u8 UART6_DMA_RX1[UART_RX_BUFFER_SIZE]={0};
u8 UART6_RX[COM_RX_BUFFER_SIZE]={0};

//!< struct of Uart X(COM X) interface configuration & DMA transfer configuration
COMSENDSTRUCT ComTrans[MAX_DMA_COM_NUMS]={
                         	{COM1,USART1,UART1_BAUD,USART1_IRQn,UART1_SUB_PRIO_VALUE,{{UART1_DMA_TX0,UART1_DMA_TX1},{UART1_DMA_RX0,UART1_DMA_RX1},{0,0},{0,0},0},DMA2_Stream7,DMA_Channel_4,DMA2_Stream7_IRQn,DMA2T_ST7_SUB_PRIO_VALUE,UART1_DMA_RX0,UART1_DMA_RX1,DMA2_Stream2,DMA_Channel_4,DMA2_Stream2_IRQn,DMA_IT_TCIF7,DMA_IT_TEIF7,DMA_FLAG_TCIF2,DMA_FLAG_TEIF2},
							{COM2,UART4,UART4_BAUD,UART4_IRQn,UART4_SUB_PRIO_VALUE,{{UART4_DMA_TX0,UART4_DMA_TX1},{UART4_DMA_RX0,UART4_DMA_RX1},{0,0},{0,0},0},DMA1_Stream4,DMA_Channel_4,DMA1_Stream4_IRQn,DMA1T_ST4_SUB_PRIO_VALUE,UART4_DMA_RX0,UART4_DMA_RX1,DMA1_Stream2,DMA_Channel_4,DMA1_Stream2_IRQn,DMA_IT_TCIF4,DMA_IT_TEIF4,DMA_FLAG_TCIF2,DMA_FLAG_TEIF2},
                         	{COM3,USART3,UART3_BAUD,USART3_IRQn,UART3_SUB_PRIO_VALUE,{{UART3_DMA_TX0,UART3_DMA_TX1},{UART3_DMA_RX0,UART3_DMA_RX1},{0,0},{0,0},0},DMA1_Stream3,DMA_Channel_4,DMA1_Stream3_IRQn,DMA1T_ST3_SUB_PRIO_VALUE,UART3_DMA_RX0,UART3_DMA_RX1,DMA1_Stream1,DMA_Channel_4,DMA1_Stream1_IRQn,DMA_IT_TCIF3,DMA_IT_TEIF3,DMA_FLAG_TCIF1,DMA_FLAG_TEIF1},
                         	{COM4,USART6,UART6_BAUD,USART6_IRQn,UART6_SUB_PRIO_VALUE,{{UART6_DMA_TX0,UART6_DMA_TX1},{UART6_DMA_RX0,UART6_DMA_RX1},{0,0},{0,0},0},DMA2_Stream6,DMA_Channel_5,DMA2_Stream6_IRQn,DMA2T_ST6_SUB_PRIO_VALUE,UART6_DMA_RX0,UART6_DMA_RX1,DMA2_Stream1,DMA_Channel_5,DMA2_Stream1_IRQn,DMA_IT_TCIF6,DMA_IT_TEIF6,DMA_FLAG_TCIF1,DMA_FLAG_TEIF1},
                         	{COM5,UART5,UART5_BAUD,UART5_IRQn,UART5_SUB_PRIO_VALUE,{{UART5_DMA_TX0,UART5_DMA_TX1},{UART5_DMA_RX0,UART5_DMA_RX1},{0,0},{0,0},0},DMA1_Stream7,DMA_Channel_4,DMA1_Stream7_IRQn,DMA1T_ST7_SUB_PRIO_VALUE,UART5_DMA_RX0,UART5_DMA_RX1,DMA1_Stream0,DMA_Channel_4,DMA1_Stream0_IRQn,DMA_IT_TCIF7,DMA_IT_TEIF7,DMA_FLAG_TCIF0,DMA_FLAG_TEIF0},
                         	{COM6,USART2,UART2_BAUD,USART2_IRQn,UART2_SUB_PRIO_VALUE,{{UART2_DMA_TX0,UART2_DMA_TX1},{UART2_DMA_RX0,UART2_DMA_RX1},{0,0},{0,0},0},DMA1_Stream6,DMA_Channel_4,DMA1_Stream6_IRQn,DMA1T_ST6_SUB_PRIO_VALUE,UART2_DMA_RX0,UART2_DMA_RX1,DMA1_Stream5,DMA_Channel_4,DMA1_Stream5_IRQn,DMA_IT_TCIF6,DMA_IT_TEIF6,DMA_FLAG_TCIF5,DMA_FLAG_TEIF5},
                         };

//!< struct of Uart X Dma receive buffer and control
COMRECVSTRUCT ComRecv[MAX_DMA_COM_NUMS]={
                                     {UART1_RX,UART1_RX,UART1_RX,0},      //!< COM1  COM_RX_BUFFER_SIZE=512 bytes
                                     {UART4_RX,UART4_RX,UART4_RX,0},      //!< COM2
                                     {UART3_RX,UART3_RX,UART3_RX,0},      //!< COM3
                                     {UART6_RX,UART6_RX,UART6_RX,0},      //!< COM4
                                     {UART5_RX,UART5_RX,UART5_RX,0},      //!< COM5
                                     {UART2_RX,UART2_RX,UART2_RX,0}       //!< COM6
};


BUFFERSENDCTRL PrintTransCtrl={{UART1_DMA_TX0,UART1_DMA_TX1},{UART1_DMA_RX0,UART1_DMA_RX1},{0,0},{0,0},0};


void DMAx_Streamy_Tx_IRQHandlers(COM_PORT_E com);
void DMAx_Streamy_Rx_IRQHandlers(COM_PORT_E com);
void UARTx_IRQHanders(COM_PORT_E com);


/* --------------------------------------------------------------------------*/
/**
* @name DMAx_Streamy_Tx_IRQHandlers
* @brief 
* @details Uart X DMAx_Streamy Transmit interrupt
*
* @param[in] port COMx
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
/*void DMAx_Streamy_Tx_IRQHandlers(COM_PORT_E com)               //!< COMx dma tx interrupt 
{
	if (DMA_GetITStatus(ComTrans[com].ptStream,ComTrans[com].dma_ttcflag) != RESET)//!< TCIF transfer complete interrupt flag
	{		
		DMA_ClearITPendingBit(ComTrans[com].ptStream, ComTrans[com].dma_ttcflag);
					
		DMA_Cmd(ComTrans[com].ptStream,DISABLE);							 //!<  stop DMA transmit
	
		ComTrans[com].tbsy = 0;
	}			
							
	if (DMA_GetITStatus(ComTrans[com].ptStream, ComTrans[com].dma_tteflag) != RESET)//!< TEIF transfer error interrupt flag
	{
		DMA_ClearITPendingBit(ComTrans[com].ptStream, ComTrans[com].dma_tteflag);
	}
}
*/

void DMAx_Streamy_Tx_IRQHandlers(COM_PORT_E com)               //!< COMx dma tx interrupt 
{
	u8 i;

	BUFFERSENDCTRL* psCtrl = &ComTrans[com].sCtrl;
	
	if (DMA_GetITStatus(ComTrans[com].ptStream,ComTrans[com].dma_ttcflag) != RESET)//!< TCIF transfer complete interrupt flag
	{		
		DMA_ClearITPendingBit(ComTrans[com].ptStream, ComTrans[com].dma_ttcflag);
						
		//DMA_Cmd(ComTrans[COM1].ptStream,DISABLE); 		//!<  stop DMA transmit,enter interrupt already stoped.
	
		i = psCtrl->CurBuf;
		psCtrl->bsy[i] = 0;
		psCtrl->SndCnt[i] = 0;
		psCtrl->CurBuf = 0;
	
		for(i = 0;i<UART_TX_BUFF_CNT;i++)
		{
			if(psCtrl->SndCnt[i]!=0)
				break;
		}
	
		if(i<UART_TX_BUFF_CNT)
		{
			psCtrl->CurBuf = i;	
		
			psCtrl->bsy[i] = 1;
	
			(ComTrans[com].ptStream)->M0AR = (u32)psCtrl->pSnd[i];
	
			DMA_SetCurrDataCounter(ComTrans[com].ptStream,psCtrl->SndCnt[i]);
			
			DMA_Cmd(ComTrans[com].ptStream,ENABLE); 
		}
	}			
								
	if (DMA_GetITStatus(ComTrans[com].ptStream, ComTrans[com].dma_tteflag) != RESET)//!< TEIF transfer error interrupt flag
	{
		DMA_ClearITPendingBit(ComTrans[com].ptStream, ComTrans[com].dma_tteflag);
	}

}




/* --------------------------------------------------------------------------*/
/**
* @name DMAx_Streamy_Rx_IRQHandlers
* @brief 
* @details Uart X DMAx_Streamy Received Data interrupt,in normal receive mode
* can not call this function unless received buffer overflow at one time.
*
* @param[in] port COMx
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMAx_Streamy_Rx_IRQHandlers(COM_PORT_E com)               //!< COMx dma rx interrupt 
{
    if (DMA_GetITStatus(ComTrans[com].prStream, ComTrans[com].dma_rtcflag) != RESET)    //!< transfer complete flag
    {
        DMA_ClearITPendingBit(ComTrans[com].prStream, ComTrans[com].dma_rtcflag);
	}

	if (DMA_GetITStatus(ComTrans[com].prStream, ComTrans[com].dma_rteflag) != RESET)    //!< transfer error flag
    {
        DMA_ClearITPendingBit(ComTrans[com].prStream, ComTrans[com].dma_rteflag);
	}
}



/* --------------------------------------------------------------------------*/
/**
* @name DMA2_Stream7_IRQHandler
* @brief 
* @details COM1(UART1) DMA Tx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA2_Stream7_IRQHandler(void)                     //!< usart1 tx
{
    u8 i;
	if (DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) != RESET)//!< TCIF transfer complete interrupt flag
	{		
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
					
		//DMA_Cmd(ComTrans[COM1].ptStream,DISABLE);			//!<  stop DMA transmit,enter interrupt already stoped.

		i = PrintTransCtrl.CurBuf;
		PrintTransCtrl.bsy[i] = 0;
		PrintTransCtrl.SndCnt[i] = 0;
		PrintTransCtrl.CurBuf = 0;

		for(i = 0;i<PRINTF_TX_BUFF_CNT;i++)
		{
			if(PrintTransCtrl.SndCnt[i]!=0)
				break;
		}

		if(i<PRINTF_TX_BUFF_CNT)
		{
			PrintTransCtrl.CurBuf = i;	
	
			PrintTransCtrl.bsy[i] = 1;

			(DMA2_Stream7)->M0AR = (u32)PrintTransCtrl.pSnd[i];

			DMA_SetCurrDataCounter(DMA2_Stream7,PrintTransCtrl.SndCnt[i]);
		
			DMA_Cmd(DMA2_Stream7,ENABLE); 
		}
	}			
							
	if (DMA_GetITStatus(DMA2_Stream7, ComTrans[COM1].dma_tteflag) != RESET)//!< TEIF transfer error interrupt flag
	{
		DMA_ClearITPendingBit(DMA2_Stream7, ComTrans[COM1].dma_tteflag);
	}
}

/* --------------------------------------------------------------------------*/
/**
* @name DMA2_Stream2_IRQHandler
* @brief 
* @details COM1(UART1) DMA Rx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA2_Stream2_IRQHandler(void)                     //!< usart1 rx
{
	DMAx_Streamy_Rx_IRQHandlers(COM1);
}


/* --------------------------------------------------------------------------*/
/**
* @name DMA1_Stream6_IRQHandler
* @brief 
* @details COM6(UART2) DMA Tx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA1_Stream6_IRQHandler(void)                     //!< usart2 tx
{
	DMAx_Streamy_Tx_IRQHandlers(COM6);
}


/* --------------------------------------------------------------------------*/
/**
* @name DMA1_Stream5_IRQHandler
* @brief 
* @details COM6(UART2) DMA Rx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA1_Stream5_IRQHandler(void)                    //!< usart2 rx
{
	DMAx_Streamy_Rx_IRQHandlers(COM6);
}


/* --------------------------------------------------------------------------*/
/**
* @name DMA1_Stream3_IRQHandler
* @brief 
* @details COM3(UART3) DMA Tx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA1_Stream3_IRQHandler(void)                     //!< usart3 tx
{
	DMAx_Streamy_Tx_IRQHandlers(COM3);		
}

/* --------------------------------------------------------------------------*/
/**
* @name DMA1_Stream1_IRQHandler
* @brief 
* @details COM3(UART3) DMA Rx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA1_Stream1_IRQHandler(void)                    //!< usart3 rx
{
    DMAx_Streamy_Rx_IRQHandlers(COM3);
}


///////////////////////////////COM2 UART4 DMA tx rx的中断///////////////////////////////
/* --------------------------------------------------------------------------*/
/**
* @name DMA1_Stream4_IRQHandler
* @brief 
* @details COM2(UART4) DMA Tx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA1_Stream4_IRQHandler(void)                    //!< uart4 tx
{
    DMAx_Streamy_Tx_IRQHandlers(COM2);
}

/* --------------------------------------------------------------------------*/
/**
* @name DMA1_Stream2_IRQHandler
* @brief 
* @details COM2(UART4) DMA Rx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA1_Stream2_IRQHandler(void)                    //!< uart4 rx
{
    DMAx_Streamy_Rx_IRQHandlers(COM2);
}

/* --------------------------------------------------------------------------*/
/**
* @name DMA2_Stream6_IRQHandler
* @brief 
* @details COM4(UART6) DMA Tx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA2_Stream6_IRQHandler(void)                     //!< com4 uart6 tx
{
	DMAx_Streamy_Tx_IRQHandlers(COM4);
}


/* --------------------------------------------------------------------------*/
/**
* @name DMA2_Stream1_IRQHandler
* @brief 
* @details COM4(UART6) DMA Rx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA2_Stream1_IRQHandler(void)                     //!< com4 uart6 rx
{
    DMAx_Streamy_Rx_IRQHandlers(COM4);
}

///////////////////////////COM5 UART5 DMA tx rx的中断///////////////////////////////////
/* --------------------------------------------------------------------------*/
/**
* @name DMA1_Stream7_IRQHandler
* @brief 
* @details COM5(UART5) DMA Tx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA1_Stream7_IRQHandler(void)                    //!< com5 uart5 tx
{
	DMAx_Streamy_Tx_IRQHandlers(COM5);	
}



/* --------------------------------------------------------------------------*/
/**
* @name DMA1_Stream0_IRQHandler
* @brief 
* @details COM5(UART5) DMA Rx IRQHandler
*
* @param[in] None
*
* @returns None
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void DMA1_Stream0_IRQHandler(void)		              //!< uart5 rx
{
	DMAx_Streamy_Rx_IRQHandlers(COM5);				
}

/* --------------------------------------------------------------------------*/
/**
* @name UartDmaTransferConfig
* @brief 
* @details copy data from DMA rx buff 0 or 1 to Uartx rx buffer Uartx_Rx[]
*
* @param[in] port COMx
* @param[in] length The length of recvd data.
* @param[in] pRevNo DMA recieve bufferX pointer
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 DMARecvDataDeal(COM_PORT_E port,u16 length,u8 pRevNo)
{
    u8 *pbuf;
    u16 len=length;
	u16 rest0,tmp0;

	if(pRevNo==1)
	{
        pbuf = ComTrans[port].pRecv1;
	}
	else
	{
        pbuf = ComTrans[port].pRecv0;
	}

	if((len+ComRecv[port].RxCount)>COM_RX_BUFFER_SIZE)    //!< 接收数据个数超出buffer
	{
        return ERR_DATA_OVERFLOW;
	}

	if(ComRecv[port].pRxWrite>=ComRecv[port].pRxRead)           //!< 正常类型
	{   
	    if((ComRecv[port].pRxWrite==ComRecv[port].pRxRead)&&(ComRecv[port].RxCount!=0))
	    {                                                      //!< buffer全满,丢弃数据并返回错误
            return ERR_DATA_OVERFLOW;
		}
		
	    rest0 = (u16)(ComRecv[port].pBuffer+COM_RX_BUFFER_SIZE-ComRecv[port].pRxWrite);
		
        if(len<rest0)
        {
            memcpy(ComRecv[port].pRxWrite,pbuf,len);
			ComRecv[port].pRxWrite += len;
			ComRecv[port].RxCount += len;
		}
		else
		{
            tmp0 = len-rest0;
			memcpy(ComRecv[port].pRxWrite,pbuf,rest0);
			memcpy(ComRecv[port].pBuffer,(pbuf+rest0),tmp0);

			ComRecv[port].pRxWrite = ComRecv[port].pBuffer+tmp0;
			ComRecv[port].RxCount += len;
		}
	}
	else                                                      //!< write指针在上，read指针在下
	{
        memcpy(ComRecv[port].pRxWrite,pbuf,len);
		ComRecv[port].pRxWrite += len;
		ComRecv[port].RxCount += len;
	}
	
	return ERR_NONE;
}

/* --------------------------------------------------------------------------*/
/**
* @name UARTx_IRQHanders
* @brief 
* @details process the IDLE interrupt of Uart x 
*
* @param[in] port COMx
*
* @returns none 
*
* @author 
*/
/* --------------------------------------------------------------------------*/
void UARTx_IRQHanders(COM_PORT_E com)
{
	u16 len;
/////////////////////////////////////////////////////////////////////////////
	if(USART_GetITStatus(ComTrans[com].uartx, USART_IT_RXNE) != RESET)     //!< 使用DMA接收所以并不使能usart的接收中断
	{
        USART_ClearFlag(ComTrans[com].uartx,USART_FLAG_RXNE);              //!< 用不到
	}
////////////////////////////////////////////////////////////////////////////
    if(USART_GetITStatus(ComTrans[com].uartx, USART_IT_IDLE) != RESET)  
    {
        (ComTrans[com].uartx)->SR;  
        (ComTrans[com].uartx)->DR;                                  //!< 清USART_IT_IDLE标志  
        
        DMA_Cmd(ComTrans[com].prStream,DISABLE);               //!< 关闭DMA         
        
        DMA_ClearFlag(ComTrans[com].prStream,ComTrans[com].dma_rtcflag);  //!< 清除标志位    
        
        len = UART_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(ComTrans[com].prStream);  //!< 获得接收帧帧长  

		if(DMA_GetCurrentMemoryTarget(ComTrans[com].prStream)==0)
		{
            //res = 
			DMARecvDataDeal(com,len,0);     
		}
		else
		{
            //res = 
			DMARecvDataDeal(com,len,1);
		}
		
        DMA_SetCurrDataCounter(ComTrans[com].prStream,UART_RX_BUFFER_SIZE);  //!< 设置传输数据长度    
        
        DMA_Cmd(ComTrans[com].prStream,ENABLE);  //!< 打开DMA    
	}
//////////////////////////////////////////////////////////////////////////////
}

/* --------------------------------------------------------------------------*/
/**
* @name Uart IRQ Handler
* @brief 
* @details Usart1
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void USART1_IRQHandler(void)
{
    UARTx_IRQHanders(COM1);
}

/* --------------------------------------------------------------------------*/
/**
* @name Uart IRQ Handler
* @brief 
* @details Usart2
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void USART2_IRQHandler(void)
{
    UARTx_IRQHanders(COM6);
}

/* --------------------------------------------------------------------------*/
/**
* @name Uart IRQ Handler
* @brief 
* @details Usart3
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void USART3_IRQHandler(void)
{
	UARTx_IRQHanders(COM3);
}

/* --------------------------------------------------------------------------*/
/**
* @name Uart IRQ Handler
* @brief 
* @details Usart4
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void UART4_IRQHandler(void)
{
    UARTx_IRQHanders(COM2);
}

/* --------------------------------------------------------------------------*/
/**
* @name Uart IRQ Handler
* @brief 
* @details Usart6
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void USART6_IRQHandler(void)
{
	UARTx_IRQHanders(COM4);
}

/* --------------------------------------------------------------------------*/
/**
* @name Uart IRQ Handler
* @brief 
* @details Uart5
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void UART5_IRQHandler(void)
{
    UARTx_IRQHanders(COM5);
}


/* --------------------------------------------------------------------------*/
/**
* @name UartxAndDmax_Init
* @brief 
* @details Initialize Uart X Pins&Configuration&DMA transfer 
*
* @param[in] port COMx
*
* @returns None 
*
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void UartxAndDmax_Init(COM_PORT_E Comport)
{
    NVIC_InitTypeDef NVIC_InitStructure ;     
    GPIO_InitTypeDef GPIO_InitStructure;  
    USART_InitTypeDef USART_InitStructure; 
    DMA_InitTypeDef DMA_InitStructure;
	
//!< 1.管脚配置
    switch(Comport)
    {
        case COM1://!< COM1====UART1==== PA9 PA10
			      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	              RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	              GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	              GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
		
	              GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	              GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	              GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		
	              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	              GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	              GPIO_Init(GPIOA, &GPIO_InitStructure);
			
	              GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	              GPIO_Init(GPIOA, &GPIO_InitStructure);
			break;
		case COM2://!< COM2====UART4==== PC10 PC11
			      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	              RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	              GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
	              GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	              GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	              GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	              GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	              GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	              GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	              GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	              GPIO_Init(GPIOC, &GPIO_InitStructure);
			break;
		case COM3://!< COM3====UART3====PB10 PB11
	              RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	              RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	              GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	              GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
		
	              GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	              GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	              GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		
	              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	              GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	              GPIO_Init(GPIOB, &GPIO_InitStructure);
			
	              GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	              GPIO_Init(GPIOB, &GPIO_InitStructure);	
			break;
		case COM4://!< COM4====UART6====PC6 PC7 
	              RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	              RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	              GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	              GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	              GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	              GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	              GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	              GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	              GPIO_Init(GPIOC, &GPIO_InitStructure);

	              GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	              GPIO_Init(GPIOC, &GPIO_InitStructure);
			break;
		case COM5://!< COM5====UART5====PC12 PD2
	              RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC |RCC_AHB1Periph_GPIOD, ENABLE);
	              RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	              GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
	              GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	              GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	              GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	              GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	              GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	              GPIO_Init(GPIOC, &GPIO_InitStructure);

	              GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	              GPIO_Init(GPIOD, &GPIO_InitStructure);
			break;
		case COM6://!< COM6====UART2====PD5 PD6,TX RX	
                  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	              RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	              GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	              GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	              GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	              GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	              GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	              GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	              GPIO_Init(GPIOD, &GPIO_InitStructure);
			break;
		
		default:
			break;
	}

//!< 2.串口配置:参数及中断优先级
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_BaudRate = ComTrans[Comport].chbps;
	USART_Init(ComTrans[Comport].uartx, &USART_InitStructure);	

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMAx_UARTx_RT_PRE_PRIO_VALUE;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  	
    NVIC_InitStructure.NVIC_IRQChannel = ComTrans[Comport].uartIrq;              //!< 串口中断优先级设置，主要是在RX时会使用到  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = ComTrans[Comport].subPrio;	
	NVIC_Init(&NVIC_InitStructure);  

	USART_ITConfig(ComTrans[Comport].uartx,USART_IT_TC,DISABLE);//!< no dma transfer,enable TC,RXNE interrupt
	USART_ITConfig(ComTrans[Comport].uartx,USART_IT_RXNE,DISABLE);
	USART_ITConfig(ComTrans[Comport].uartx,USART_IT_TXE,DISABLE);
	USART_ITConfig(ComTrans[Comport].uartx,USART_IT_IDLE,ENABLE);

//!< 3.打开DMA时钟

//!< 4.配置发送DMA、中断及优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_InitStructure.NVIC_IRQChannel = ComTrans[Comport].dmatchlirq;     
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = ComTrans[Comport].dmatsubPrio;
    NVIC_Init(&NVIC_InitStructure); 

    DMA_DeInit(ComTrans[Comport].ptStream);	                                    //!< DMA通道配置	
	DMA_InitStructure.DMA_Channel = ComTrans[Comport].dmatChl;		            //!< DMA发送通道
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(ComTrans[Comport].uartx)->DR);  //!< 外设地址 	
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ComTrans[Comport].sCtrl.pSnd[0];  //!< 内存地址		
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; 			 //!< dma传输方向	
	DMA_InitStructure.DMA_BufferSize = 0x64;							 //!< 设置DMA在传输时缓冲区的长度	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	 //!< 设置DMA的外设递增模式，一个外设	
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 			 //!< 设置DMA的内存递增模式  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //!< 外设数据字长 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  //!< 内存数据字长 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;						 //!< 设置DMA的传输模式	
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 				 //!< 设置DMA的优先级别  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;				 //!< 指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式	
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;	 //!< 指定了FIFO阈值水平	  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 		 //!< 指定的Burst转移配置内存传输	
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  //!< 指定的Burst转移配置外围转
    DMA_ITConfig(ComTrans[Comport].ptStream,DMA_IT_TC,ENABLE);			 //!< 使能DMA发送完成中断 
    DMA_ITConfig(ComTrans[Comport].ptStream,DMA_IT_TE,ENABLE);			 //!< 使能DMA发送错误中断
	DMA_Init(ComTrans[Comport].ptStream, &DMA_InitStructure); 			 //!< 配置DMAx的通道	
	
//!< 5.配置接收DMA、中断及优先级
    DMA_DeInit(ComTrans[Comport].prStream);  
    DMA_InitStructure.DMA_Channel = ComTrans[Comport].dmarChl;      
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ComTrans[Comport].uartx)->DR);     
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(ComTrans[Comport].pRecv0);      
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;    
    DMA_InitStructure.DMA_BufferSize = UART_RX_BUFFER_SIZE;      
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;              //!< 软件优先级高于发送DMA的
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;   
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;    
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;    
	
    DMA_DoubleBufferModeConfig(ComTrans[Comport].prStream,(uint32_t)(ComTrans[Comport].pRecv1),DMA_Memory_0);   //!< config Memory1BaseAddr & CT bit
	DMA_DoubleBufferModeCmd(ComTrans[Comport].prStream,ENABLE);	        //!< 使能double buffer mode 
	DMA_Init(ComTrans[Comport].prStream, &DMA_InitStructure);           //!< 配置DMA的通道      
    DMA_Cmd(ComTrans[Comport].prStream,ENABLE);                         //!< 使能DMA接收通道  

//!< 6.打开UARTx&DMA的开关
	USART_DMACmd(ComTrans[Comport].uartx,USART_DMAReq_Tx,ENABLE);	//!< 使能串口DMA发送
	USART_DMACmd(ComTrans[Comport].uartx,USART_DMAReq_Rx,ENABLE);   //!< 使能串口DMA接收
	// DMA_Cmd(ComTrans[Comport].ptStream,ENABLE);                   //当需要用到发送时，再使能DMA发送 Stream
	USART_ClearFlag(ComTrans[Comport].uartx, USART_FLAG_TC);
	USART_Cmd(ComTrans[Comport].uartx, ENABLE);	                    //!< 使能串口
}


/* --------------------------------------------------------------------------*/
/**
* @name BspDmaUart_init
* @brief 
* @details Initialize the Uarts and DMAs transfer
*
* @param[in] None.
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void BspDmaUart_init(void)
{
    COM_PORT_E i;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);      //!< 启动DMA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	for(i=COM1;i<=COM6;i++)
	{	          
		UartxAndDmax_Init(i);	
	}	
}

/* --------------------------------------------------------------------------*/
/**
* @name PrintfUart1DmaTransferConfig
* @brief 
* @details 
*
* @param[in] pSndBuf The package Number.
* @param[in] len 　　　　　The Length of the Buffer.
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
/*  没有问题的
u8 PrintfUart1DmaTransferConfig(u8 *pSndBuf,u16 len)
{
	u8 res;
	u8 *pSnd;
	//DMA_Stream_TypeDef pDMATransStream;

	if(len>UART_TX_BUFFER_SIZE) return ERR_INPUT_PARAMETERS;

	if(PrintTransCtrl.bsy[0]==0)                            //!< buff0 is not busy
	{
		if((PrintTransCtrl.SndCnt[0]+len)<=UART_TX_BUFFER_SIZE)
		{
			pSnd = PrintTransCtrl.pSnd[0];
			
			memcpy((pSnd+PrintTransCtrl.SndCnt[0]),pSndBuf,len);
			
			PrintTransCtrl.SndCnt[0] += len;

			if(PrintTransCtrl.bsy[1]==0)                    //!< if buff1 is not busy,start DMA Tx
			{
				PrintTransCtrl.bsy[0] = 1;

				PrintTransCtrl.CurBuf = 0;	                //!< buff0 is transmitting

				(DMA2_Stream7)->M0AR = (u32)PrintTransCtrl.pSnd[0];

				DMA_SetCurrDataCounter(DMA2_Stream7,PrintTransCtrl.SndCnt[0]);
		
				DMA_Cmd(DMA2_Stream7,ENABLE);				
			}

			res = ERR_NONE;
		}
		else                                               //!< data length exceeds buff0 empty space 
		{				
			res = ERR_UART_TX_BUF_FULL;
		}
	}
	else                                                   //!< buff0 is busy
	{
		if((PrintTransCtrl.SndCnt[1]+len)<=UART_TX_BUFFER_SIZE)
		{
			pSnd = PrintTransCtrl.pSnd[1];
			
			memcpy((pSnd+PrintTransCtrl.SndCnt[1]),pSndBuf,len);
			
			PrintTransCtrl.SndCnt[1] += len;

			if(PrintTransCtrl.bsy[0]==0)                    //!< if buff1 is not busy,start DMA Tx
			{
				PrintTransCtrl.bsy[1] = 1;

				PrintTransCtrl.CurBuf = 1;	                //!< buff1 is transmitting

				(DMA2_Stream7)->M0AR = (u32)PrintTransCtrl.pSnd[1];

				DMA_SetCurrDataCounter(DMA2_Stream7,PrintTransCtrl.SndCnt[1]);
		
				DMA_Cmd(DMA2_Stream7,ENABLE);				
			}

			res = ERR_NONE;
		}
		else                                               //!< data length exceeds buff1 empty space 
		{
			res = ERR_UART_TX_BUF_FULL;
		}
	}

	return res;
	
}
*/

s8 PrintfUart1DmaTransferConfig(u8 *pSndBuf,u16 len)
{
	s8 res;
	u8 *pSnd;
	//DMA_Stream_TypeDef pDMATransStream;

	if(len>UART_TX_BUFFER_SIZE) return ERR_INPUT_PARAMETERS;

	if(PrintTransCtrl.bsy[0]==0)                            //!< buff0 is not busy
	{
		if((PrintTransCtrl.SndCnt[0]+len)<=UART_TX_BUFFER_SIZE)
		{
			pSnd = PrintTransCtrl.pSnd[0];
			
			memcpy((pSnd+PrintTransCtrl.SndCnt[0]),pSndBuf,len);
			
			PrintTransCtrl.SndCnt[0] += len;

			if(PrintTransCtrl.bsy[1]==0)                    //!< if buff1 is not busy,start DMA Tx
			{
				PrintTransCtrl.bsy[0] = 1;

				PrintTransCtrl.CurBuf = 0;	                //!< buff0 is transmitting

				(DMA2_Stream7)->M0AR = (u32)PrintTransCtrl.pSnd[0];

				DMA_SetCurrDataCounter(DMA2_Stream7,PrintTransCtrl.SndCnt[0]);
		
				DMA_Cmd(DMA2_Stream7,ENABLE);				
			}

			res = ERR_NONE;
		}
		else                                               //!< data length exceeds buff0 empty space 
		{				
			res = ERR_UART_TX_BUF_FULL;
		}
	}
	else                                                   //!< buff0 is busy
	{
		if((PrintTransCtrl.SndCnt[1]+len)<=UART_TX_BUFFER_SIZE)
		{
			pSnd = PrintTransCtrl.pSnd[1];
			
			memcpy((pSnd+PrintTransCtrl.SndCnt[1]),pSndBuf,len);
			
			PrintTransCtrl.SndCnt[1] += len;

			if(PrintTransCtrl.bsy[0]==0)                    //!< if buff1 is not busy,start DMA Tx
			{
				PrintTransCtrl.bsy[1] = 1;

				PrintTransCtrl.CurBuf = 1;	                //!< buff1 is transmitting

				(DMA2_Stream7)->M0AR = (u32)PrintTransCtrl.pSnd[1];

				DMA_SetCurrDataCounter(DMA2_Stream7,PrintTransCtrl.SndCnt[1]);
		
				DMA_Cmd(DMA2_Stream7,ENABLE);				
			}

			res = ERR_NONE;
		}
		else                                               //!< data length exceeds buff1 empty space 
		{
			res = ERR_UART_TX_BUF_FULL;
		}
	}

	return res;
	
}


/*   //有问题的
u8 PrintfUart1DmaTransferConfig(u8 *pSndBuf,u16 len)
{
	u8 i;
	u8 *pSnd;

	if(len>UART_TX_BUFFER_SIZE) return ERR_INPUT_PARAMETERS;
	
	for(i=0;i<PRINTF_TX_BUFF_CNT;i++)      //!< check if data can be stored to buffer that is not full
	{
    	if(ComTrans[COM1].sCtrl.bsy[i]==0)
		{
			if((ComTrans[COM1].sCtrl.SndCnt[i]+len)<=UART_TX_BUFFER_SIZE)
			{
				pSnd = ComTrans[COM1].sCtrl.pSnd[i];
				memcpy((pSnd+ComTrans[COM1].sCtrl.SndCnt[i]),pSndBuf,len);
				ComTrans[COM1].sCtrl.SndCnt[i] += len;

				break;
			}
			else
			{
				continue;
			}
		}
	}

	if(i>=PRINTF_TX_BUFF_CNT)
	{
		return ERR_UART_TX_BUF_FULL;
	}

	for(i=0;i<PRINTF_TX_BUFF_CNT;i++)//!< check if busy trans,if no ,start new transfer
	{
		if(ComTrans[COM1].sCtrl.bsy[i]!=0)
		{
			break;
		}
	}

	if(i>=PRINTF_TX_BUFF_CNT)        //!< start transfer
	{		
		for(i=0;i<PRINTF_TX_BUFF_CNT;i++)
		{
			if(ComTrans[COM1].sCtrl.bsy[i]==0)
			{
				break;
			}
		}
		
		ComTrans[COM1].sCtrl.CurBuf = i;	
	
		ComTrans[COM1].sCtrl.bsy[i] = 1;

		(ComTrans[COM1].ptStream)->M0AR = (u32)ComTrans[COM1].sCtrl.pSnd[i];

		DMA_SetCurrDataCounter(ComTrans[COM1].ptStream,ComTrans[COM1].sCtrl.SndCnt[i]);
		
		DMA_Cmd(ComTrans[COM1].ptStream,ENABLE); 
	}
	
	return ERR_NONE;

}

*/

/* --------------------------------------------------------------------------*/
/**
* @name UartDmaTransferConfig
* @brief 
* @details 
*
* @param[in] port The address of the Image file bytes.
* @param[in] pSndBuf The package Number.
* @param[in] len 　　　　　The Length of the Buffer.
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
/*
u8 UartDmaTransferConfig(COM_PORT_E port,u8 *pSndBuf,u16 len)
{
	u8 uni;
	u8 *pSnd;

	if(len>UART_TX_BUFFER_SIZE) return ERR_INPUT_PARAMETERS;
    
    for(uni=0;uni<MAX_DMA_COM_NUMS;uni++)     //!< cpy to dma tx buffer
    {
        if(port==ComTrans[uni].port)
		{
            pSnd = ComTrans[uni].pSend;
			
			break;
		}
	}
	
	if(ComTrans[uni].tbsy==0)                //!< not busy
	{
        memcpy(pSnd,pSndBuf,len);

		ComTrans[uni].tbsy = 1;

		DMA_SetCurrDataCounter(ComTrans[uni].ptStream,len);
		
	    DMA_Cmd(ComTrans[uni].ptStream,ENABLE);        
	}
	else
	{
        return ERR_IS_BUSY;
	}
	
	return ERR_NONE;

}
*/

s8 UartDmaTransferConfig(COM_PORT_E port,u8 *pSndBuf,u16 len)
{
	u8 i;	
	u8 *pSnd;
	BUFFERSENDCTRL *psCtrl;
	DMA_Stream_TypeDef* pStream;
	
	if(len>UART_TX_BUFFER_SIZE) return ERR_INPUT_PARAMETERS;

	for(i=0;i<MAX_DMA_COM_NUMS;i++)     //!< cpy to dma tx buffer
    {
        if(port==ComTrans[i].port)
		{
            psCtrl = &ComTrans[i].sCtrl;

			pStream = ComTrans[i].ptStream;
			
			break;
		}
	}
	
		
	for(i=0;i<UART_TX_BUFF_CNT;i++)	   //!< check if data can be stored to buffer that is not full
	{
		if(psCtrl->bsy[i]==0)
		{
			if((psCtrl->SndCnt[i]+len)<=UART_TX_BUFFER_SIZE)
			{
					pSnd = psCtrl->pSnd[i];
					
					memcpy((pSnd+psCtrl->SndCnt[i]),pSndBuf,len);
					
					psCtrl->SndCnt[i] += len;
	
					break;
				}
				else
				{
					continue;
				}
			}
		}
	
		if(i>=UART_TX_BUFF_CNT)
		{
			return ERR_UART_TX_BUF_FULL;
		}
	
		for(i=0;i<UART_TX_BUFF_CNT;i++)    //!< check if busy trans,if no ,start new transfer
		{
			if(psCtrl->bsy[i]!=0)
			{
				break;
			}
		}
	
		if(i>=UART_TX_BUFF_CNT)	   	     //!< start transfer
		{		
			for(i=0;i<UART_TX_BUFF_CNT;i++)
			{
				if(psCtrl->bsy[i]==0)
				{
					break;
				}
			}
			
			psCtrl->CurBuf = i;	
		
			psCtrl->bsy[i] = 1;
	
			pStream->M0AR = (u32)psCtrl->pSnd[i];
	
			DMA_SetCurrDataCounter(pStream,psCtrl->SndCnt[i]);
			
			DMA_Cmd(pStream,ENABLE); 
		}
		
		return ERR_NONE;
}



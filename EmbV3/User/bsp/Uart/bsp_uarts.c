/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp_uarts.c
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


UART_T g_tUart7;
u8 g_TxBuf7[UART7_TX_BUF_SIZE];		
u8 g_RxBuf7[UART7_RX_BUF_SIZE];

#if UART8_FIFO_EN == 1
UART_T g_tUart8;
u8 g_TxBuf8[UART8_TX_BUF_SIZE];
u8 g_RxBuf8[UART8_RX_BUF_SIZE];
#endif
static void InitUartsVar(void);
static void InitHardUarts(void);
static void InitUartsNVIC(void);
static void UartIRQ(UART_T *_pUart);
static void UartSend(UART_T *_pUart, u8 *_ucaBuf, u16 _usLen);


/* --------------------------------------------------------------------------*/
/**
* @name BspUarts_init
* @brief 
* @details Uart 7&8 transfer configuration:Pins&interrupt&Buffer RW Control
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void BspUarts_init(void)
{
	InitUartsVar();		//!<  必须先初始化全局变量,再配置硬件
	InitUartsNVIC();	//!<  配置串口中断 
	InitHardUarts();	//!<  配置串口的硬件参数(波特率等) 	
}

/* --------------------------------------------------------------------------*/
/**
* @name InitUartsVar
* @brief 
* @details Uart 7&8 Tx&Rx buffer config
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
static void InitUartsVar(void)
{	
	g_tUart7.uart = UART7;
	g_tUart7.pTxBuf = g_TxBuf7;
	g_tUart7.pRxBuf = g_RxBuf7;
	g_tUart7.usTxBufSize = UART7_TX_BUF_SIZE;
	g_tUart7.usRxBufSize = UART7_RX_BUF_SIZE;
	g_tUart7.usTxWrite = 0;
	g_tUart7.usTxRead = 0;
	g_tUart7.usRxWrite = 0;
	g_tUart7.usRxRead = 0;
	g_tUart7.usRxCount = 0;
	g_tUart7.usTxCount = 0;
	
#if UART8_FIFO_EN == 1
	g_tUart8.uart = UART8;
	g_tUart8.pTxBuf = g_TxBuf8;
	g_tUart8.pRxBuf = g_RxBuf8;
	g_tUart8.usTxBufSize = UART8_TX_BUF_SIZE;
	g_tUart8.usRxBufSize = UART8_RX_BUF_SIZE;
	g_tUart8.usTxWrite = 0;
	g_tUart8.usTxRead = 0;
	g_tUart8.usRxWrite = 0;
	g_tUart8.usRxRead = 0;
	g_tUart8.usRxCount = 0;
	g_tUart8.usTxCount = 0;
#endif

}

/* --------------------------------------------------------------------------*/
/**
* @name InitHardUarts
* @brief 
* @details Uart 7&8 Hardware interface config
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void InitHardUarts(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_UART7);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_UART7);
	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = UART7_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART7, &USART_InitStructure);

	USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART7, ENABLE);
	USART_ClearFlag(UART7, USART_FLAG_TC);

#if UART8_FIFO_EN == 1	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_UART8);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_UART8);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = UART8_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART8, &USART_InitStructure);

	USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART8, ENABLE);
	USART_ClearFlag(UART8, USART_FLAG_TC);
#endif
}

/* --------------------------------------------------------------------------*/
/**
* @name InitUartsNVIC
* @brief 
* @details Uart 7&8 interrupt priority config
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
static void InitUartsNVIC(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART7_PRE_PRIO_VALUE;//!<  Configure the NVIC Preemption Priority Bits
	NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = UART7_SUB_PRIO_VALUE;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

#if UART8_FIFO_EN == 1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART8_PRE_PRIO_VALUE;//!<  Configure the NVIC Preemption Priority Bits
	NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = UART8_SUB_PRIO_VALUE;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
#endif
}

/* --------------------------------------------------------------------------*/
/**
* @name ComToUart
* @brief 
* @details Exchange between Uart X and COM X
*
* @param[in] port COMx
*
* @returns address of g_tUart7 or g_tUart8
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
UART_T *ComToUart(COM_PORT_E _ucPort)
{
	if (_ucPort == COM7)
	{
		return &g_tUart7;
	}
	else if (_ucPort == COM8)
	{
		#if UART8_FIFO_EN == 1
			return &g_tUart8;
		#else
			return NULL;
		#endif
	}
	else
	{		
		return NULL;
	}
}

/* --------------------------------------------------------------------------*/
/**
* @name comSendBuf
* @brief 
* @details process Uart 7&8 data transmit operation
*
* @param[in] port COMx
* @param[in] _ucaBuf Pointer of Transmitting data
* @param[in] _usLen Length of Transmitting data
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void comSendBuf(COM_PORT_E _ucPort, u8 *_ucaBuf, u16 _usLen)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);

	if (pUart == 0)
	{
		return;
	}

	UartSend(pUart, _ucaBuf, _usLen);
}


/* --------------------------------------------------------------------------*/
/**
* @name UartSend
* @brief 
* @details process Uart 7&8 data transmit operation
*
* @param[in] _pUart Uart 7 or 8
* @param[in] _ucaBuf Pointer of Transmitting data
* @param[in] _usLen Length of Transmitting data
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
static void UartSend(UART_T *_pUart, u8 *_ucaBuf, u16 _usLen)
{
	u16 i;
	u16 usCount;

	for (i = 0; i < _usLen; i++)
	{		
		while (1)                                //!< 当 _pUart->usTxBufSize == 1 时, 下面的函数会死掉(待完善) 
		{
			DISABLE_INT();
			
			usCount = _pUart->usTxCount;
			
			ENABLE_INT();

			if (usCount < _pUart->usTxBufSize)
			{
				break;
			}
		}
		
		_pUart->pTxBuf[_pUart->usTxWrite] = _ucaBuf[i];    //!< 将新数据填入发送缓冲区 

		DISABLE_INT();
		
		if (++_pUart->usTxWrite >= _pUart->usTxBufSize)
		{
			_pUart->usTxWrite = 0;
		}
		
		_pUart->usTxCount++;
		
		ENABLE_INT();
	}

	USART_ITConfig(_pUart->uart, USART_IT_TXE, ENABLE);
}

/* --------------------------------------------------------------------------*/
/**
* @name UartIRQ
* @brief 
* @details Uart 7 or 8 transfer interrupt handler,called by UartIRQ Handler
*
* @param[in] _pUart g_tUart7 or g_tUart8
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
static void UartIRQ(UART_T *_pUart)
{
	u8 ch;
	
	if (USART_GetFlagStatus(_pUart->uart, USART_FLAG_ORE) != RESET)
	{
		USART_ReceiveData(_pUart->uart);
	}
	
	if (USART_GetITStatus(_pUart->uart, USART_IT_RXNE) != RESET)   //!< 处理接收中断  
	{
		ch = USART_ReceiveData(_pUart->uart);                      //!< 从串口接收数据寄存器读取数据存放到接收FIFO 
		
		if (_pUart->usRxCount < _pUart->usRxBufSize)
		{
			_pUart->usRxCount++;
			
			_pUart->pRxBuf[_pUart->usRxWrite] = ch;
				
			if (++_pUart->usRxWrite >= _pUart->usRxBufSize)
			{
				_pUart->usRxWrite = 0;
			}				
		}		
	}
	
	if (USART_GetITStatus(_pUart->uart, USART_IT_TXE) != RESET)  //!< 处理发送缓冲区空中断 
	{
		if (_pUart->usTxCount == 0)
		{
			//!< 发送缓冲区的数据已取完时， 禁止发送缓冲区空中断 （注意：此时最后1个数据还未真正发送完毕）
			USART_ITConfig(_pUart->uart, USART_IT_TXE, DISABLE);

			//!< 使能数据发送完毕中断 
			USART_ITConfig(_pUart->uart, USART_IT_TC, ENABLE);
		}
		else
		{
			//!< 从发送FIFO取1个字节写入串口发送数据寄存器 
			USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}	
	}		
	else if (USART_GetITStatus(_pUart->uart, USART_IT_TC) != RESET)//!< 数据bit位全部发送完毕的中断 
	{
		if (_pUart->usTxCount == 0)
		{
			
			USART_ITConfig(_pUart->uart, USART_IT_TC, DISABLE);    //!< 如果发送FIFO的数据全部发送完毕，禁止数据发送完毕中断 
		}
		else
		{
			//!< 正常情况下，不会进入此分支 			
			USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);//!< 如果发送FIFO的数据还未完毕，则从发送FIFO取1个数据写入发送数据寄存器 

			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}
	}
}


/* --------------------------------------------------------------------------*/
/**
* @name Uart IRQ Handler
* @brief 
* @details Uart 7 or 8 transfer interrupt handler
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void UART7_IRQHandler(void)
{
	UartIRQ(&g_tUart7);
}


#if UART8_FIFO_EN == 1
void UART8_IRQHandler(void)
{
	UartIRQ(&g_tUart8);
}
#endif


int fputc(int ch, FILE *f)
{
    s8 res = 0;
	u8 dat = (u8)ch;
	
	res = UartDmaTransferConfig(COM1,&dat,1);

	return res;
}


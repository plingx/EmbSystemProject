/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file bsp.h
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
* 2017/12/20 | 0.0.1 | LingPing | Create file
*
*/
#ifndef _BSP_H_
#define _BSP_H_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "bsp_adc.h"
#include "bsp_i2c.h"
#include "bsp_tim.h"
#include "IMU.h"
#include "stm32f4x7_eth.h"
#include "bsp_eth.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_uarts.h"
#include "bsp_userlib.h"
#include "bsp_uart_dma.h"
#include "sys_utils.h"


#define EWAYBOTEMB_02                       //!< Define EwayBot EMB Version
#define __STM32F4_BSP_VERSION		"STM32429ZGT6"     //!< Define the BSP_VERSION,reserved

#define ENABLE_INT()	__set_PRIMASK(0)	//!< Enable global interrupts
#define DISABLE_INT()	__set_PRIMASK(1)	//!< Disable global interrupts

//#define Bsp_Printf		        printf      //!< Define debug printf
#define PRINT_SD_DEBUG_INFO        0        //!< Define SD Card debug print info
//#define PRINT_FS_DEBUG_INFO        1        //!< Define file R/W debug print info
#define PRINT_ETH_DEBUG_INFO       0
#define PRINT_BSP_DEBUG_INFO       0
#define PRINT_ASSERT_DEBUG         1
#define IWDG_RELOAD_VALUE          0x7D0         //32K LSI 16��Ƶ��Ӧÿ��counterΪ0.5ms,0x7D0��ӦΪ1s


/**
* Define the ethernet interrupt preemption and sub priority.
* Define the Timer9 interrupt preemption and sub priority.
* Define DMA uart tx rx preemption and sub interrupt priority.
* @param None.
* @return None.
*/
/*
#define TIM9_PRE_PRIO_VALUE				0
#define TIM9_SUB_PRIO_VALUE         	1

#define ETHERNET_PRE_PRIO_VALUE     	1
#define ETHERNET_SUB_PRIO_VALUE     	0

#define DMAx_UARTx_RT_PRE_PRIO_VALUE	2
*/

#define TIM9_PRE_PRIO_VALUE				2
#define TIM9_SUB_PRIO_VALUE         	0
    
#define ETHERNET_PRE_PRIO_VALUE     	3
#define ETHERNET_SUB_PRIO_VALUE     	0
    
#define DMAx_UARTx_RT_PRE_PRIO_VALUE	1


/**
* Define the uarts interrupt sub priority.
*/
/*
#define UART1_SUB_PRIO_VALUE    1			//!<	COM1
#define UART2_SUB_PRIO_VALUE    1			//!<	COM6
#define UART3_SUB_PRIO_VALUE    1			//!<	COM3
#define UART4_SUB_PRIO_VALUE    1			//!<	COM2
#define UART5_SUB_PRIO_VALUE    1			//!<	COM5
#define UART6_SUB_PRIO_VALUE    1			//!<	COM4
*/

#define UART1_SUB_PRIO_VALUE    0			//!<	COM1
#define UART2_SUB_PRIO_VALUE    0			//!<	COM6
#define UART3_SUB_PRIO_VALUE    0			//!<	COM3
#define UART4_SUB_PRIO_VALUE    0			//!<	COM2
#define UART5_SUB_PRIO_VALUE    0			//!<	COM5
#define UART6_SUB_PRIO_VALUE    0			//!<	COM4


/**
* Define the uarts dma interrupt priority.
* @param None.
* @return None.
*/
#define DMA2T_ST7_SUB_PRIO_VALUE    0       //!< COM1
#define DMA1T_ST4_SUB_PRIO_VALUE    0       //!< COM2
#define DMA1T_ST3_SUB_PRIO_VALUE    0       //!< COM3
#define DMA2T_ST6_SUB_PRIO_VALUE    0       //!< COM4
#define DMA1T_ST7_SUB_PRIO_VALUE    0       //!< COM5
#define DMA1T_ST6_SUB_PRIO_VALUE    0       //!< COM6

/**
* Define the uart7&8 interrupt pre-emption and sub priority.
*/
/*
#define UART7_PRE_PRIO_VALUE    3
#define UART8_PRE_PRIO_VALUE    3

#define UART7_SUB_PRIO_VALUE    0
#define UART8_SUB_PRIO_VALUE    1
*/
#define UART7_PRE_PRIO_VALUE    5
#define UART8_PRE_PRIO_VALUE    6

#define UART7_SUB_PRIO_VALUE    0
#define UART8_SUB_PRIO_VALUE    0


/**
* Define the timer3 interrupt priority.
* @param None.
* @return None.
*/

//#define TIM3_SUB_PRIO_VALUE         4


typedef enum{
	LogCiritcal=0,
	LogMajor=1,
	LogNormal=2,
	LogMinor=3,
	LogTrivial
}LogPrioity;


void bsp_Init(void);
void SysConfigNVICPriorityGroup(void);
//s8 Bsp_printf(char *fmt,int arg1,int arg2,int arg3,int arg4,int arg5,int arg6);
s8 Bsp_printf(char *fmt,...);
void BspWatchDog_Init(void);
void IWDG_Feed(void);
s8 SysLogWrite(LogPrioity eLogPri,const char * strLog,...);

#endif

/***************************** EWAYBOT(END OF FILE) *********************************/

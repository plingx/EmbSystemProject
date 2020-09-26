/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp_eth.h
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
* 2017/12/20 | 0.0.1 | LingPing | Create file
*
*/
#ifndef _BSP_ETH_H_
#define _BSP_ETH_H_

//#include "stm32f4x7_eth.h"

	

#define LAN8720_PHY_ADDRESS  	0x00				//LAN8720 PHY芯片地址.
//extern ETH_DMADESCTypeDef *DMARxDscrTab;			//以太网DMA接收描述符数据结构体指针
//extern ETH_DMADESCTypeDef *DMATxDscrTab;			//以太网DMA发送描述符数据结构体指针 
//extern u8 *Rx_Buff; 							//以太网底层驱动接收buffers指针 
//extern u8 *Tx_Buff; 							//以太网底层驱动发送buffers指针

//extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;	//DMA最后接收到的帧信息指针
 

u8 BspEth_Init(void);
u32 ETH_GetCurrentTxBuffer(void);
u8 ETH_Tx_Packet(u16 FrameLength);
//FrameTypeDef ETH_Rx_Packet(void);
u8 ETH_Mem_Malloc(void);
void ETH_Mem_Free(void);



#endif 


/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file bsp_eth.h
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
#ifndef _BSP_ETH_H_
#define _BSP_ETH_H_

//#include "stm32f4x7_eth.h"

	

#define LAN8720_PHY_ADDRESS  	0x00				//LAN8720 PHYоƬ��ַ.
//extern ETH_DMADESCTypeDef *DMARxDscrTab;			//��̫��DMA�������������ݽṹ��ָ��
//extern ETH_DMADESCTypeDef *DMATxDscrTab;			//��̫��DMA�������������ݽṹ��ָ�� 
//extern u8 *Rx_Buff; 							//��̫���ײ���������buffersָ�� 
//extern u8 *Tx_Buff; 							//��̫���ײ���������buffersָ��

//extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;	//DMA�����յ���֡��Ϣָ��
 

u8 BspEth_Init(void);
u32 ETH_GetCurrentTxBuffer(void);
u8 ETH_Tx_Packet(u16 FrameLength);
//FrameTypeDef ETH_Rx_Packet(void);
u8 ETH_Mem_Malloc(void);
void ETH_Mem_Free(void);



#endif 


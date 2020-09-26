#ifndef _E_M_B_P_S_M_H
#define _E_M_B_P_S_M_H

//s8 Psm_Init();
//s8 Psm_Ping();
//s8 Psm_Read();

#include "stm32f4xx.h"

///PSM帧头
#define  PSMHeader_H                                                0xDDDD
/****************************************************************************************************
	Instruction
 ****************************************************************************************************/
///PING,在线监测识别，是否做好接收控制指令准备
#define  PSMPing_H                                                  0x01
///ping，帧长
#define  PSMPingLength_H	                                          0x07
///读寄存器;读数据指令
#define  PSMRead_H                                                  0x02
///读，帧长
#define  PSMReadLength_H	                                          0x07
///写寄存器,写数据指令，收到后验证无误立即执行
#define  PSMWrite_H                                                 0x03
///重启,驱动器软件复位重启
#define  PSMReborn_H                                                0xE1
///恢复出厂设置
#define  PSMSetInit_H                                               0xE2

#define CRC_LEN  2

#define PSM_HEADER_LEN  5

///PING——PSM
s8 PSM_PING(u8 unSlaveAddr);
///读PSM寄存器
s8 SendQuery_PSM(u8 unSlaveAddr, u8 *unSendBuffer,u8 unLen);
///写PSM寄存器
s8 PSM_WRITE(u8 unSlaveAddr,u8 unRegAddr,u16 unRegNum,u8 *unSendBuffer);
///PSM初始化
s8 PSM_SET_INITE(u8 unSlaveAddr);
///读寄存器BUF
extern u8 unPSMRegAddr[2];


#endif

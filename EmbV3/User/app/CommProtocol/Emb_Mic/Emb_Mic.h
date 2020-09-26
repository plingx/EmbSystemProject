/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Emb_Mic.h
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2018-05-30
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/05/30 | 0.0.1 | LingPing | Create file
*
*/
#ifndef _EMB_MIC_H_
#define _EMB_MIC_H_

///MIC帧头
#define  MICHeader_H                                                0xFFFF
#define  MIC_Start_ID                                               0x01


#define	MIC_ONLINE			1 //!< 在线
#define	MIC_NOLINE			0 //!< 未在线



/****************************************************************************************************
	Instruction
 ****************************************************************************************************/
///PING,在线监测识别，是否做好接收控制指令准备
#define  MICPing_H                                                  0x01
///ping，帧长
#define  MICPingLength_H	                                        0x07
///读寄存器;读数据指令
#define  MICRead_H                                                  0x02
///读，帧长
#define  MICReadLength_H	                                        0x07
///写寄存器,写数据指令，收到后验证无误立即执行
#define  MICWrite_H                                                 0x03


//======================Mic Register Macro define=============================
#define	GET_ONLINE_STATE	0x0E	//检查mic在线状态(人为添加寄存器)
#define	GET_VERSION			0x0F	//查询版本号命令
#define	GET_WAKE_ANGLE		0x10	//查询唤醒角度
#define	SET_RESET			0x11	//复位
#define	SET_RECE_BS_ID		0x12	//设置拾音波束(0-5)
#define	SET_WAKE_MODE		0x13    //设置唤醒  0：关闭唤醒  1:开启唤醒
#define	SET_CALL_MODE		0x14	//设置通话模式
#define	GET_WAKEWORD_ID		0x15	//查询唤醒词ID
#define	GET_WAKE_SCORE		0x16	//查询唤醒得分




///PING——MIC
s8 Mic_Ping(u8 unSlaveAddr);
///读MIC寄存器
s8 Mic_Read(u8 unSlaveAddr, u8 *unSendBuffer,u16 unLength);
///写MIC寄存器
s8 Mic_Write(u8 unSlaveAddr,u8 unRegAddr,u16 unRegNum,u8 *unSendBuffer);


#endif 

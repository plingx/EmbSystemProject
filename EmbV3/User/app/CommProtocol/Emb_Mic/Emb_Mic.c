/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Emb_Mic.c
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

#include "includes.h"





//MIC添加帧头，私有
u8 MICAddHead(u8 unKeyWord,u8 unSlaveAddr,u8 *punBuffer,u16 unInfoLength);
///MIC读寄存器BUF
u8 unMICVerRegAddr[2]={0x0F,0x01};//mic版本寄存器地址
u8 unMICAngRegAddr[2]={0x10,0x01};//mic唤醒声源寄存器地址
/**
*********************************************************************************************************
* @name MIC_PING
* @brief 检查MIc板是否在线
* @details 
*
* @param[in] unSlaveAddr
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*///FF FF 01 07 01 30 92
s8 Mic_Ping(u8 unSlaveAddr)	
{
	u8 unSendBuf[MICPingLength_H];
	u16 unCRC;

	//MICAddHead(MICPing_H,unSlaveAddr,unSendBuf,MICPingLength_H);
	unSendBuf[0] = (u8)MICHeader_H;//侦头
	unSendBuf[1] = (u8)(MICHeader_H>>8);
	unSendBuf[2] = unSlaveAddr;
	unSendBuf[3] = MICPingLength_H;
	unSendBuf[4] = MICPing_H;
    
	unCRC=CRC16_Modbus(&unSendBuf[0],5);//校验
	unSendBuf[5] = (u8)unCRC;
	unSendBuf[6] = (u8)(unCRC>>8);
    
	comSendBuf(COM8,unSendBuf,MICPingLength_H);
    
	return ERR_NONE;
}
/**
*********************************************************************************************************
* @name MIC_Read
* @brief 读MIC寄存器
* @details 
*
* @param[in] unSlaveAddr 从站ID
* @param[in] *unSendBuffer 读寄存器BUF
* @param[in] unLength BUF长度
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
1: FF FF 01 09 02 0F 01 AC 5B    //发送读取mic版本号
2: FF FF 01 09 02 10 01 9C 53		 //发送读取声源唤醒角度
*********************************************************************************************************
*/
s8 Mic_Read(u8 unSlaveAddr, u8 *unSendBuffer,u16 unLength)	
{
	u8 unSendBuf[50],uni;
	u16 unCRC;

	unSendBuf[0] = (u8)MICHeader_H;         //帧头
	unSendBuf[1] = (u8)(MICHeader_H>>8);
	unSendBuf[2] = unSlaveAddr;
	unSendBuf[3] = 7 + unLength;
	unSendBuf[4] = MICRead_H;        
	
	for(uni=0;uni<unLength;uni++)//读寄存器
	{
		unSendBuf[uni+5]=unSendBuffer[uni];
	}
    
	unCRC=CRC16_Modbus(&unSendBuf[0],(5+unLength));
    
	unSendBuf[5+unLength] = (u8)unCRC&0XFF;
	unSendBuf[6+unLength] = (u8)(unCRC>>8);
    
	comSendBuf(COM8,unSendBuf,unSendBuf[3]);
    
	return ERR_NONE;
}
/**
*********************************************************************************************************
* @name MIC_WRITE
* @brief 写MIC寄存器
* @details 
*
* @param[in] unSlaveAddr 从站ID
* @param[in] unRegAddr   寄存器地址
* @param[in] unRegNum    寄存器个数
* @param[in] *unSendBuffer 写入数据
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*/
s8 Mic_Write(u8 unSlaveAddr,u8 unRegAddr,u16 unRegNum,u8 *unSendBuffer)	
{
	u8 unSendBuf[20];
    u8 uni;
	u16 unCRC;
    
	//MICAddHead(MICWrite_H,unSlaveAddr,unSendBuf,unSendLenth);

    unSendBuf[0] = (u8)MICHeader_H;//侦头
	unSendBuf[1] = (u8)(MICHeader_H>>8);
	unSendBuf[2] = unSlaveAddr;
	unSendBuf[3] = 7+unRegNum+2;
	unSendBuf[4] = MICWrite_H;    
    unSendBuf[5] = unRegAddr;	
	unSendBuf[6] = unRegNum;	
    
	for(uni=0;uni<unRegNum;uni++)//写寄存器
	{
	  unSendBuf[uni+7]=unSendBuffer[uni];
	}
    
	unCRC=CRC16_Modbus(&unSendBuf[0],(7+unRegNum));    
	unSendBuf[7+unRegNum] = (u8)unCRC;
	unSendBuf[8+unRegNum] = (u8)(unCRC>>8);
    
	comSendBuf(COM8,unSendBuf,unSendBuf[3]);
    
	return ERR_NONE;
}

/**
*********************************************************************************************************
* @name MICAddHead
* @brief 添加MIC帧头
* @details 
*
* @param[in] unKeyWord 功能码
* @param[in] unSlaveAddr 从站地址
* @param[in] *punBuffer  帧buf
* @param[in] unInfoLength 帧长度
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*/
/*
u8 MICAddHead(u8 unKeyWord,u8 unSlaveAddr,u8 *punBuffer,u16 unInfoLength)	
{
	punBuffer[0]=MICHeader_H&0XFF;//侦头
	punBuffer[1]=(MICHeader_H>>8)&0XFF;
	punBuffer[2]=unSlaveAddr;
	punBuffer[3]=unInfoLength&0XFF;
	punBuffer[4]=unKeyWord&0XFF;
		
	return 0;
}
*/


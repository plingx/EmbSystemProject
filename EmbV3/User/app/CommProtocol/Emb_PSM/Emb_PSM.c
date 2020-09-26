/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Emb_PSM.c
* @brief 本文件实现下位机与电源开关板之间的通信协议
* @details   下位机/电源开关板/充电控制板在同一个485总线上。并且电源开关板和充电控制板的从地址都是1。所以其实
*                      是靠帧头区分 电源开关板帧头是0xDDDD ,充电控制板帧头是0xEEEE
* @author chengyi@ewaybot.com
* @version 0.0.1
* @date 2018-1-24
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/1/24 | 0.0.1 | chengyi | Create file
*
*/

#include "includes.h"


//PSM添加帧头，私有
static s8 PSMAddHead(u8 *punBuffer,u8 unKeyWord,u8 unSlaveAddr,u8 unInfoLength);
///PSM读寄存器BUF
u8 unPSMRegAddr[2]={PSM_TriggerPowerOff,0x1F}; // 开始寄存器地址为0x36,往后读取0x36~0x56 共有0x1f个寄存器

extern QueueHandle_t EmbSysToPSMPSUHdl;
/**
*********************************************************************************************************
* @name PSM_PING
* @brief 对舵机变量初始化
* @details 
*
* @param[in] unSlaveAddr
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*/
s8 PSM_PING(u8 unSlaveAddr)    
{
    u8 unSendBuf[PSMPingLength_H];
    u16 unCRC;
    s8 uRes=0;
    ///添加头
    uRes=PSMAddHead(unSendBuf,PSMPing_H,unSlaveAddr,PSMPingLength_H);
    if(NULL==uRes)
    {
        return ERR_ADD_HEADER;
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[0],5);///校验
    unSendBuf[5]=unCRC&0XFF;
    unSendBuf[6]=(unCRC>>8)&0XFF;
    comSendBuf(COM7,unSendBuf,PSMPingLength_H);
    
    return ERR_NONE;
}
/**
*********************************************************************************************************
* @name SendQuery_PSM
* @brief 发送PSM查询指令
* @details 
*
* @param[in] unSlaveAddr 从站ID
* @param[in] *unSendBuffer 读寄存器BUF
* @param[in] unLength BUF长度
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*/
/*
s8 SendQuery_PSM(u8 unSlaveAddr, u8 *unSendBuffer,u8 unLen)    
{
    if(NULL==unSendBuffer)
        return ERR_POINTER_NULL;
    
    u8 unSendBuf[255]={0},unSendLen=0,uni=0;
    u16 unCRC=0;
    s8 sRet=0;
    
    unSendLen=(PSM_HEADER_LEN+CRC_LEN)+unLen;
    
    sRet=PSMAddHead(unSendBuf,PSMRead_H,unSlaveAddr,unSendLen);
    if(ERR_NONE!=sRet)
    {
        return ERR_ADD_HEADER;
    }
    
    for (uni=0;uni<unLen;uni++)//读寄存器
    {
        unSendBuf[uni+PSM_HEADER_LEN]=unSendBuffer[uni];
    }

    unCRC=CRC16_Modbus(&unSendBuf[0],unSendLen-2);
    unSendBuf[unSendLen-2]=unCRC&0XFF;
    unSendBuf[unSendLen-1]=(unCRC>>8)&0XFF;
    comSendBuf(COM7,unSendBuf,unSendLen);
    
    return ERR_NONE;
}*/
s8 SendQuery_PSM(u8 unSlaveAddr, u8 *unSendBuffer,u8 unLen)    
{
    u8 unSendBuf[255]={0},unSendLen=0,uni=0;
    u16 unCRC=0;
    s8 sRet=0;

    if(NULL==unSendBuffer)
    {
        return ERR_POINTER_NULL;
    }
    
    unSendLen=(PSM_HEADER_LEN+CRC_LEN)+unLen;
    
    sRet=PSMAddHead(&unSendBuf[2],PSMRead_H,unSlaveAddr,unSendLen);
    if(ERR_NONE!=sRet)
    {
        return ERR_ADD_HEADER;
    }
    
    for (uni=0;uni<unLen;uni++)//读寄存器
    {
        unSendBuf[uni+PSM_HEADER_LEN+2]=unSendBuffer[uni];
    }

    unCRC=CRC16_Modbus(&unSendBuf[2],unSendLen-2);
    unSendBuf[unSendLen]=unCRC&0XFF;
    unSendBuf[unSendLen+1]=(unCRC>>8)&0XFF;

    //send:
    unSendBuf[0] = COM_SEND_INTERVAL_PSM_QUERY;
    unSendBuf[1] = unSendLen;
    if(pdTRUE != xQueueSend(EmbSysToPSMPSUHdl,unSendBuf,0))
    {
        return ERR_PROCESS_FAILED;
    }    
    
    //comSendBuf(COM7,unSendBuf,unSendLen);
    
    return ERR_NONE;
}

/**
*********************************************************************************************************
* @name PSM_WRITE
* @brief 写PSM寄存器
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
s8 PSM_WRITE(u8 unSlaveAddr,u8 unRegAddr,u16 unRegNum,u8 *unSendBuffer)    
{
    u8 unSendBuf[256],unSendLen=0,uni;
    u16 unCRC;
    s8 sRet=0;

    if(NULL==unSendBuffer)
    {
        return ERR_POINTER_NULL;
    }
    
    unSendLen=7+unRegNum+2;
    
    sRet=PSMAddHead(&unSendBuf[2],PSMWrite_H,unSlaveAddr,unSendLen);
    
    if(ERR_NONE!=sRet)
    {
        return ERR_ADD_HEADER;
    }
    
    unSendBuf[7]=unRegAddr;    
    unSendBuf[8]=unRegNum;    
    
    for (uni=0;uni<unRegNum;uni++)//写寄存器
    {
      unSendBuf[uni+9]=unSendBuffer[uni];
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[2],unSendLen-2);
    unSendBuf[unSendLen]=unCRC&0XFF;
    unSendBuf[unSendLen+1]=(unCRC>>8)&0XFF;

    //send:
    unSendBuf[0] = COM_SEND_INTERVAL_PSM_CMD;      //!< 这个时间间隔是建立在已知目前下位机调用PSM_Write()发送的写指令寄存器个数均不超过2bytes的基础上.当数据发送过长时需重新确定时间间隔
    unSendBuf[1] = unSendLen;
    if(pdTRUE != xQueueSend(EmbSysToPSMPSUHdl,unSendBuf,0))
    {
        return ERR_PROCESS_FAILED;
    }    
    
    //comSendBuf(COM7,unSendBuf,unSendLen);

    return ERR_NONE;
}
/**
*********************************************************************************************************
* @name PSM_SET_Reborn
* @brief PSM重启
* @details 
*
* @param[in] unSlaveAddr 从站ID
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*/
s8 PSM_SET_Reborn(u8 unSlaveAddr)    
{
    u8 unSendBuf[7]={0},unSendLen=7;
    u16 unCRC=0;
    s8 sRet=0;

    sRet=PSMAddHead(unSendBuf,PSMReborn_H,unSlaveAddr,unSendLen);    
    if(ERR_NONE!=sRet)
    {
        return ERR_ADD_HEADER;
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[0],unSendLen-2);
    unSendBuf[5]=unCRC&0XFF;
    unSendBuf[6]=(unCRC>>8)&0XFF;
    comSendBuf(COM7,unSendBuf,unSendLen);
    
    return ERR_NONE;
}
/**
*********************************************************************************************************
* @name PSM_SET_INITE
* @brief PSM，复位
* @details 
*
* @param[in] unSlaveAddr 从站ID
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*/
s8 PSM_SET_INITE(u8 unSlaveAddr)    
{
    u8 unSendBuf[7]={0},unSendLen=7;
    u16 unCRC;
    s8 sRet=0;

    sRet=PSMAddHead(unSendBuf,PSMSetInit_H,unSlaveAddr,unSendLen);
    if(ERR_NONE!=sRet)
    {
        return ERR_ADD_HEADER;
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[0],unSendLen-2);
    unSendBuf[5]=unCRC&0XFF;
    unSendBuf[6]=(unCRC>>8)&0XFF;
    comSendBuf(COM7,unSendBuf,unSendLen);
    
    return ERR_NONE;
}
/**
*********************************************************************************************************
* @name PSMAddHead
* @brief 添加PSM帧头
* @details 
*
* @param[in] unKeyWord 功能码
* @param[in] unSlaveAddr 从站地址
* @param[in] *punBuffer  帧buf
* @param[in] unInfoLen 帧长度
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*/
s8 PSMAddHead(u8 *punBuffer,u8 unKeyWord,u8 unSlaveAddr,u8 unInfoLen)    
{
    if(NULL==punBuffer)
    {
        return ERR_POINTER_NULL;
    }
    
    punBuffer[0]=PSMHeader_H&0XFF;//侦头
    punBuffer[1]=(PSMHeader_H>>8)&0XFF;
    punBuffer[2]=unSlaveAddr;
    punBuffer[3]=unInfoLen&0XFF;
    punBuffer[4]=unKeyWord&0XFF;
        
    return ERR_NONE;
}



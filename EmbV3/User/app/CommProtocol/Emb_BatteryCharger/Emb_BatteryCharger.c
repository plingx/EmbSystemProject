/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Emb_BatteryCharger.c
* @brief 本文件实现下位机与充电控制板之间的通信协议
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

extern QueueHandle_t EmbSysToPSMPSUHdl;

//私有
static s8 BatteryChargerAddHead(u8 *punBuffer,u8 unKeyWord,u8 unSlaveAddr,u8 unInfoLen);    
///读寄存器BUF

/**
*********************************************************************************************************
* @name BatteryCharger_PING
* @brief 电池板PING
* @details 
*
* @param[in] unSlaveAddr 从站ID
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*/
s8 BatteryCharger_PING(u8 unSlaveAddr)    
{
    u8 unSendBuf[BatteryPingLength_H+2]={0};
    s8 sRet;
    u16 unCRC;
    
    sRet=BatteryChargerAddHead(&unSendBuf[2],BatteryPing_H,unSlaveAddr,BatteryPingLength_H);
    
    if(ERR_NONE!=sRet)
    {
        return ERR_ADD_HEADER;
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[2],5);
    
    unSendBuf[7]=unCRC&0XFF;
    unSendBuf[8]=(unCRC>>8)&0XFF;
    
    //send:
    unSendBuf[0] = COM_SEND_INTERVAL_PSU_QUERY;
    unSendBuf[1] = BatteryPingLength_H;
    
    if(pdTRUE != xQueueSend(EmbSysToPSMPSUHdl,unSendBuf,0))
    {
        return ERR_PROCESS_FAILED;
    }    
    
    //comSendBuf(COM7,unSendBuf,BatteryPingLength_H);
    
    return ERR_NONE;
}
/**
*********************************************************************************************************
* @name SendQuery_BatteryCharger
* @brief 发送电池充电板查询指令
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
s8 SendQuery_BatteryCharger(u8 unSlaveAddr, u8 *unSendBuffer, u8 unLen)    
{
    
    u8 unSendBuf[255]={0}, unSendLen=0, uni=0;
    s8 sRet=0;
    u16 unCRC=0;
    
    if(NULL==unSendBuffer)
    {
        return ERR_POINTER_NULL;
    }   
        
    unSendLen=(BATTERY_HEADER_LEN+CRC_LEN)+unLen;
    
    sRet=BatteryChargerAddHead(&unSendBuf[2],BatteryRead_H,unSlaveAddr,unSendLen);
    if(ERR_NONE!=sRet)
    {
        return ERR_ADD_HEADER;
    }
    
    for (uni=0;uni<unLen;uni++)//读寄存器
    {
        unSendBuf[uni+7]=unSendBuffer[uni];
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[2],unSendLen-2);
    unSendBuf[unSendLen]=unCRC&0XFF;
    unSendBuf[unSendLen+1]=(unCRC>>8)&0XFF;

    //send:
    unSendBuf[0] = COM_SEND_INTERVAL_PSU_QUERY;
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
* @name BatteryCharger_WRITE
* @brief 写电池板寄存器
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
s8 BatteryCharger_WRITE(u8 unSlaveAddr,u8 *unSendBuffer,u16 unLen)    
{
    u8 unSendBuf[256]={0},unSendLen=0,uni=0;
    u16 unCRC=0;
    s8 sRet=0;
    unSendLen=(BATTERY_HEADER_LEN+CRC_LEN)+unLen;
    
    sRet=BatteryChargerAddHead(&unSendBuf[2],BatteryWrite_H,unSlaveAddr,unSendLen);    
    if(ERR_NONE!=sRet)
    {
        return ERR_ADD_HEADER;
    }
    
    for (uni=0;uni<unLen;uni++)//写寄存器
    {
        unSendBuf[uni+BATTERY_HEADER_LEN+2]=unSendBuffer[uni];
    }
    unCRC=CRC16_Modbus(&unSendBuf[2],unSendLen-2);
    unSendBuf[unSendLen]=unCRC&0XFF;
    unSendBuf[unSendLen+1]=(unCRC>>8)&0XFF;

    //send:
    unSendBuf[0] = COM_SEND_INTERVAL_PSU_QUERY;
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
* @name BatteryCharger_SET_INITE
* @brief 电池板初始化
* @details 
*
* @param[in] unSlaveAddr 从站ID
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*/
s8 BatteryCharger_SET_INITE(u8 unSlaveAddr)    
{
    u8 unSendBuf[9],unSendLen=7;
    u16 unCRC;
    s8 sRet=0;
    
    sRet=BatteryChargerAddHead(&unSendBuf[2],BatterySetInit_H,unSlaveAddr,unSendLen);
    if(ERR_NONE!=sRet)
    {
        return ERR_ADD_HEADER;
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[2],unSendLen-2);
    unSendBuf[7]=unCRC&0XFF;
    unSendBuf[8]=(unCRC>>8)&0XFF;

    //send:
    unSendBuf[0] = COM_SEND_INTERVAL_PSU_QUERY;
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
* @name BatteryChargerAddHead
* @brief 添加帧头
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
s8 BatteryChargerAddHead(u8 *punBuffer,u8 unKeyWord,u8 unSlaveAddr,u8 unInfoLen)    
{
    if(NULL==punBuffer)
        return ERR_POINTER_NULL;
    
    punBuffer[0]=BatteryHeader_H&0XFF;//侦头
    punBuffer[1]=(BatteryHeader_H>>8)&0XFF;
    punBuffer[2]=unSlaveAddr;
    punBuffer[3]=unInfoLen&0XFF;
    punBuffer[4]=unKeyWord&0XFF;

    return ERR_NONE;
}


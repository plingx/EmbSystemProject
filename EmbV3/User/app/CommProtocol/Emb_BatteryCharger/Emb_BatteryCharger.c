/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Emb_BatteryCharger.c
* @brief ���ļ�ʵ����λ��������ư�֮���ͨ��Э��
* @details   ��λ��/��Դ���ذ�/�����ư���ͬһ��485�����ϡ����ҵ�Դ���ذ�ͳ����ư�Ĵӵ�ַ����1��������ʵ
*                      �ǿ�֡ͷ���� ��Դ���ذ�֡ͷ��0xDDDD ,�����ư�֡ͷ��0xEEEE
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

//˽��
static s8 BatteryChargerAddHead(u8 *punBuffer,u8 unKeyWord,u8 unSlaveAddr,u8 unInfoLen);    
///���Ĵ���BUF

/**
*********************************************************************************************************
* @name BatteryCharger_PING
* @brief ��ذ�PING
* @details 
*
* @param[in] unSlaveAddr ��վID
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
* @brief ���͵�س����ѯָ��
* @details 
*
* @param[in] unSlaveAddr ��վID
* @param[in] *unSendBuffer ���Ĵ���BUF
* @param[in] unLength BUF����
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
    
    for (uni=0;uni<unLen;uni++)//���Ĵ���
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
* @brief д��ذ�Ĵ���
* @details 
*
* @param[in] unSlaveAddr ��վID
* @param[in] *unSendBuffer ���Ĵ���BUF
* @param[in] unLength BUF����
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
    
    for (uni=0;uni<unLen;uni++)//д�Ĵ���
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
* @brief ��ذ��ʼ��
* @details 
*
* @param[in] unSlaveAddr ��վID
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
* @brief ���֡ͷ
* @details 
*
* @param[in] unKeyWord ������
* @param[in] unSlaveAddr ��վ��ַ
* @param[in] *punBuffer  ֡buf
* @param[in] unInfoLength ֡����
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
    
    punBuffer[0]=BatteryHeader_H&0XFF;//��ͷ
    punBuffer[1]=(BatteryHeader_H>>8)&0XFF;
    punBuffer[2]=unSlaveAddr;
    punBuffer[3]=unInfoLen&0XFF;
    punBuffer[4]=unKeyWord&0XFF;

    return ERR_NONE;
}


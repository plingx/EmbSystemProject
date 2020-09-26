/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Emb_PSM.c
* @brief ���ļ�ʵ����λ�����Դ���ذ�֮���ͨ��Э��
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


//PSM���֡ͷ��˽��
static s8 PSMAddHead(u8 *punBuffer,u8 unKeyWord,u8 unSlaveAddr,u8 unInfoLength);
///PSM���Ĵ���BUF
u8 unPSMRegAddr[2]={PSM_TriggerPowerOff,0x1F}; // ��ʼ�Ĵ�����ַΪ0x36,�����ȡ0x36~0x56 ����0x1f���Ĵ���

extern QueueHandle_t EmbSysToPSMPSUHdl;
/**
*********************************************************************************************************
* @name PSM_PING
* @brief �Զ��������ʼ��
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
    ///���ͷ
    uRes=PSMAddHead(unSendBuf,PSMPing_H,unSlaveAddr,PSMPingLength_H);
    if(NULL==uRes)
    {
        return ERR_ADD_HEADER;
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[0],5);///У��
    unSendBuf[5]=unCRC&0XFF;
    unSendBuf[6]=(unCRC>>8)&0XFF;
    comSendBuf(COM7,unSendBuf,PSMPingLength_H);
    
    return ERR_NONE;
}
/**
*********************************************************************************************************
* @name SendQuery_PSM
* @brief ����PSM��ѯָ��
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
    
    for (uni=0;uni<unLen;uni++)//���Ĵ���
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
    
    for (uni=0;uni<unLen;uni++)//���Ĵ���
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
* @brief дPSM�Ĵ���
* @details 
*
* @param[in] unSlaveAddr ��վID
* @param[in] unRegAddr   �Ĵ�����ַ
* @param[in] unRegNum    �Ĵ�������
* @param[in] *unSendBuffer д������
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
    
    for (uni=0;uni<unRegNum;uni++)//д�Ĵ���
    {
      unSendBuf[uni+9]=unSendBuffer[uni];
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[2],unSendLen-2);
    unSendBuf[unSendLen]=unCRC&0XFF;
    unSendBuf[unSendLen+1]=(unCRC>>8)&0XFF;

    //send:
    unSendBuf[0] = COM_SEND_INTERVAL_PSM_CMD;      //!< ���ʱ�����ǽ�������֪Ŀǰ��λ������PSM_Write()���͵�дָ��Ĵ���������������2bytes�Ļ�����.�����ݷ��͹���ʱ������ȷ��ʱ����
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
* @brief PSM����
* @details 
*
* @param[in] unSlaveAddr ��վID
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
* @brief PSM����λ
* @details 
*
* @param[in] unSlaveAddr ��վID
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
* @brief ���PSM֡ͷ
* @details 
*
* @param[in] unKeyWord ������
* @param[in] unSlaveAddr ��վ��ַ
* @param[in] *punBuffer  ֡buf
* @param[in] unInfoLen ֡����
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
    
    punBuffer[0]=PSMHeader_H&0XFF;//��ͷ
    punBuffer[1]=(PSMHeader_H>>8)&0XFF;
    punBuffer[2]=unSlaveAddr;
    punBuffer[3]=unInfoLen&0XFF;
    punBuffer[4]=unKeyWord&0XFF;
        
    return ERR_NONE;
}



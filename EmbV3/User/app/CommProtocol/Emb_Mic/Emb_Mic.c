/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Emb_Mic.c
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
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





//MIC���֡ͷ��˽��
u8 MICAddHead(u8 unKeyWord,u8 unSlaveAddr,u8 *punBuffer,u16 unInfoLength);
///MIC���Ĵ���BUF
u8 unMICVerRegAddr[2]={0x0F,0x01};//mic�汾�Ĵ�����ַ
u8 unMICAngRegAddr[2]={0x10,0x01};//mic������Դ�Ĵ�����ַ
/**
*********************************************************************************************************
* @name MIC_PING
* @brief ���MIc���Ƿ�����
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
	unSendBuf[0] = (u8)MICHeader_H;//��ͷ
	unSendBuf[1] = (u8)(MICHeader_H>>8);
	unSendBuf[2] = unSlaveAddr;
	unSendBuf[3] = MICPingLength_H;
	unSendBuf[4] = MICPing_H;
    
	unCRC=CRC16_Modbus(&unSendBuf[0],5);//У��
	unSendBuf[5] = (u8)unCRC;
	unSendBuf[6] = (u8)(unCRC>>8);
    
	comSendBuf(COM8,unSendBuf,MICPingLength_H);
    
	return ERR_NONE;
}
/**
*********************************************************************************************************
* @name MIC_Read
* @brief ��MIC�Ĵ���
* @details 
*
* @param[in] unSlaveAddr ��վID
* @param[in] *unSendBuffer ���Ĵ���BUF
* @param[in] unLength BUF����
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
1: FF FF 01 09 02 0F 01 AC 5B    //���Ͷ�ȡmic�汾��
2: FF FF 01 09 02 10 01 9C 53		 //���Ͷ�ȡ��Դ���ѽǶ�
*********************************************************************************************************
*/
s8 Mic_Read(u8 unSlaveAddr, u8 *unSendBuffer,u16 unLength)	
{
	u8 unSendBuf[50],uni;
	u16 unCRC;

	unSendBuf[0] = (u8)MICHeader_H;         //֡ͷ
	unSendBuf[1] = (u8)(MICHeader_H>>8);
	unSendBuf[2] = unSlaveAddr;
	unSendBuf[3] = 7 + unLength;
	unSendBuf[4] = MICRead_H;        
	
	for(uni=0;uni<unLength;uni++)//���Ĵ���
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
* @brief дMIC�Ĵ���
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
s8 Mic_Write(u8 unSlaveAddr,u8 unRegAddr,u16 unRegNum,u8 *unSendBuffer)	
{
	u8 unSendBuf[20];
    u8 uni;
	u16 unCRC;
    
	//MICAddHead(MICWrite_H,unSlaveAddr,unSendBuf,unSendLenth);

    unSendBuf[0] = (u8)MICHeader_H;//��ͷ
	unSendBuf[1] = (u8)(MICHeader_H>>8);
	unSendBuf[2] = unSlaveAddr;
	unSendBuf[3] = 7+unRegNum+2;
	unSendBuf[4] = MICWrite_H;    
    unSendBuf[5] = unRegAddr;	
	unSendBuf[6] = unRegNum;	
    
	for(uni=0;uni<unRegNum;uni++)//д�Ĵ���
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
* @brief ���MIC֡ͷ
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
/*
u8 MICAddHead(u8 unKeyWord,u8 unSlaveAddr,u8 *punBuffer,u16 unInfoLength)	
{
	punBuffer[0]=MICHeader_H&0XFF;//��ͷ
	punBuffer[1]=(MICHeader_H>>8)&0XFF;
	punBuffer[2]=unSlaveAddr;
	punBuffer[3]=unInfoLength&0XFF;
	punBuffer[4]=unKeyWord&0XFF;
		
	return 0;
}
*/


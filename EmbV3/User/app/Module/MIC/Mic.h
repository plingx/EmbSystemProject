/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Mic.h
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2018-05-28
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/05/28 | 0.0.1 |          | Create file
*
*/
#ifndef _APP_MODULE_MIC_H_
#define _APP_MODULE_MIC_H_


#define MIC_HEADER_FLAG                 0xFFFF
#define MIC_RESPONSE_INFO_LEN_MIN	    0x08
#define MIC_RESPONSE_LENGTH_WRITE	    0x08
#define MIC_RESPONSE_LENGTH_PING	    0x08
#define MIC_RESPONSE_LENGTH_WkPos	    0x0A
#define MIC_RESPONSE_LENGTH_Versi	    0x10




#define EwayMicRunState_UnInitial               0x0000
#define EwayMicRunState_OnLine_SendReset        0x0001
#define EwayMicRunState_OnLine_SendWake         0x0002
#define EwayMicRunState_Initialized_RdVersi     0x0003
#define EwayMicRunState_Initialized_RdAngle     0x0004
#define EwayMicRunState_Run_SendReset           0x0005



typedef struct{
    u16 wakeup_angle;   //���ѽǶ�
    u8  verinfo[8];     //mic�汾��Ϣ
    u8  mic_online;     //0:δ����   1:����
    u8  rtReset;        //!< ��Set Resetָ��Ļظ�
    u8  rtWake;         //!< ��Wakeup ָ��Ļظ� 
}MicInfoModule;


/*
Mic����״̬ 

0:δ��ʼ��      ��ping �յ��ظ�                     ------>   1:����

1:����          ��ʼ������ResetMic openWakeup       ------>   2:�ѳ�ʼ��

2:��ʼ�����    ���ڶ�Angle&Version                 
    ����

*/
typedef struct{
    MicInfoModule MicInfo;      //!< Mic ��Ϣ
    u16 Last_wkAngle;           //!< ��һ�λ��ѵĽǶ�
    u16 runState;               //!< Mic����״̬ 0:δ��ʼ��
    u16 errCnt;                 //!< ���������
    u16 comCnt;                 //!< ��ȡ���������ͨ�����ڼ��������ڵ���
    u16 mCommStatus;            //!< every peripheral device communication status  bitx:0-disconnected,1-connection
}EwayMicModule;






void sysEmbMicMaintainWork(EwayMicModule* pModule);
s8 sysEmbResetMicBoard(u8 sAddr);
s8 sysEmbEnableMicWakeup(u8 sAddr);
s8 sysEmbReadMicInfo(u8 sAddr);
s8 sysEmbReadMicWakeupAngle(u8 sAddr);
s8 g_DataRecvdFromMicprocess(u8* pDat,u16 len);
s8 sysEmbEnhanceMicSoundBeam(u8 sAddr,u8 beam);



#endif

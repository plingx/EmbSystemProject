/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Mic.h
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
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
    u16 wakeup_angle;   //唤醒角度
    u8  verinfo[8];     //mic版本信息
    u8  mic_online;     //0:未在线   1:在线
    u8  rtReset;        //!< 对Set Reset指令的回复
    u8  rtWake;         //!< 对Wakeup 指令的回复 
}MicInfoModule;


/*
Mic运行状态 

0:未初始化      发ping 收到回复                     ------>   1:在线

1:在线          初始化，发ResetMic openWakeup       ------>   2:已初始化

2:初始化完成    周期读Angle&Version                 
    运行

*/
typedef struct{
    MicInfoModule MicInfo;      //!< Mic 信息
    u16 Last_wkAngle;           //!< 上一次唤醒的角度
    u16 runState;               //!< Mic运行状态 0:未初始化
    u16 errCnt;                 //!< 错误计数器
    u16 comCnt;                 //!< 获取正常结果的通信周期计数，用于调试
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

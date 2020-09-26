/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Shoulder.h
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2017-12-20
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/01/15 | 0.0.1 | xx       | Create file
*
*/
#ifndef _APP_MODULE_SHOULDER_H_
#define _APP_MODULE_SHOULDER_H_



#define CMD_SHOULDER2MOTOR_INFO_LEN    64   //!< define the Emb send to shoulder motor cmd buffer length  
#define CMD_SHOULDER2MOTOR_QUEU_NUM    10    //!< 每个周期可以向电机发送的最大指令数目，可以根据实际需要进行修改。


#define SHOULDER_LIMIT_TRIGGER_BIT     0x03//!< 每30ms检查一次上下限位管脚状态，若连续两次检测到管脚状态为0,则认为已经撞到了上下限位了。
#define SHOULDER_LIMIT_UP              1
#define SHOULDER_LIMIT_DN              0





/*
typedef struct{
    MotorCMDBuff* pToMotorCmd;         //!< pointer of shoulder cmd buffer    
    MotorReducRatio* pMotorRatio;      //!< 
    Motor_CtrlMode mCtrlMode;          //!< motor control mode
    Motor_ApplicationMode  mAppMode;   //!< motro application mode
    Motor_PosClosedLoopCtrlMode mPosClsedLoopCtrlMode;
    u16 posKeep;                         //!< 当上位机发送完Clear指令后，是否需要位置保持的标志，若保持，则不为0
}NodeCtrlModule;
*/


#define UpDnLimShlCtrl_Off      0x00
#define UpDnLimShlCtrl_SpdCmd   0x01
#define UpDnLimShlCtrl_PosCmd   0x02


typedef struct{
    u8 en;                      //!< 在压住上下限位时 标记当前是否使能阻止运动继续的开关，当使能时，当前肩膀电机无指令但有压住上限位or下限位了，则不阻止运动，继续进行，直到无限位被压住时清除标志
    u16 SpdCmd;                 //!< 记录使能此功能的速度指令值
    s32 PosCmd;                 //!< 记录使能此功能的速度指令值
}UpDownLimitShoulderCtrl;


typedef struct{
    u8 upDnInitFlag;                       //!< bit0-3 low limit,bit4-7 high limit
    u8 pinState[2];                        //!< up/dn Limit pin state 0:dn 1:up
    s32 mPosCurr[2];                       //!< motor wheel mode position realtime     
    s32 mPosInit[2];                       //!< motor wheel mode position when up/dn   撞到上下限位时的码盘值
    UpDownLimitShoulderCtrl mCtrl;
}UpDownLimitCtrl;


typedef struct{
    u8 mRegs[SERVO_REG_READ_NUMS_MAX];    //!< store data received from motors every cycle
}MotorSTATEBuff;


typedef struct{
    u8 wRegs[Write_Regs_Record_Max_Space];  //!< 记录每个周期给电机下发的指令
}MotorRecordBuff;


typedef struct{
    MotorSTATEBuff* pFromMotor;
    MotorRecordBuff* pMotorRecord;
    u8* pMCfgReg;
}NodeStateModule;

typedef struct{
    NodeCtrlModule mControl;           //!< motor control parameters
    UpDownLimitCtrl shLimit;           //!< shoulder up down limit control mode variables
    NodeStateModule mState;
    MotorReRecord mResp;                //!< 对肩膀电机回复的记录
    u16 mCount;                        //!< number of motors
    u16 mCommStatus;                   //!< every motor communication status  bitx:0-disconnected,1-connection
    u8* pmPid;
}EwayShoulderModule;




#include "Shoulder_Cmd.h"
#include "Shoulder_Status.h"


void sysPcToEmbCmdProcess(EwayMotor_Device_Type devType,QueueHandle_t* pQueHandle);
void sysEmbToMotorCmdProcess(EwayMotor_Device_Type devType);
s8 sysGetLimitSwitchStatus(EwayShoulderModule* pModule);
s8 sysPcToShoulderCmdProcess(u16 Code,u32 timStamp,u16 dlen,u8* pdat);
s8 sysGeneralGetEmbToMotorCmd(MotorCMDBuff* pmotorCmd,u8* pDat);
s8 g_StorePcCmdtoCmdBuff(MotorCMDBuff* pCmd,u16 id,u8* pdat,u8 len);
s8 g_ClearAllPartPcToXCmdBuff(EwayMotor_Device_Type devType,QueueHandle_t* pQue,MotorCMDBuff* pCmdBuff);




#endif


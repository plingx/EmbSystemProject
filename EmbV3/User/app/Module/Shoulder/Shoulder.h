/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Shoulder.h
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
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
#define CMD_SHOULDER2MOTOR_QUEU_NUM    10    //!< ÿ�����ڿ����������͵����ָ����Ŀ�����Ը���ʵ����Ҫ�����޸ġ�


#define SHOULDER_LIMIT_TRIGGER_BIT     0x03//!< ÿ30ms���һ��������λ�ܽ�״̬�����������μ�⵽�ܽ�״̬Ϊ0,����Ϊ�Ѿ�ײ����������λ�ˡ�
#define SHOULDER_LIMIT_UP              1
#define SHOULDER_LIMIT_DN              0





/*
typedef struct{
    MotorCMDBuff* pToMotorCmd;         //!< pointer of shoulder cmd buffer    
    MotorReducRatio* pMotorRatio;      //!< 
    Motor_CtrlMode mCtrlMode;          //!< motor control mode
    Motor_ApplicationMode  mAppMode;   //!< motro application mode
    Motor_PosClosedLoopCtrlMode mPosClsedLoopCtrlMode;
    u16 posKeep;                         //!< ����λ��������Clearָ����Ƿ���Ҫλ�ñ��ֵı�־�������֣���Ϊ0
}NodeCtrlModule;
*/


#define UpDnLimShlCtrl_Off      0x00
#define UpDnLimShlCtrl_SpdCmd   0x01
#define UpDnLimShlCtrl_PosCmd   0x02


typedef struct{
    u8 en;                      //!< ��ѹס������λʱ ��ǵ�ǰ�Ƿ�ʹ����ֹ�˶������Ŀ��أ���ʹ��ʱ����ǰ�������ָ���ѹס����λor����λ�ˣ�����ֹ�˶����������У�ֱ������λ��ѹסʱ�����־
    u16 SpdCmd;                 //!< ��¼ʹ�ܴ˹��ܵ��ٶ�ָ��ֵ
    s32 PosCmd;                 //!< ��¼ʹ�ܴ˹��ܵ��ٶ�ָ��ֵ
}UpDownLimitShoulderCtrl;


typedef struct{
    u8 upDnInitFlag;                       //!< bit0-3 low limit,bit4-7 high limit
    u8 pinState[2];                        //!< up/dn Limit pin state 0:dn 1:up
    s32 mPosCurr[2];                       //!< motor wheel mode position realtime     
    s32 mPosInit[2];                       //!< motor wheel mode position when up/dn   ײ��������λʱ������ֵ
    UpDownLimitShoulderCtrl mCtrl;
}UpDownLimitCtrl;


typedef struct{
    u8 mRegs[SERVO_REG_READ_NUMS_MAX];    //!< store data received from motors every cycle
}MotorSTATEBuff;


typedef struct{
    u8 wRegs[Write_Regs_Record_Max_Space];  //!< ��¼ÿ�����ڸ�����·���ָ��
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
    MotorReRecord mResp;                //!< �Լ�����ظ��ļ�¼
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


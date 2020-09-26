/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file EmbSysProc.h
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author xxx, xxx@ewaybot.com
* @version 0.0.1
* @date 2000-01-01
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/01/29 | 0.0.1 | Ling     | Create file
*
*/
#ifndef _EMB_SYSPROC_H_
#define _EMB_SYSPROC_H_


#define CMD_EMBSYSPROC_INFO_LEN    12
#define CMD_EMBSYSPROC_QUEU_NUM    10

#define SystemInfo_EpromStartAddr    0       //!< 系统参数存储在eeprom的起始地址
#define EEPROM_ADDRESS_MAX         0x1FFFF   //!< (AT24C1024 128KByte 2的次方)
#define EEPROM_PAGE_CAPACITY       (0x0100)    //!< 每页256字节

#define LastPosAvailableOff          0x00
#define LastPosAvailablePosOn        0x01
#define LastPosAvailableSpdOn        0x02


typedef struct{
    u8 sQueryRecd;    //!< bit7-4 发送Query个数      bit3-0 收到 fCode=0x02 的回复个数
	u8 sExeCmRecd;    //!< bit7-4 发送写指令个数     bit3-0 收到 fCode=0x03 的回复个数
}PSMPSUReRecord;        //!< 发送数据包的记录与收到回复的记录


typedef struct{
    u8 avail;       //!< 保存的最后一条位置指令是否可用     
    s32 lastPos;    //!< 最后一条指令的位置
    s32 tmpPos;    
}Motor_LastPosCmd;


typedef struct{
    MotorCMDBuff* pToMotorCmd;         //!< pointer of shoulder cmd buffer    
    MotorReducRatio* pMotorRatio;      //!< 
    Motor_CtrlMode mCtrlMode;          //!< motor control mode
    Motor_ApplicationMode  mAppMode;   //!< motro application mode
    Motor_PosClosedLoopCtrlMode mPosClsedLoopCtrlMode;
    Motor_LastPosCmd* pmlstPos;
    u16 posKeep;                         //!< 当上位机发送完Clear指令后，是否需要位置保持的标志，若保持，则不为0
}NodeCtrlModule;



typedef struct{
    Motor_ApplicationMode           mAppMode;               //!< reg0x07,电机应用模式
    Motor_CtrlMode                  mCtrlMode;              //!< reg0x08,电机控制模式
    Motor_PosClosedLoopCtrlMode     mPosClsLopMode;         //!< reg0x09,电机位置闭环控制类型
    Motor_ReductionRatioMode        mReductionRatioMode;    //!< reg0x0B&0x0C,电机减速比
}EwayEmbSysMotorInfoModule;



s8 sysPcToEmbsysCmdProcess(u16 Code,u32 timStamp,u16 dlen,u8* pdat);
s8 EmbSys_Init(void);
//void sysEmbCommStateUpdate(void);
//void sysEmbCommStateMonitor(void);
void sysEmbCommStateUpdateAndMonitor(void);
void sysEmbMotorsCommSupervise(void);
void sysEmbCheckWheelMotorsStatus(void);
s8 SysE2PromWriteProtectFuncSet(u8 sta);
s8 SysE2PromWriteProtectFuncGet(u8* psta);
s8 SysE2PromWrite(u32 addr,u8* pdat,u32 nums);
s8 SysE2PromRead(u32 addr,u8* pdat,u32 nums);
void sysProcessRecvDataFromPsmPsu(void);
void sysProcessRecvDataFromMic(void);
s8 sysReadEmbSystemInfoFromE2prom(void);
s8 ReadInfoFromE2prom(void);
s8 g_ClearPcCmdtoNodeCmdBuff(MotorCMDBuff* pCmd,u16 id);
s8 sysEmbCheckMotorConfigRegs(NodeCtrlModule* pmCtrl,u8 mIdx,u8* pReg);
s8 sysInitReadAndCheckEmbMotorsCfgRegs(void);
void sysCyclicalCheckEmbMotorsCfgRegs(void);
void sysClrPcToEmbPartsCmdBuff(void);
s8 EmbRecordingLastPosCmdSendtoMotor(Motor_LastPosCmd* pLstPos,u8 Idx,s32 pos);
void sysUpdateMotorsLastPosCmd(void);
s8 EmbGetLastPosCmdSendtoMotor(Motor_LastPosCmd* pLstPos,u8 Idx,s32* pRslt);
s8 EmbUpdateLastPosCmdSendtoMotor(Motor_LastPosCmd* pLstPos,u8 Idx);
s8 EmbClrLastPosCmdAvailableFlag(Motor_LastPosCmd* pLstPos,u8 Idx);
s8 EmbEnableLastPosCmdAvailableFlag(Motor_LastPosCmd* pLstPos,u8 Idx);
void sysEmbMotorsCommSupervise(void);
s8 sysSendFreeStopToWheel(u8 en);
void sysPutWheelMotorsIntoFreeStopMode(void);
void sysPutWheelMotorsIntoStopOffMode(void);
s8 EmbRecordCmdSendToMotor(u8 Idx,Motor_CtrlMode sCtrlMode,s16 sSpd,s32 sPosi,EwayMotor_Device_Type devType);
s8 sysEmbCheckMotorAppCtrlModeInConfigRegs(NodeCtrlModule* pmCtrl,u8 mIdx,u8* pReg);
s8 EmbConfigMotorsParametersForArms(u8 stId,u8 nums,EwayEmbSysMotorInfoModule* pMotor);
s8 EmbConfigMotorsParametersForHeads(u8 stId,u8 nums,EwayEmbSysMotorInfoModule* pMotor);
s8 EmbConfigMotorsParametersForShoulder(u8 stId,u8 nums,EwayEmbSysMotorInfoModule* pMotor);
s8 EmbConfigMotorsParametersForWheels(u8 stId,u8 nums,EwayEmbSysMotorInfoModule* pMotor);
void sysEmbCheckWheelMotorsCommuStatus(void);


#endif

/***************************** END OF FILE *********************************/

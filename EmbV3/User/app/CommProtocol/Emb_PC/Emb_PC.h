/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Emb_PC.h
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
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
#ifndef _EMB_PC_H_
#define _EMB_PC_H_


#define CMD_PC2SHOULDER_INFO_LEN         24   //!< define the PC send shoulder cmd buffer length  ，最长20Bytes
#define CMD_PC2SHOULDER_QUEU_NUM         18

#define CMD_PC2ARMS_INFO_LEN             84   //!< define the PC send Arms cmd buffer length  ，最长82Bytes
#define CMD_PC2ARMS_QUEU_NUM             18

#define CMD_PC2HEAD_INFO_LEN             24   //!< define the PC send Head cmd buffer length  ，最长22Bytes
#define CMD_PC2HEAD_QUEU_NUM             18

#define CMD_PC2WHEEL_INFO_LEN            28   //!< define the PC send Wheel cmd buffer length  ，最长26Bytes
#define CMD_PC2WHEEL_QUEU_NUM            18

#define CMD_PC2GRIPPER_INFO_LEN          28   //!< define the PC send Gripper cmd buffer length  ，最长26Bytes
#define CMD_PC2GRIPPER_QUEU_NUM          18

#define CMD_PC2EMBSYS_INFO_LEN           20   //!< define the PC send EmbSystem cmd buffer length  ，最长18Bytes
#define CMD_PC2EMBSYS_QUEU_NUM           10

#define CMD_EMBWRLOG_INFO_LEN            128 //!< define the PC send shoulder cmd buffer length  ，最长128Bytes
#define CMD_EMBWRLOG_QUEU_NUM            10

#define CMD_EMBSYS2PSM_INFO_LEN          16   //!< define the EmbSystem send cmd to PSM buffer length  ，最长16Bytes
#define CMD_EMBSYS2PSM_QUEU_NUM           4


/**
* Define the CMD To PC.
*/
#define  PSU_Status                                                0x0000  //!< 电池状态
#define  PSM_Status                                                0x0001  //!<电源开关板状态
#define  IMU_Status                                                0x0002  //!<IMU状态
#define  TOF_Status                                                0x0003
#define  RDS_Status                                                0x0004
#define  MIC_Status                                                0x0005
#define  TOF_NUM                                                   6

#define  SingleCy_Motor_NUM                                        6  //!< 目前就先上传左臂6个的


//保护状态和运动状态合并
#define  Motor_Status                                               0x0010  //!< 舵机运动状态  双臂+头
#define  Shoulder_Status                                            0x0011  //!< 舵机运动状态  肩部
#define  Wheel_Status                                               0x0020  //!< 轮子运动状态
#define  Manipulation_Status                                        0x0030  //!< 爪子运动状态
#define  SingleCycle_JointMotors_Status                             0x0040  //< 单周期舵机指令与状态
#define  SingleCycle_WheelMotors_Status                             0x0041  //< 单周期舵机指令与状态
#define  EmbSystemSoftwareVersion_Info                              0x00F0  //!< 下位机软件版本号信息

#define  EmbSTOP_Preparing                                          0x00FE  //!< 关机报备
/**
* Define the CMD Len to PC.
*/

#define  PSU_Status_Length                                         0x17  //!< 电池状态数据长度
#define  PSM_Status_Length                                         0x2a  //!< 电源开关板状态长度
#define  IMU_Status_Length                                         0x1E  //!< IMU状态长度
#define  TOF_Status_Length                                         (0x0A+2*(TOF_NUM+1))  //!< TOF状态长度
#define  RDS_Status_Length                                         0x17  //!< RDS状态长度*/
#define  MIC_Status_Length                                         0x10
#define  EmbSysSoftVer_Length                                      0x11  //!< 软件版本信息数据长度

//保护状态与运动状态合并
#define  Motor_Status_Length                                (0x0F*15+0xA+2)  //!< 舵机状态*长度
#define  Shoulder_Status_Lenght                             (0x13+0x0A)     //!< 一个肩膀的长度   
#define  Wheel_Status_Length                                (0x11*2+0x0A)  //!< 轮子状态长度
#define  Manipulation_Status_Length                         (0x0F*2+0x0A)  //!< 爪子状态*长度

#define  SingleCycle_Motor_Unit_Length                      0x0E    //!> id(2) + DnPos(4) + DnSpd(2) + UpPos(4) + UpSpd(2)
#define  SingleCycle_Motors_Status_Length                   (SingleCycle_Motor_Unit_Length*SingleCy_Motor_NUM+0x0A) //!< 单周期舵机指令&状态 包长度

#define  EmbSTOP_Preparing_Length                                  0x0C  //!< 关机报备*长度


/**
* Define the CMD with PC.
*/
//===============================================================================================//
#define  PC_CMD_FRAME_HEADER                                      0xE1FE   //!< 帧头
#define  MAX_ATE_LENTH                                            500
#define  PC_CMD_HEAD_LENTH                                        10      
#define  PC_CMD_PKT_LEN_MAX                                       128      //!< 限定指令包长度不大于128(cdm+timeStamp+datalen+Datas....)


#define  PCCmd_ArmPosMoveMode                                        0x0110   //!< 手臂位置模式
#define  PCCmd_ArmSpdMoveMode                                        0x0111   //!< 手臂速度模式
#define  PCCmd_ArmPIDMode                                            0x0112   //!< 手臂PID控制
#define  PCCmd_ArmTorqueSwitch                                       0x0113   //!< 手臂转矩开关
#define  PCCmd_ArmMotorLimit                                         0x0114   //!< 手臂转矩限制

#define  PCCmd_HeadPosMove                                           0x0120   //!< 头位置模式
#define  PCCmd_HeadSpeedMove                                         0x0121   //!< 头速度模式
#define  PCCmd_HeadPID                                               0x0122   //!< 头PID控制
#define  PCCmd_HeadTorqueSwitch                                      0x0123   //!< 头转矩开关
#define  PCCmd_HeadMotorLimit                                        0x0124   //!< 头转矩限制
 
#define  PCCmd_WheelPositionMove                                     0x0130   //!< 轮子位置模式
#define  PCCmd_WheelSpeedMove                                        0x0131   //!< 轮子速度模式
#define  PCCmd_WheelPID                                              0x0132   //!< 轮子PID控制
#define  PCCmd_WheelTorqueSwitch                                     0x0133   //!< 轮子转矩开关
#define  PCCmd_WheelMotorLimit                                       0x0134   //!< 轮子转矩限制

#define  PCCmd_GripperMove                                           0x0140   //!< 爪子运动
#define  PCCmd_GripperPID                                            0x0142   //!< 爪子PID控制
#define  PCCmd_GripperTorqueSwitch                                   0x0143   //!< 爪子转矩开关
#define  PCCmd_GripperMotorLimit                                     0x0144   //!< 爪子转矩限制

#define  PCCmd_ShoulderPosMoveMode                                   0x0160   //!< 肩膀位置模式
#define  PCCmd_ShoulderSpdMoveMode                                   0x0161   //!< 肩膀速度模式
#define  PCCmd_ShoulderPIDMode                                       0x0162   //!< 肩膀PID控制
#define  PCCmd_ShoulderTorqueSwitch                                  0x0163   //!< 肩膀转矩开关
#define  PCCmd_ShoulderMotorLimit                                    0x0164   //!< 肩膀转矩限制

#define  PCCmd_Led                                                   0x0150   //!< 灯闪烁状态
#define  PCCmd_Clear                                                 0x0100   //!< 清除命令
#define  PCCmd_ObstclAvoid                                           0x0101   //!< 避障开关命令
#define  PCCmd_HeartBeat                                             0x0102   //!< 心跳包
#define  PCCmd_SysStop                                               0x01FE   //!< 关机指令
#define  PCCmd_Laser                                                 0x01FF   //!< 激光开关命令

#define  PCCmd_WheelStop                                             (s16)0x8000   //!< 关机指令
#define  PCCmd_ServoStop                                             0   //!< 关机指令
#define  PCCmd_Motor_Invilid                                            30001   //!< 

#define  PCCmd_DebugCtrl                                             0xE001   //!< Debug控制指令


#define PCCmd_ArmPosMoveMode_Unit                                    6
#define PCCmd_ArmSpdMoveMode_Unit                                    6
#define PCCmd_HeadPosMove_Unit                                       6
#define PCCmd_HeadSpeedMove_Unit                                     6
#define PCCmd_ShoulderPosMoveMode_Unit                               10     //!< 8
#define PCCmd_ShoulderSpdMoveMode_Unit                               10     //!< 8
#define PCCmd_ShoulderPosPIDSet_Unit                                 8
#define PCCmd_WheelPositionMove_Unit                                 8
#define PCCmd_WheelSpeedMove_Unit                                    4
#define PCCmd_WheelPosPIDSet_Unit                                    8
#define PCCmd_GripperMove_Unit                                       6
#define PCCmd_GripperTorqueSw_Unit                                   3
#define PCCmd_GripperTorqueLm_Unit                                   4
#define PCCmd_DebugCtrl_Unit                                         8


#define PCCmd_CLEAR_ARM_MASK_BIT                                    0x01
#define PCCmd_CLEAR_SHOULDER_MASK_BIT                               0x04
#define PCCmd_CLEAR_HEAD_MASK_BIT                                   0x08
#define PCCmd_CLEAR_WHEEL_MASK_BIT                                  0x10
#define PCCmd_CLEAR_GRIPPER_MASK_BIT                                0x20

#define Shoulder_Status_CommErr_MaskBit                             0x01




//===============================================================================================//
#define CMD_PC_2_EMB_BUFF_NUMS             16   //<! store cmd pc to emb(shoulder,gripper,wheels and arms) buffer numbers
#define CMD_PC_2_EMB_BUFF_LEN              18   //!< 每条指令的最大长度: code(2) + timStamp(4) + cmdContent(10)

//!< 对于每个Motor的指令缓存
typedef struct{
    u8 dCmd[CMD_PC_2_EMB_BUFF_NUMS][CMD_PC_2_EMB_BUFF_LEN];      //!< 14*16=224 //18*16=288bytes  modified by ling at 0529 
    u8 dCmdWr;
	u8 dCmdRe;
	u8 dCmdCnt;    
    osMutexId* mCmdBuffHandle;
}MotorCMDBuff;


typedef struct{
	u8* pSnd[PRINTF_TX_BUFF_CNT];
	u8* pRev[PRINTF_RX_BUFF_CNT];
	u16 SndCnt[PRINTF_TX_BUFF_CNT];
	u8  bsy[PRINTF_TX_BUFF_CNT];                              //!< bit x,channel x        1:busy,0:idle
	u8 CurBuf;
}TCPSENDCTRL;


typedef enum{
	CMDClearID_Arm = 0x10,
	CMDClearID_Head=0x20,
	CMDClearID_Wheel=0x30,
	CMDClearID_Shoulder=0x40
}CMDClearID_Type;






void sysExtractLegalPktFromTCP(EmbTcpRecvModule* pTcpConn);
s8 sysCmdRecvdFromPCprocess(u8* pMsg,u16 len);
s8 sysGeneralWriteDataToTcpSendBuffer(u8* pdat,u16 len,EmbTcpTransModule* pTrans);
void sysSendEmbInfoToPC(void);
s8 sysGetImuInfo(u8* pdat,u16* plen);
s8 sysGetRDSInfo(u8* pdat,u16* plen);
s8 sysGetMICInfo(u8* pdat,u16* plen);
s8 sysGetShoulderMotorInfo(u8* pdat,u16* plen);


//按照上下位机间的协议，状态上报的函数

s8 sysGetWheelInfo(u8* pdat,u16* plen);
s8 sysGetMotorInfo(u8* pdat,u16* plen);
s8 sysGetGripperInfo(u8* pdat,u16* plen);
s8 sysGetRadarInfo(u8* pdat,u16* plen);
s8 sysGetPSUInfo(u8* pdat,u16* plen);
s8 sysGetPSMInfo(u8* pdat,u16* plen);
void sysPcCommMaintainWork(void);
s8 sysGetEmbSystemSoftwareVersion(u8* pdat,u16* plen);

//按照上下位机间的协议，命令下发的函数
s8 PC_PowerOffCMD(u8 *pUData,u16 unLen);
s8 PC_CmdLedCtrl(u8 *pUData,u16 unLen);
s8 PC_CmdLaserCtrl(u8 *pUData,u16 unLen);
s8 PC_CmdClear(u8 *pUData,u16 unLen);
s8 PC_CmdObstSwitch(u8 *pUData,u16 unLen);
s8 PC_CmdDebugCtrl(u8 *pUData,u16 unLen);
s8 PC_CmdHeartBeat(u8 *pUData,u16 unLen);




#endif

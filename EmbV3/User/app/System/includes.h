/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Includes.h
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author xxx, xxx@ewaybot.com
* @version 0.0.1
* @date 2017-12-20
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2017/12/20 | 0.0.1 | Ling | Create file
*
*/
#ifndef _INCLUDES_H_
#define _INCLUDES_H_


#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include <math.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "bsp.h"
#include "ff.h"
#include "filesys.h"
#include "sys_utils.h"
#include "Emb_MyServo.h"
#include "Emb_Gripper.h"
#include "tcp_server.h"
#include "Emb_PC.h"
#include "Emb_PSM.h"
#include "Emb_Mic.h"
#include "EmbSysProc.h"
#include "Shoulder.h"
#include "Arms.h"
#include "Head.h"
#include "Wheel.h"
#include "Grippers.h"
#include "IMU.h"
#include "BatteryCharger.h"
#include "PSM.h"
#include "Emb_BatteryCharger.h"
#include "Radar.h"
#include "Mic.h"
#include "Led.h"




#define EMB_SYS_VERSION  0x3130U    //!< V1.0


typedef long long int s64;
typedef long long unsigned int u64;



//!< system debug config
#define _EMB_PRINT_DEBUG_        1        //!< 允许下位机使用Bsp_Printf()打印调试信息
#define _EMB_SYS_PRINT_DEBUG     1        //!< 系统指示信息打印

#define Emb_MIC_Debug            1
#define EMB_Upload_SingleCycle_Motor_Info_DEBUG    1
#define EMB_Upload_SingleCycle_Motor_Info_Print_ArmLeft     0
#define EMB_Upload_SingleCycle_Motor_Info_Print_ArmRight    0
#define EMB_Upload_SingleCycle_Motor_Info_Print_Head        0
#define EMB_Upload_SingleCycle_Motor_Info_Print_Shoulder    0
#define EMB_Upload_SingleCycle_Motor_Info_Print_Wheel       0
#define Emb_Set_MotorPID                                    1   //!< 是否使能可设置肩膀电机的PID参数功能
#define Emb_LaserCtrl_Enable                                0   //!< 是否使能可设置激光管脚电平的功能
#define CheckWheelMotorReadyBit_Def                         0   //!< 是否使能读轮子ready寄存器的功能
#define CheckWheelMotorCommBeforeSendCmd                    1   //!< 是否使能在给轮子发送指令前，查询2个轮子是否在线的功能
#define Emb_PC_HeartBeatPack_DEBUG                          1   //!< 是否开启网络心跳包检查的功能
#define Emb_System_TaskMonitor_Enable                       1   //!< 是否使能系统看门狗
#define EMB_Upload_EmbSoftVersion                           1   //!< 是否使能在每次网络连接成功后，上传一包下位机软件版本号的功能

#define Emb_Wheel_Offline_Debug                             1   //!< 轮子模式切换时，会出现offline的情况，目前先放宽轮子掉线的条件

//调试的一级开关
/*
下位机系统打印&记录的总开关，每一条Bsp_printf()&SysLogWrite()的打印和记录，均要检查全局的打印记录开关
*/
#define EMB_SYS_DEBUG                              1        //!< 下位机系统调试开关             //!< 0  xxxx
#define EmbSystemInfo_Debug                         1
#define EMB_COMM_STATUS_DISCONNECTED               0x00
#define EMB_COMM_STATUS_CONNECTED                  0x01
/*
//tComm各个bit的定义
bit0-11        左手、右手
bit12        肩膀
bit13-14    头Yaw,Pitch
bit15-16    轮子左、右
bit17-18    爪子左、右
bit19        上位机
bit20        PSM
bit21        PSU
*/
#define EMB_COMM_ERR_BITS_3_TIMES                  0x07        //< 连续3次通信不正常

#define EMB_COMM_ARM_START_BIT                     0
#define EMB_COMM_SHOULDER_START_BIT                12
#define EMB_COMM_HEAD_START_BIT                    13
#define EMB_COMM_WHEEL_START_BIT                   15
#define EMB_COMM_GRIP_START_BIT                    17
#define EMB_COMM_PC_START_BIT                      19
#define EMB_COMM_PSM_START_BIT                     20
#define EMB_COMM_PSU_START_BIT                     21
#define EMB_COMM_MIC_START_BIT                     22

typedef struct{
    u8 wMotors[SYS_MAX_GENERAL_SERVO_NUMS+Motor_Num_Gripper];        //!< 与各个电机通信状态:头-2、手-12、肩-1、轮子-2、爪子-2、 bitx 0:断开 1:连接
    u8 wPc;                                      //!< 与PC通信状态 0:断开 1:连接
    u8 wPsm;
    u8 wPsu;
    u8 wMic;
}CommSta;

typedef struct{
    u32 mCfgRegCommSta;     //!< 读配置寄存器的通信状态，当收到电机对读配置寄存器的回复包时，置位。手臂(0-11) 肩膀(12) 头(13-14) 轮子(15-16)
    u32 mCfgCompared;       //!< 存放检查各个电机的配置寄存器并比较完毕的结果，当需要重新配置时，需要参考此变量。
    u32 mCfgInitSta;        //!< 对电机配置寄存器的检查状态   0:上电初始，每个周期读电机的cfgRegs并写入到全局中，当本地全局与读回的cfgReg一致，则进入到已初始化后的定期检查阶段 
                                                     //!< 1:定期(4.8s)检查阶段(目前只有肩膀及轮子电机是有此状态位的  bit0-1标识肩膀  bit2-5标识2个轮子电机)
}GenMotorCommSta;

/* 向电机发送读写指令的发送指令控制结构体，由于向电机发送广播读指令涉及到需要相应的等待回复时间，而发送广播写指令并不需要回复。
因此使用此COM口与时间的控制模块来控制读写电机寄存器指令的发送。

COM2    肩膀(1)+头部(2)    10ms
COM3    左臂(6)            
COM4    右臂(6)            
COM5    爪子(2)            
COM6    轮子(2)            

COM口对应的队列指针、发送时间控制模块

*/
#define MOTOR_COM_SEND_INTERVAL_SHOULDER           2    //!< 4ms    肩膀1个电机2ms足矣，但头部2个电机需要4ms的间隔时间
#define MOTOR_COM_SEND_INTERVAL_ARMLEFT            5    //!< 10ms
#define MOTOR_COM_SEND_INTERVAL_ARMRIGHT           5    //!< 10ms
#define MOTOR_COM_SEND_INTERVAL_GRIPPER            3    //!< 6ms
#define MOTOR_COM_SEND_INTERVAL_WHEEL              2    //!< 4ms        modified by ling at 20180527
#define COM_SEND_INTERVAL_PSM_QUERY                30    //!< 60ms
#define COM_SEND_INTERVAL_PSM_CMD                  15    //!< 30ms
#define COM_SEND_INTERVAL_PSU_QUERY                20    //!< 40ms



#define EMB_READ_PSM_PSU_INTERVAL                  16    //!< 16*30ms,每隔480ms读一次PSM,PSU
#define EMB_CHECK_PSU_Capacity_Interval            330   //!< 9.9s,约每隔10s查一次电池电量

#define EMB_READ_MIC_INTERVAL                      30    //!< 30*30ms,每隔900ms读一次MIC
#define EMB_COMMUNICATION_ERR_MIC_INTERVAL         5    //!<  5*30*30ms,每隔900ms读一次MIC
#define EMB_READ_WHEEL_MOTORS_READY_REG_INTERVAL   8    //!< 4*30ms,每隔240ms查询一次轮子电机的0xC0寄存器
#define EMB_READ_MotorsCfgReg_INTERVAL             160    //!< 160*30ms,每隔4.8s检查一次各个电机的配置
#define EMB_SET_GENERAL_MOTOR_CFG_START_REG         EwayMotor_AppMode   //从reg7开始配置
#define EMB_SET_GENERAL_MOTOR_CFG_REG_NUM           3                   //写3个
#define EMB_READ_GENERAL_MOTOR_CFG_REG_NUM          8                   //读8个，区分读PID的6个寄存器
#define EMB_READ_WHEEL_MOTOR_RDY_REG_NUM            1                   //wheel's Ready reg(0xC0) 读1个
#define EMB_READ_WHEELMODE_POS_PID_REG_NUM          6
#define EMB_READ_WHEELMODE_POS_PID_START_REG        0x21                //!< 位置环PID寄存器起始地址
#define EMB_PCHEARTBEAT_TIMEOUT_INTERVAL            400    //!<  400*15ms,每隔6s检查一次心跳包


#define EMB_READ_E2PROM_MAX_TIMES                   5

#define EMB_SYSTEM_TASKS_NUMS                       5
#define EMB_SYSTEM_TASK_NetCommRecv                 0
#define EMB_SYSTEM_TASK_COMSend                     1
#define EMB_SYSTEM_TASK_SysRegular                  2
#define EMB_SYSTEM_TASK_LogProc                     3
#define EMB_SYSTEM_TASK_TaskMonitor                 4

//!< TaskMonitor每隔500ms检查一次各个任务的执行次数，如下数值均以150ms能够到达的次数来计算的
#define IWDG_EXEC_CNT_NetCommRecv           16      //!< 执行16次
#define IWDG_EXEC_CNT_COMSend               62     //!< 需执行250次
#define IWDG_EXEC_CNT_SysRegular            8      //!< 需执行16次
#define IWDG_EXEC_CNT_LogProc               50     //!< 需执行100次
#define IWDG_EXEC_CNT_TaskMonitor           1       //!< 




typedef struct{
    u8 en;        //!< COM口延迟发送功能是否使能,0为禁止，1为使能
    u8 Cnt;        //!< COM口延迟发送计数器，当Cnt为0时，才允许发送；当发送完一条需要电机回复的指令后，重新为Cnt赋值
    const u8 tCnt;    //!< COM口延时发送的时间间隔
}MotorSendTimeCtrl;

typedef struct{
    osThreadId *pComQ;
    MotorSendTimeCtrl tCtr;    //!< com2,3,4,5
    EwayMotor_Device_Type devTy;
}MotorCOMSendModule;

typedef struct{
    u32 Init;
    u32 Cur;
}EmbTimStampModule;

typedef struct{
    u8 En;
    u8 InitCnt[EMB_SYSTEM_TASKS_NUMS];          //!< 存放各个任务执行一段时间后，需到达的周期数。
    u8 CurCnt[EMB_SYSTEM_TASKS_NUMS];
}EmbIWDGModule;

typedef struct{
    CommSta Comm;                           //!< 与外部通信状态
    u32 rtComm;                             //!< real time Communication status 用于存放本周期内与各个设备的通信状态  手臂(0-11) 肩膀(12) 头(13-14) 轮子(15-16) 爪子(17-18) 上位机(19) PSM(20) PSU(21) Mic(22)
    GenMotorCommSta genmCommSta;            //!< 与各个电机的非周期性通信状态，包括对电机配置寄存器参数的周期性检查等。
    MotorCOMSendModule COMSendCtrl[6];
    u32 sysTimCnt;
    EmbTimStampModule tStmp;
    EmbIWDGModule iWdg;
}EwayEmbSysModule;


/*EEProm 存储系统信息相关数据结构及宏定义*/

#define EMB_PCB_VER_LENTH               10
#define EMB_SFT_VER_LENTH               10
#define EMB_PDT_VER_LENTH               10
#define EMB_PSM_VER_LENTH               10
#define EWAY_EMB_MOTOR_NUMS             17      //< 手臂12 + 肩膀1 + 头部2 + 轮子2
#define EWAY_MOTOR_CONFIG_REGS_NUM      0x12


typedef struct{
    u8 mRegs[EWAY_MOTOR_CONFIG_REGS_NUM];
}EwayMotorParameterModule;





typedef struct{
    char PcbVersion[EMB_PCB_VER_LENTH];
    char SoftVersion[EMB_SFT_VER_LENTH];
    char PsmVersion[EMB_PSM_VER_LENTH];
    char ProductVersion[EMB_PDT_VER_LENTH];
    EwayEmbSysInfoRDSModule Rds;
    EwayEmbSysMotorInfoModule mMotor[EWAY_EMB_MOTOR_NUMS];
}EwayEmbSysInfoModule;


#define SysDebugCtrl_EMB_SYS_MASK             0x8000        //!< Bit15
#define SysDebugCtrl_EmbSysInterLogicProc     0x0400        //!< Bit10
#define SysDebugCtrl_Mic_2_Emb_MASK           0x0200        //!< Bit9
#define SysDebugCtrl_Emb_2_Mic_MASK           0x0100        //!< Bit8
#define SysDebugCtrl_Motor_2_Emb_MASK         0x0080        //!< Bit7
#define SysDebugCtrl_Emb_2_PC_MASK            0x0040        //!< Bit6
#define SysDebugCtrl_Psmu_2_Emb_MASK          0x0020        //!< Bit5
#define SysDebugCtrl_Emb_2_Psmu_MASK          0x0010        //!< Bit4
#define SysDebugCtrl_Emb_2_Motor_MASK         0x0008        //!< Bit3
#define SysDebugCtrl_Emb_Proc_PC_Cmd_MASK     0x0004        //!< Bit2
#define SysDebugCtrl_PC_2_Emb_MASK            0x0002        //!< Bit1
#define SysDebugCtrl_EMB_Sys_Err_MASK         0x0001        //!< Bit0

#define JointSwDebugCtrl_Eth_MASK               0x4000      //!< Bit14
#define JointSwDebugCtrl_MIC_MASK               0x2000      //!< Bit13
#define JointSwDebugCtrl_EmbSystem_MASK         0x1000      //!< Bit12
#define JointSwDebugCtrl_ObstacleAvoid_MASK     0x0800      //!< Bit11
#define JointSwDebugCtrl_RopeDisSensor_MASK     0x0400      //!< Bit10
#define JointSwDebugCtrl_UpDnLimit_MASK         0x0200      //!< Bit9
#define JointSwDebugCtrl_IMU_MASK               0x0100      //!< Bit8
#define JointSwDebugCtrl_PSU_MASK               0x0080      //!< Bit7
#define JointSwDebugCtrl_PSM_MASK               0x0040      //!< Bit6
#define JointSwDebugCtrl_Gripper_MASK           0x0020      //!< Bit5
#define JointSwDebugCtrl_Wheel_MASK             0x0010      //!< Bit4
#define JointSwDebugCtrl_Head_MASK              0x0008      //!< Bit3
#define JointSwDebugCtrl_Shoulder_MASK          0x0004      //!< Bit2
#define JointSwDebugCtrl_ArmRight_MASK          0x0002      //!< Bit1
#define JointSwDebugCtrl_ArmLeft_MASK           0x0001      //!< Bit0

#define SecFunDebugCtrl_EmbIn_MotorLstCmd_MASK  0x20000000      //!< Bit29
#define SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK    0x10000000      //!< Bit28
#define SecFunDebugCtrl_SinglWritResp_MASK      0x04000000      //!< Bit26
#define SecFunDebugCtrl_SinglReadResp_MASK      0x02000000      //!< Bit25
#define SecFunDebugCtrl_BroadReadResp_MASK      0x01000000      //!< Bit24
#define SecFunDebugCtrl_NetCommSend_MASK        0x00400000      //!< Bit22
#define SecFunDebugCtrl_SendSem_MASK            0x00200000      //!< Bit21
#define SecFunDebugCtrl_PackXInfo_MASK          0x00100000      //!< Bit20
#define SecFunDebugCtrl_P2E_PSU_MASK            0x00020000      //!< Bit17
#define SecFunDebugCtrl_P2E_PSM_MASK            0x00010000      //!< Bit16
#define SecFunDebugCtrl_E2P_PSU_MASK            0x00002000      //!< Bit13
#define SecFunDebugCtrl_E2P_PSM_MASK            0x00001000      //!< Bit12
#define SecFunDebugCtrl_COM_Send_MASK           0x00000100      //!< Bit8
#define SecFunDebugCtrl_CmdProcInfo_MASK        0x00000040      //!< Bit6
#define SecFunDebugCtrl_PeriodCommStatus_MASK   0x00000020      //!< Bit5
#define SecFunDebugCtrl_XMovementProcess_MASK   0x00000010      //!< Bit4
#define SecFunDebugCtrl_XQueue_2_XCmdBuf_MASK   0x00000002      //!< Bit1
#define SecFunDebugCtrl_TcpRBuf_2_XQueue_MASK   0x00000001      //!< Bit0


/*
sysDebugCtrl;          

Bit15       EMB_SYS_DEBUG                   1
...
Bit10       EmbSystemInterLogicProc         0
Bit9        Mic-->Emb                       0
Bit8        Emb-->Mic                       0
Bit7        Motors-->Emb                    0
Bit6        Emb-->PC                        0
Bit5        PSM&PSU-->EMB                   0
Bit4        EMB-->PSM&PSU                   0
Bit3        Emb-->Motors                    0
Bit2        Emb Process PC Cmd              0
Bit1        Pc-->Emb                        0
Bit0        EMB_SYS_ERR_DEBUG               1
-------------------------------
jointSw

...
bit14       Ethernet                        0
bit13       Mic                             0
bit12       EmbSystem                       0
bit11       ObstacleAvoid                   0
bit10       RDS                             0
bit9        Up-DnLimit                      0
bit8        IMU                             0
bit7        PSU                             0
bit6        PSM                             0
bit5        Gripper                         0
bit4        Wheel                           0
bit3        Head                            0
bit2        Shoulder                        0
bit1        ArmR                            0
bit0        ArmL                            0
-------------------------------
secondFun   
                sysDebugCtrl        
bit0-3              bit1        
    bit0                        Pc-->Emb过程中的Tcp_Recv_Buff--->XQueue                             0
    bit1                        Pc-->Emb过程中的XQueue---->XCmdBuff                                 0

bit4-7              bit2        
    bit4                        Emb Process PC Cmd 过程中的XMovementProcess                         0
    bit5                        Emb Process PC Cmd 过程中的PeriodCommStatus                         0
    bit6                        Emb Process PC Cmd 过程中对于各种指令执行的情况提示 CmdProcInfo     0

bit8-bit11          bit3        
    bit8                        Motors-->Emb过程中的COM Send                        0

bit12-bit15         bit4
    bit12                       EMB-->PSM&PSU过程中的与PSM相关的处理                0
    bit13                       EMB-->PSM&PSU过程中的与PSU相关的处理                0

bit16-19            bit5        
    bit16                       PSM&PSU-->EMB过程中的与PSM相关的处理                0
    bit17                       PSM&PSU-->EMB过程中的与PSU相关的处理                0

bit20-23            bit6        
    bit20                       Emb-->PC过程中的各个包的组包                        0
    bit21                       Emb-->PC过程中的发信号量                            0
    bit22                       Emb-->PC过程中的NetCommSend提示                     0

bit24-27            bit7        
    bit24                       Motors-->Emb的电机的广播读回复                      0
    bit25                       Motors-->Emb的电机对SingleRead的回复
    bit26                       Motors-->Emb的电机对SingleWrite的回复

bit28-31            bit10
    bit28                       EmbSystemInterLogicProc下位机内部逻辑处理中的对肩膀升降电机和是否压上下限位处逻辑的调试打印
    bit29                       EmbSystemInterLogicProc下位机内部逻辑处理中的对电机最后一条指令的记录的功能
    
others:Reserved
*/
typedef struct{
    u16 sysDebugCtrl;           //!< bit15:EMB_SYS_DEBUG  bit0:EMB_SYS_ERR_DEBUG,bit1:Pc->Emb,bit2
    u16 jointSw;                //!< bit0:ArmL,bit1:ArmR,bit2:Shoulder,bit3:Head,bit4:Wheel,bit5:Gripper,bit6:PSM,bit7:PSU,bit8:IMU,bit9:Up-DnLimit,bit10:RDS,bit11:ObstacleAvoid
    u32 secondFun;              //!< 
}EwayEmbSysDebugModule;


#endif

/***************************** END OF FILE *********************************/



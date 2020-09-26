/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Includes.h
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
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
#define _EMB_PRINT_DEBUG_        1        //!< ������λ��ʹ��Bsp_Printf()��ӡ������Ϣ
#define _EMB_SYS_PRINT_DEBUG     1        //!< ϵͳָʾ��Ϣ��ӡ

#define Emb_MIC_Debug            1
#define EMB_Upload_SingleCycle_Motor_Info_DEBUG    1
#define EMB_Upload_SingleCycle_Motor_Info_Print_ArmLeft     0
#define EMB_Upload_SingleCycle_Motor_Info_Print_ArmRight    0
#define EMB_Upload_SingleCycle_Motor_Info_Print_Head        0
#define EMB_Upload_SingleCycle_Motor_Info_Print_Shoulder    0
#define EMB_Upload_SingleCycle_Motor_Info_Print_Wheel       0
#define Emb_Set_MotorPID                                    1   //!< �Ƿ�ʹ�ܿ����ü������PID��������
#define Emb_LaserCtrl_Enable                                0   //!< �Ƿ�ʹ�ܿ����ü���ܽŵ�ƽ�Ĺ���
#define CheckWheelMotorReadyBit_Def                         0   //!< �Ƿ�ʹ�ܶ�����ready�Ĵ����Ĺ���
#define CheckWheelMotorCommBeforeSendCmd                    1   //!< �Ƿ�ʹ���ڸ����ӷ���ָ��ǰ����ѯ2�������Ƿ����ߵĹ���
#define Emb_PC_HeartBeatPack_DEBUG                          1   //!< �Ƿ����������������Ĺ���
#define Emb_System_TaskMonitor_Enable                       1   //!< �Ƿ�ʹ��ϵͳ���Ź�
#define EMB_Upload_EmbSoftVersion                           1   //!< �Ƿ�ʹ����ÿ���������ӳɹ����ϴ�һ����λ������汾�ŵĹ���

#define Emb_Wheel_Offline_Debug                             1   //!< ����ģʽ�л�ʱ�������offline�������Ŀǰ�ȷſ����ӵ��ߵ�����

//���Ե�һ������
/*
��λ��ϵͳ��ӡ&��¼���ܿ��أ�ÿһ��Bsp_printf()&SysLogWrite()�Ĵ�ӡ�ͼ�¼����Ҫ���ȫ�ֵĴ�ӡ��¼����
*/
#define EMB_SYS_DEBUG                              1        //!< ��λ��ϵͳ���Կ���             //!< 0  xxxx
#define EmbSystemInfo_Debug                         1
#define EMB_COMM_STATUS_DISCONNECTED               0x00
#define EMB_COMM_STATUS_CONNECTED                  0x01
/*
//tComm����bit�Ķ���
bit0-11        ���֡�����
bit12        ���
bit13-14    ͷYaw,Pitch
bit15-16    ��������
bit17-18    צ������
bit19        ��λ��
bit20        PSM
bit21        PSU
*/
#define EMB_COMM_ERR_BITS_3_TIMES                  0x07        //< ����3��ͨ�Ų�����

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
    u8 wMotors[SYS_MAX_GENERAL_SERVO_NUMS+Motor_Num_Gripper];        //!< ��������ͨ��״̬:ͷ-2����-12����-1������-2��צ��-2�� bitx 0:�Ͽ� 1:����
    u8 wPc;                                      //!< ��PCͨ��״̬ 0:�Ͽ� 1:����
    u8 wPsm;
    u8 wPsu;
    u8 wMic;
}CommSta;

typedef struct{
    u32 mCfgRegCommSta;     //!< �����üĴ�����ͨ��״̬�����յ�����Զ����üĴ����Ļظ���ʱ����λ���ֱ�(0-11) ���(12) ͷ(13-14) ����(15-16)
    u32 mCfgCompared;       //!< ��ż�������������üĴ������Ƚ���ϵĽ��������Ҫ��������ʱ����Ҫ�ο��˱�����
    u32 mCfgInitSta;        //!< �Ե�����üĴ����ļ��״̬   0:�ϵ��ʼ��ÿ�����ڶ������cfgRegs��д�뵽ȫ���У�������ȫ������ص�cfgRegһ�£�����뵽�ѳ�ʼ����Ķ��ڼ��׶� 
                                                     //!< 1:����(4.8s)���׶�(Ŀǰֻ�м�����ӵ�����д�״̬λ��  bit0-1��ʶ���  bit2-5��ʶ2�����ӵ��)
}GenMotorCommSta;

/* �������Ͷ�дָ��ķ���ָ����ƽṹ�壬�����������͹㲥��ָ���漰����Ҫ��Ӧ�ĵȴ��ظ�ʱ�䣬�����͹㲥дָ�����Ҫ�ظ���
���ʹ�ô�COM����ʱ��Ŀ���ģ�������ƶ�д����Ĵ���ָ��ķ��͡�

COM2    ���(1)+ͷ��(2)    10ms
COM3    ���(6)            
COM4    �ұ�(6)            
COM5    צ��(2)            
COM6    ����(2)            

COM�ڶ�Ӧ�Ķ���ָ�롢����ʱ�����ģ��

*/
#define MOTOR_COM_SEND_INTERVAL_SHOULDER           2    //!< 4ms    ���1�����2ms���ӣ���ͷ��2�������Ҫ4ms�ļ��ʱ��
#define MOTOR_COM_SEND_INTERVAL_ARMLEFT            5    //!< 10ms
#define MOTOR_COM_SEND_INTERVAL_ARMRIGHT           5    //!< 10ms
#define MOTOR_COM_SEND_INTERVAL_GRIPPER            3    //!< 6ms
#define MOTOR_COM_SEND_INTERVAL_WHEEL              2    //!< 4ms        modified by ling at 20180527
#define COM_SEND_INTERVAL_PSM_QUERY                30    //!< 60ms
#define COM_SEND_INTERVAL_PSM_CMD                  15    //!< 30ms
#define COM_SEND_INTERVAL_PSU_QUERY                20    //!< 40ms



#define EMB_READ_PSM_PSU_INTERVAL                  16    //!< 16*30ms,ÿ��480ms��һ��PSM,PSU
#define EMB_CHECK_PSU_Capacity_Interval            330   //!< 9.9s,Լÿ��10s��һ�ε�ص���

#define EMB_READ_MIC_INTERVAL                      30    //!< 30*30ms,ÿ��900ms��һ��MIC
#define EMB_COMMUNICATION_ERR_MIC_INTERVAL         5    //!<  5*30*30ms,ÿ��900ms��һ��MIC
#define EMB_READ_WHEEL_MOTORS_READY_REG_INTERVAL   8    //!< 4*30ms,ÿ��240ms��ѯһ�����ӵ����0xC0�Ĵ���
#define EMB_READ_MotorsCfgReg_INTERVAL             160    //!< 160*30ms,ÿ��4.8s���һ�θ������������
#define EMB_SET_GENERAL_MOTOR_CFG_START_REG         EwayMotor_AppMode   //��reg7��ʼ����
#define EMB_SET_GENERAL_MOTOR_CFG_REG_NUM           3                   //д3��
#define EMB_READ_GENERAL_MOTOR_CFG_REG_NUM          8                   //��8�������ֶ�PID��6���Ĵ���
#define EMB_READ_WHEEL_MOTOR_RDY_REG_NUM            1                   //wheel's Ready reg(0xC0) ��1��
#define EMB_READ_WHEELMODE_POS_PID_REG_NUM          6
#define EMB_READ_WHEELMODE_POS_PID_START_REG        0x21                //!< λ�û�PID�Ĵ�����ʼ��ַ
#define EMB_PCHEARTBEAT_TIMEOUT_INTERVAL            400    //!<  400*15ms,ÿ��6s���һ��������


#define EMB_READ_E2PROM_MAX_TIMES                   5

#define EMB_SYSTEM_TASKS_NUMS                       5
#define EMB_SYSTEM_TASK_NetCommRecv                 0
#define EMB_SYSTEM_TASK_COMSend                     1
#define EMB_SYSTEM_TASK_SysRegular                  2
#define EMB_SYSTEM_TASK_LogProc                     3
#define EMB_SYSTEM_TASK_TaskMonitor                 4

//!< TaskMonitorÿ��500ms���һ�θ��������ִ�д�����������ֵ����150ms�ܹ�����Ĵ����������
#define IWDG_EXEC_CNT_NetCommRecv           16      //!< ִ��16��
#define IWDG_EXEC_CNT_COMSend               62     //!< ��ִ��250��
#define IWDG_EXEC_CNT_SysRegular            8      //!< ��ִ��16��
#define IWDG_EXEC_CNT_LogProc               50     //!< ��ִ��100��
#define IWDG_EXEC_CNT_TaskMonitor           1       //!< 




typedef struct{
    u8 en;        //!< COM���ӳٷ��͹����Ƿ�ʹ��,0Ϊ��ֹ��1Ϊʹ��
    u8 Cnt;        //!< COM���ӳٷ��ͼ���������CntΪ0ʱ���������ͣ���������һ����Ҫ����ظ���ָ�������ΪCnt��ֵ
    const u8 tCnt;    //!< COM����ʱ���͵�ʱ����
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
    u8 InitCnt[EMB_SYSTEM_TASKS_NUMS];          //!< ��Ÿ�������ִ��һ��ʱ����赽�����������
    u8 CurCnt[EMB_SYSTEM_TASKS_NUMS];
}EmbIWDGModule;

typedef struct{
    CommSta Comm;                           //!< ���ⲿͨ��״̬
    u32 rtComm;                             //!< real time Communication status ���ڴ�ű�������������豸��ͨ��״̬  �ֱ�(0-11) ���(12) ͷ(13-14) ����(15-16) צ��(17-18) ��λ��(19) PSM(20) PSU(21) Mic(22)
    GenMotorCommSta genmCommSta;            //!< ���������ķ�������ͨ��״̬�������Ե�����üĴ��������������Լ��ȡ�
    MotorCOMSendModule COMSendCtrl[6];
    u32 sysTimCnt;
    EmbTimStampModule tStmp;
    EmbIWDGModule iWdg;
}EwayEmbSysModule;


/*EEProm �洢ϵͳ��Ϣ������ݽṹ���궨��*/

#define EMB_PCB_VER_LENTH               10
#define EMB_SFT_VER_LENTH               10
#define EMB_PDT_VER_LENTH               10
#define EMB_PSM_VER_LENTH               10
#define EWAY_EMB_MOTOR_NUMS             17      //< �ֱ�12 + ���1 + ͷ��2 + ����2
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
    bit0                        Pc-->Emb�����е�Tcp_Recv_Buff--->XQueue                             0
    bit1                        Pc-->Emb�����е�XQueue---->XCmdBuff                                 0

bit4-7              bit2        
    bit4                        Emb Process PC Cmd �����е�XMovementProcess                         0
    bit5                        Emb Process PC Cmd �����е�PeriodCommStatus                         0
    bit6                        Emb Process PC Cmd �����ж��ڸ���ָ��ִ�е������ʾ CmdProcInfo     0

bit8-bit11          bit3        
    bit8                        Motors-->Emb�����е�COM Send                        0

bit12-bit15         bit4
    bit12                       EMB-->PSM&PSU�����е���PSM��صĴ���                0
    bit13                       EMB-->PSM&PSU�����е���PSU��صĴ���                0

bit16-19            bit5        
    bit16                       PSM&PSU-->EMB�����е���PSM��صĴ���                0
    bit17                       PSM&PSU-->EMB�����е���PSU��صĴ���                0

bit20-23            bit6        
    bit20                       Emb-->PC�����еĸ����������                        0
    bit21                       Emb-->PC�����еķ��ź���                            0
    bit22                       Emb-->PC�����е�NetCommSend��ʾ                     0

bit24-27            bit7        
    bit24                       Motors-->Emb�ĵ���Ĺ㲥���ظ�                      0
    bit25                       Motors-->Emb�ĵ����SingleRead�Ļظ�
    bit26                       Motors-->Emb�ĵ����SingleWrite�Ļظ�

bit28-31            bit10
    bit28                       EmbSystemInterLogicProc��λ���ڲ��߼������еĶԼ������������Ƿ�ѹ������λ���߼��ĵ��Դ�ӡ
    bit29                       EmbSystemInterLogicProc��λ���ڲ��߼������еĶԵ�����һ��ָ��ļ�¼�Ĺ���
    
others:Reserved
*/
typedef struct{
    u16 sysDebugCtrl;           //!< bit15:EMB_SYS_DEBUG  bit0:EMB_SYS_ERR_DEBUG,bit1:Pc->Emb,bit2
    u16 jointSw;                //!< bit0:ArmL,bit1:ArmR,bit2:Shoulder,bit3:Head,bit4:Wheel,bit5:Gripper,bit6:PSM,bit7:PSU,bit8:IMU,bit9:Up-DnLimit,bit10:RDS,bit11:ObstacleAvoid
    u32 secondFun;              //!< 
}EwayEmbSysDebugModule;


#endif

/***************************** END OF FILE *********************************/



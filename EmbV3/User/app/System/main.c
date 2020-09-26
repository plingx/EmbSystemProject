/**
*********************************************************************************************************
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file main.c
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
* @author xxx, xxx@ewaybot.com
* @version 0.0.1
* @date 2000-01-01
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2000/01/01 | 0.0.1 | xxxx | Create file
*
* 2008/02/13 | 0.0.2 | xxxx | Add .....
**********************************************************************************************************
*/

#include "includes.h"
#include "arm_math.h"
#include "malloc.h" 
#include "lwip_comm.h"
#include "tcp_server.h" 
#include "tcp.h"



//------------------��λ���汾��˵��-----------------------|
/*
        �豸����(2Bytes)                                   |          ����汾(5Bytes)
----------------------------------------------------------------------------------------------------------------
    ������(1B)  |       �η���(1B)                         |     ����(4B)                  |    С�汾(1B)
-------------------------------------------------------------------------------------------
                |    ������1(4-bit)  |  ������2(4-bit)     |     ��(2B)   ��(1B)   ��(1B)  |       
----------------------------------------------------------------------------------------------------------------
    1:Emb               1:Moro1                            |     
                        -----------------------------------|
                                                           |
                        2:Moro2     1:�ɿ�����+������      |
                                    -----------------------|
                                    2:�ɿ�����+������      |
                                    -----------------------|
                                    3:�¿�����+������      |
                        -----------------------------------|
                                                           |
                        3:MoroB     4:�¿����� + С����    |
                                                           |
                        -----------------------------------|
                                                           |
                        4:MoroL                            |
                                                           |
-----------------------------------------------------------|
                                                           |
    2:PSM               1:Moro1                            |
                        -----------------------------------|
                                                           |
                        2:Moro2                            |
                        -----------------------------------|
                                                           |
                        3:MoroB                            |
                        -----------------------------------|
                                                           |
                        4:MoroL                            |
-----------------------------------------------------------|

    3:Motor

*/

#define EMB_System_Primary_Type        0x01            //!< 1:Emb, 2:PSM,  3:Motor
#define EMB_System_Secondary_Type1     0x20            //!< 0x10:Moro1,   0x20:Moro2,    0x30:MoroB     0x40:MoroL
#define EMB_System_Secondary_Type2     0x03            //!< 0x01:�ɿ�����+������       0x02:�ɿ�����+������     0x03:�¿�����+������
#define EMB_System_SoftVer_Year        2019            //!< 
#define EMB_System_SoftVer_Month       2              //!< 
#define EMB_System_SoftVer_Day         26              //!< 
#define EMB_System_SoftVer_Tail        0               //!< 2018.11.23����ĵ�1���汾

//--------------------------�ָ���-------------------------|




extern u8 ArmSvdelayTm[6];
extern EwayShoulderModule EwayShoulder;
extern __IO uint32_t MySysTick;

extern EwayIMUModule EwayImu;
extern EwayEmbTCPModule EwayEmbTCP;
extern EwayEmbFileSysModule EwayEmbFileSys;
extern EwayPSMModule EwayPSM;
extern EwayArmModule EwayArms;
extern EwayShoulderModule EwayShoulder;
extern EwayHeadModule EwayHeads;
extern EwayGripperModule EwayGrippers;
extern EwayWheelModule EwayWheels;
extern EwayBatteryChargerModule EwayBatteryCharger;
extern EwayMicModule EwayMic;
extern EwayRadarObstAvoidModule EwayRadarObstAvoid;
extern u8 RadarDist[];
extern u8 RadarRecvFlag;
extern MotorSTATEBuff ArmsReg[];
extern MotorSTATEBuff ShoulderReg;
extern MotorSTATEBuff HeadsReg[];
extern MotorSTATEBuff WheelsReg[];


osThreadId TaskNetCommSendHandle;
osThreadId TaskNetCommRecvHandle;
osThreadId TaskSysRegularHandle;
osThreadId TaskMainHandle;
osThreadId TaskLogProcHandle;
osThreadId TaskCOMSendHandle;

osSemaphoreId NetSend_BinarySemHandle;

QueueHandle_t EmbToLogWrQueHdl;       //!< store Cmd Emb Write to TF Card Info

QueueHandle_t PCToShoulderQueHdl;     //!< store Cmd Pc send to Emb Shoulder Cmd
QueueHandle_t ShoulderToMotorQueHdl;  //!< store Cmd Emb send to Shoulder Motor 

QueueHandle_t PCToArmsQueHdl;         //!< store Cmd Pc send to Emb Arms Cmd
QueueHandle_t ArmsLeftToMotorQueHdl;  //!< store Cmd Emb send to Left Arms Motor 
QueueHandle_t ArmsRightToMotorQueHdl; //!< store Cmd Emb send to Right Arms Motor 

QueueHandle_t PCToHeadsQueHdl;        //!< store Cmd Pc send to Emb Heads Cmd

QueueHandle_t PCToWheelsQueHdl;       //!< store Cmd Pc send to Emb Wheels Cmd
QueueHandle_t WheelsToMotorQueHdl;    //!< store Cmd Emb send to Wheels Motor 

QueueHandle_t PCToGrippersQueHdl;     //!< store Cmd Pc send to Emb Grippers Cmd
QueueHandle_t GrippersToMotorQueHdl;  //!< store Cmd Emb send to Grippers Motor 

QueueHandle_t PCToEmbSysQueHdl;       //!< store Cmd Pc send to Emb System Cmd
QueueHandle_t EmbSysToMotorQueHdl;    //!< store Cmd Emb send to Emb System Motor 

QueueHandle_t EmbSysToPSMPSUHdl;        //!< stroe Cmd Emb System send to PSM&PSU(COM7)

osMutexId PCCmdBuffOpHandleAml[Motor_Num_ArmLeft];
osMutexId PCCmdBuffOpHandleAmr[Motor_Num_ArmRight];
osMutexId PCCmdBuffOpHandleShd;
osMutexId PCCmdBuffOpHandleHed[Motor_Num_Head];
osMutexId PCCmdBuffOpHandleWhl[Motor_Num_Wheel];
osMutexId PCCmdBuffOpHandleGrp[Motor_Num_Gripper];

osMutexId NetCommSendOpHandle;

void StartTaskLogProc(void const * argument);
void StartTaskCOMSend(void const * argument);
void StartTaskNetCommRecv(void const * argument);
void StartTaskNetCommSend(void const * argument);
void StartTaskSysRegular(void const * argument);
void StartTaskMonitor(void const * argument);




__IO u32 SysTimeStamp=0;
EwayEmbSysModule EwayEmbSys={{{0},0,0,0,0},
                              0x00000000,
                             {0x00000000,0x00000000},
            /*COM2*/         {{&ShoulderToMotorQueHdl,{1,0,MOTOR_COM_SEND_INTERVAL_SHOULDER},EWAYBOT_SHOULDER},\
            /*COM3*/          {&ArmsLeftToMotorQueHdl,{1,0,MOTOR_COM_SEND_INTERVAL_ARMLEFT},EWAYBOT_ARMS_LEFT},\
            /*COM4*/          {&ArmsRightToMotorQueHdl,{1,0,MOTOR_COM_SEND_INTERVAL_ARMRIGHT},EWAYBOT_ARMS_RIGHT},\
            /*COM5*/          {&GrippersToMotorQueHdl,{1,0,MOTOR_COM_SEND_INTERVAL_GRIPPER},EWAYBOT_GRIPPERS},\
            /*COM6*/          {&WheelsToMotorQueHdl,{1,0,MOTOR_COM_SEND_INTERVAL_WHEEL},EWAYBOT_WHEEL},
            /*COM7*/          {&EmbSysToPSMPSUHdl,{1,0,COM_SEND_INTERVAL_PSM_QUERY},EWAYBOT_PSM}
                             },
                             0,
                             {0x0B081100,0x00000000},        //< Log???? 0x031A110A=2018.08.23.17:00
                             {0,{IWDG_EXEC_CNT_NetCommRecv,IWDG_EXEC_CNT_COMSend,IWDG_EXEC_CNT_SysRegular,IWDG_EXEC_CNT_LogProc,IWDG_EXEC_CNT_TaskMonitor},{0}}
                           };


EwayRDSModule EwayRDS={0,0};   //!< Rope Displacement Sensor
EwayEmbSysInfoModule EwayEmbSysInfo;
EwayEmbSysDebugModule EwayEmbSysDebug={0x8001,0,0};
EwayEmbSysDebugModule* pDebug = &EwayEmbSysDebug;
u8 testCnt[10]={0};


extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];
extern u8 MotorCfgReg[SYS_MAX_GENERAL_SERVO_NUMS][GENERAL_SERVO_CFG_REGS];
s8 PrintfUart1DmaTransferConfig(u8 *pSndBuf,u16 len);
void sysSetEmbSystemInfotoDefault(EwayEmbSysInfoModule* pModule);




//������K1��ʱ����ʾ��ǰ��λ�����ж������񣬸�����ʲô״̬
//��չһ�£��ڵ��Դ����£���λ��Ӧ��ӵ������ģʽ��debugģʽ������ĳЩ������debugģʽ�������������п�����λ�������ӵĸ��ֵ��
//������show����ܿ������������״̬
//��set֮�������ܿ��Ƹ��������


//!< COM����ͬʱ������ָ��ʹ�ò�ͬ����Ϣ���У�����粿��ͷ��
//!< , �������ڶ��Ĵ���ʱ��ʹ��ĳһ����Ϣ���оͺã�����粿��ͷ���Ķ�����Ĵ���״̬��ָ��Ϳ��Է���ͬһ�����з���
s8 sysGetMotorSendQueue(EwayMotor_Device_Type devType,QueueHandle_t* pQue)
{
    s8 res=ERR_NONE;
    
    switch(devType)
    {
        case EWAYBOT_ARMS_LEFT:
                *pQue = ArmsLeftToMotorQueHdl;            
            break;
        case EWAYBOT_ARMS_RIGHT:
                *pQue = ArmsRightToMotorQueHdl;
            break;
        case EWAYBOT_SHOULDER:
                *pQue = ShoulderToMotorQueHdl;
            break;
        case EWAYBOT_HEAD:
                *pQue = ShoulderToMotorQueHdl;//HeadsToMotorQueHdl;
            break;
        case EWAYBOT_GRIPPERS:
                *pQue = GrippersToMotorQueHdl;
            break;
        case EWAYBOT_WHEEL:
                *pQue = WheelsToMotorQueHdl;
            break;
        case EWAYBOT_PSM:
                *pQue = 0;
            break;
        case EWAYBOT_PSU:
                *pQue = 0;
            break;
        default:
                res = ERR_INPUT_PARAMETERS;
            break;
    }    
    return res;
}


s8 sysGetMotorSendRecordPointer(EwayMotor_Device_Type devType,MotorRecordBuff** pMRec,u8* max_iD,u8* startId)
{
    s8 res=ERR_NONE;
        
    switch(devType)
    {
        case EWAYBOT_ARMS_LEFT:
        case EWAYBOT_ARMS_RIGHT:
                    *pMRec = EwayArms.mState.pMotorRecord;
                    *max_iD = Motor_Num_Arm;
                    *startId = Emb_StartID_ArmL;
            break;
        case EWAYBOT_SHOULDER:
                    *pMRec = EwayShoulder.mState.pMotorRecord;
                    *max_iD = Motor_Num_Shoulder;                    
                    *startId = Emb_StartID_Shoulder;
            break;
        case EWAYBOT_HEAD:
                    *pMRec = EwayHeads.mState.pMotorRecord;                    
                    *max_iD = Motor_Num_Head;                  
                    *startId = Emb_StartID_Head;
            break;
        case EWAYBOT_GRIPPERS:
                    *pMRec = EwayGrippers.mState.pMotorRecord;                  
                    *max_iD = Motor_Num_Gripper;                
                    *startId = Emb_StartID_Gripper;
            break;
        case EWAYBOT_WHEEL:
                    *pMRec = EwayWheels.mState.pMotorRecord;                 
                    *max_iD = Motor_Num_Wheel;                
                    *startId = Emb_StartID_Wheel;
            break;
        case EWAYBOT_PSM:
        case EWAYBOT_PSU:
        default:
                res = ERR_INPUT_PARAMETERS;
            break;
    }
    
    return res;
}



void sysEmbCheckLeds(void)
{
    if(((EwayEmbSys.Comm.wPc&0x01) != 0x00)&&((EwayEmbSys.sysTimCnt%8) == 0))  //���������ӵģ���240ms�ѵ�����תLED
    {
        LedsStatus_Toggle(LED1);        //!< Blue LED1
    }

    if((EwayEmbSys.sysTimCnt%32) == 0)
    {
        LedsStatus_Toggle(LED0);        //!< White LED0
    }

}

/* --------------------------------------------------------------------------*/
/**
* @name sysSetEmbSystemInfotoDefault
* @brief ����λ��ϵͳ��Ϣ����ΪĬ��ֵ
* @details ���ڶ����ݴ����ԭ�򣬴�EEProm���ص����ݲ����ţ��������Ϊ��ȫ��Ĭ��ֵ
*          ��֤ϵͳ�ɼ����ȶ�����
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void sysSetEmbSystemInfotoDefault(EwayEmbSysInfoModule* pModule)
{
    //!< ����ֵ�Ȳ�д��ֻ���������ߴ������йص�

    pModule->Rds.En = 0;
    pModule->Rds.init = 1;
    pModule->Rds.HigLmt = 0;
    pModule->Rds.LowLmt = 0;
    pModule->Rds.Lmt_SwitchLen = LIMIT_SWITCH_LENGTH_SHORT;
    
}



s8 ReadInfoFromE2prom(void)
{
    s8 res;
    u16 i,len,sum;
    u8 dat[512]={0};
    
    res = SysE2PromRead(SystemInfo_EpromStartAddr,dat,(sizeof(EwayEmbSysInfo)+2));

    if(res!=ERR_NONE)
    {
        memset((u8*)(&EwayEmbSysInfo),0,sizeof(EwayEmbSysInfo));
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            SysLogWrite(LogNormal,"SysE2PromRead() err.rt:%d.",res);

            Bsp_printf("SysE2PromRead() err.rt:%d.",res);
        }

        return res;
    }

    sum = 0;
    len = sizeof(EwayEmbSysInfo);

    for(i=0;i<len;i++)
    {
        sum += dat[i];
    }

    if(sum != (dat[len]+(dat[len+1]<<8)))
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            Bsp_printf("SysE2PromRead() check sum err.");
        }
        memset((u8*)(&EwayEmbSysInfo),0,sizeof(EwayEmbSysInfo));
        return ERR_CHECK_SUM;
    }

    memcpy((u8*)(&EwayEmbSysInfo),dat,len);

    //!< ������λ���豸��Ϣ���汾��Ϣ
    EwayEmbSysInfo.PsmVersion[0] = 0x02;        //!< ���Ӧ��ʹ�õ�PSM�汾:Moro1,Moro2,MoroB,MoroL
    
    EwayEmbSysInfo.SoftVersion[0] = EMB_System_Primary_Type;
    EwayEmbSysInfo.SoftVersion[1] = EMB_System_Secondary_Type1|EMB_System_Secondary_Type2;
    EwayEmbSysInfo.SoftVersion[2] = (u8)(EMB_System_SoftVer_Year);          //!< LowByte
    EwayEmbSysInfo.SoftVersion[3] = (u8)(EMB_System_SoftVer_Year>>8);       //!< HighByte
    EwayEmbSysInfo.SoftVersion[4] = EMB_System_SoftVer_Month;
    EwayEmbSysInfo.SoftVersion[5] = EMB_System_SoftVer_Day;
    EwayEmbSysInfo.SoftVersion[6] = EMB_System_SoftVer_Tail;
    
    //!< ��copy���������������ȫ�ֱ���    
    EmbConfigMotorsParametersForArms(Emb_StartID_ArmL,Motor_Num_Arm,&EwayEmbSysInfo.mMotor[0]);                 //!< mMotor[0-11]

    EmbConfigMotorsParametersForShoulder(Emb_StartID_Shoulder,Motor_Num_Shoulder,&EwayEmbSysInfo.mMotor[12]);   //!< mMotor[12]

    EmbConfigMotorsParametersForHeads(Emb_StartID_Head,Motor_Num_Head,&EwayEmbSysInfo.mMotor[13]);              //!< mMotor[13-14]

    EmbConfigMotorsParametersForWheels(Emb_StartID_Wheel,Motor_Num_Wheel,&EwayEmbSysInfo.mMotor[15]);           //!< mMotor[15-16]

    //!< ���init��־λ�Ƿ�Ϊ�Ѿ���ʼ��
    if(EwayEmbSysInfo.Rds.init != 1)
    {

    }

    if(EwayEmbSysInfo.Rds.Lmt_SwitchLen==0)
    {
        //memset((u8*)(&EwayEmbSysInfo),0,sizeof(EwayEmbSysInfo));

        return ERR_CHECK_DATA_INVALID;
    }

    if(EwayEmbSysInfo.Rds.En!=0)
    {        
        //!< ���궨������λ������λ��RDS��ֵ�����ο��߶�ֵ    
        if(EwayEmbSysInfo.Rds.LowLmt>=EwayEmbSysInfo.Rds.HigLmt)    //!< ���������Ч��
        {
            //memset((u8*)(&EwayEmbSysInfo),0,sizeof(EwayEmbSysInfo));
        
            return ERR_CHECK_DATA_INVALID;
        }
    }

    //Bsp_printf("SysE2PromRead() successful.Rds.En:%d,Rds.init:%d,Rds.HigLmt:%d,Rds.LowLmt:%d.",EwayEmbSysInfo.Rds.En,EwayEmbSysInfo.Rds.init,EwayEmbSysInfo.Rds.HigLmt,EwayEmbSysInfo.Rds.LowLmt);

    
    return ERR_NONE;
}



void sysMaintainWorkPeriodically(u32 cyc)
{
    //!< ���ڼ��i2cͨ��״̬������ȡIMU data����
    sysIMUCommMaintain(&EwayImu);
    
    sysGetImuData(&EwayImu);
    
    //!< ���ڻ�ȡ������λ�ܽ�״̬������
    sysGetLimitSwitchStatus(&EwayShoulder);

    //!< ���ڻ�ȡ���ߴ�������ֵ
    sysGetRopeDisplacementSensorData(&EwayRDS);

    //!< ���ڼ�������Ƿ���Ҫ�궨
    //sysEmbCheckShoulderCalibrateProc();

    //!< �������ģʽ�Ƿ�������ģʽһ��
    sysCyclicalCheckEmbMotorsCfgRegs();       
    
    //!< ����״̬��أ���ִ����Ӧ�Ĳ���
    sysPcCommMaintainWork();

    //!< �����ư�״̬��ؼ���Ӧ�Ĳ���
    sysBatteryChargerMaintainWork();

    //!< ��������ϱ���PC�ĵ������·��������ָ�����ظ���״̬
    sysEmbMotorSingleCycleCmdStatusClear();    
	
	//!< ���ڼ����Ϲ����Ƿ�򿪣��Ƿ���Ҫ����
    sysEmbCheckRadarAndObstacleAvoid();
		
	//!< ���ڼ��Mic����״̬
    sysEmbMicMaintainWork(&EwayMic);
        
    //!< ���ڼ������ͨ��״̬������ͨ���жϻ��߻ָ�ʱ��ִ����Ӧ�Ĳ���
    //sysEmbMotorsCommSupervise();

    //!< ���ڼ��Wheel��ͨ��״̬����Wheel�ָ�ͨ�ź���Ҫ��һЩ����
#if CheckWheelMotorReadyBit_Def
    sysEmbCheckWheelMotorsStatus();   //20180711
#endif

#if CheckWheelMotorCommBeforeSendCmd
    sysEmbCheckWheelMotorsCommuStatus();
#endif
    //!< ���ڼ��ϵͳָʾ��
    sysEmbCheckLeds();
}


/* --------------------------------------------------------------------------*/
/**
* @name sysReadMotorsStatus
* @brief 
* @details ϵͳ���֡��硢ͷ��צ�����ӵȵ�����ͻ�ȡ״̬�Ĵ��������ݰ�
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void sysReadMotorsStatus(void)
{
    u8 i;
    s8 res;
    
      //////??????????
    EwayArms.mCommStatus = 0;
    
    EwayShoulder.mCommStatus = 0;
    
    EwayHeads.mCommStatus = 0;
    
    EwayGrippers.mCommStatus = 0;
    
    EwayWheels.mCommStatus = 0;
    
    memset((u8*)(&sysEwServo[15]),0,sizeof(EWAYBOT_SERVO_STATUS));

    memset((u8*)(&sysEwServo[16]),0,sizeof(EWAYBOT_SERVO_STATUS));
    
    res = sysGeneralMotorBroadcastRead(Emb_StartID_ArmL,Motor_Num_ArmLeft,SERVO_REG_READ_START_ADDR,SERVO_REG_READ_NUM_ARM,ArmSvdelayTm,EWAYBOT_ARMS_LEFT);
    if( ERR_NONE != res )
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            res = Bsp_printf("3-2 Send sysGeneralMotorBroadcastRead() to ArmL failed,rt:%d.",res);
        }

    }
    
    res = sysGeneralMotorBroadcastRead(Emb_StartID_ArmR,Motor_Num_ArmRight,SERVO_REG_READ_START_ADDR,SERVO_REG_READ_NUM_ARM,ArmSvdelayTm,EWAYBOT_ARMS_RIGHT);
    if( ERR_NONE != res )
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            res = Bsp_printf("3-2 Send sysGeneralMotorBroadcastRead() to ArmR failed,rt:%d.",res);
        }
    }
    
    res = sysGeneralMotorBroadcastRead(Emb_StartID_Shoulder,Motor_Num_Shoulder,SERVO_REG_READ_START_ADDR,SERVO_REG_READ_NUM_HEAD_SHOULDER,ArmSvdelayTm,EWAYBOT_SHOULDER);
    if( ERR_NONE != res )
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            res = Bsp_printf("3-2 Send sysGeneralMotorBroadcastRead() to Shoulder failed,rt:%d.",res);
        }
    }
    
    res = sysGeneralMotorBroadcastRead(Emb_StartID_Head,Motor_Num_Head,SERVO_REG_READ_START_ADDR,SERVO_REG_READ_NUM_HEAD_SHOULDER,ArmSvdelayTm,EWAYBOT_HEAD);
    if( ERR_NONE != res )
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            res = Bsp_printf("3-2 Send sysGeneralMotorBroadcastRead() to Head failed,rt:%d.",res);
        }
    }

    for(i=Emb_StartID_Gripper;i<(Emb_StartID_Gripper+Motor_Num_Gripper);i++)
    {
        res = sysSendDynamixelReadDataPacket(EWAYBOT_GRIPPERS,i,DYNAMIXEL_REG_CurPosL,8);     //!<COM5 Gripper
        if( ERR_NONE != res )
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("Gripper:Send Query pkt to Gripper failed,rt:%d.id:%d.",res,i);
            }
        }
        else
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&(pDebug->jointSw&JointSwDebugCtrl_Gripper_MASK))
            {
                res = Bsp_printf("Gripper:Send Query pkt to (%d) is succ.",i);
            }

        }
    }
            
    res = sysGeneralMotorBroadcastRead(Emb_StartID_Wheel,Motor_Num_Wheel,SERVO_REG_READ_START_ADDR,SERVO_REG_READ_NUM_WHEEL,ArmSvdelayTm,EWAYBOT_WHEEL);
    if( ERR_NONE != res )
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            res = Bsp_printf("3-2 Send sysGeneralMotorBroadcastRead() to Wheel failed,rt:%d.",res);
        }
    }

}


void sysReadPsmPsuStatus(void)
{
    s8 res;

    //!< Read PSM Status Reg 480ms    
    if((EwayEmbSys.sysTimCnt%EMB_READ_PSM_PSU_INTERVAL)==0)
    {
        EwayPSM.mCommStatus = 0;    //!< �������״̬�Ĵ�����־
        
        res = SendQueryPSMStatus();
        
        if(ERR_NONE != res)
        {
            //SysLogWrite(LogMajor,"read Psm-1:SendQueryPSMStatus() failed.rt:0x%x.",res);

            //res = Bsp_printf("read Psm-1:SendQueryPSMStatus() failed.rt:0x%x.",res);
        }
        else
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_2_Psmu_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_E2P_PSM_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_PSM_MASK))
            {
                res = Bsp_printf("read Psm-1:SendQueryPSMStatus() Success.");        
            }
        }

        EwayBatteryCharger.mCommStatus = 0;    //!< �������״̬�Ĵ�����־
        
        res = SendQueryBatteryChargerStatus();
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                SysLogWrite(LogMajor,"read Psu-1:SendQueryBatteryChargerStatus() failed.rt:0x%x.",res);

                res = Bsp_printf("read Psu-1:SendQueryBatteryChargerStatus() failed.rt:0x%x.",res);
            }
        }
        else
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_2_Psmu_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_E2P_PSU_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_PSU_MASK))
            {
                res = Bsp_printf("read Psu-1:SendQueryBatteryChargerStatus() Success.");        
            }
        }        
    }    
}

void sysReadMicStatus(void)
{
    s8 res;

    if((EwayEmbSys.sysTimCnt%EMB_READ_MIC_INTERVAL)==0)
    {
        EwayMic.mCommStatus = 0x00;
        
        switch(EwayMic.runState)
        {
            case EwayMicRunState_UnInitial:
                 //!< Send PING packet
                res = Mic_Ping(MIC_Start_ID);
                     
                if(res != ERR_NONE)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        SysLogWrite(LogNormal,"Mic-1-1 Send Mic_Ping() failed,rt:%d",res);
                            
                        Bsp_printf("Mic-1-1 Send Mic_Ping() failed,rt:%d",res);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                    {
                        //Bsp_printf("Mic-1-1 Send Mic_Ping() Suc!");
                    }
                }
                   
                break;
        
            case EwayMicRunState_OnLine_SendReset:
        
                //!< reset Mic board
                res = sysEmbResetMicBoard(MIC_Start_ID);

                if(res != ERR_NONE)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        SysLogWrite(LogNormal,"Mic-1-2 Send Reset_Mic pkt failed,rt:%d",res);
                            
                        Bsp_printf("Mic-1-2 Send Reset_Mic pkt failed,rt:%d",res);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                    {
                        //Bsp_printf("Mic-1-2 Send Reset_Mic pkt Suc!");
                    }
                }
                    
                break;
        
            case EwayMicRunState_OnLine_SendWake:
        
                //!< send Mic wake
                res = sysEmbEnableMicWakeup(MIC_Start_ID);

                if(res != ERR_NONE)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        SysLogWrite(LogNormal,"Mic-1-3 Send Set_Wakeup_Mic pkt failed,rt:%d",res);
                            
                        Bsp_printf("Mic-1-3 Send Set_Wakeup_Mic pkt failed,rt:%d",res);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                    {
                        //Bsp_printf("Mic-1-3 Send Set_Wakeup_Mic pkt Suc!");
                    }
                }
                    
                break;
        
            case EwayMicRunState_Initialized_RdVersi:
        
                //!< Read Mic Version
                res = sysEmbReadMicInfo(MIC_Start_ID);
        
                if(res != ERR_NONE)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        SysLogWrite(LogNormal,"Mic-1-4 Send Read_Mic_Version pkt failed,rt:%d",res);
                            
                        Bsp_printf("Mic-1-4 Send Read_Mic_Version pkt failed,rt:%d",res);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                    {
                        //Bsp_printf("Mic-1-4 Send Read_Mic_Version pkt Suc!");
                    }
                }
                    
                break;
        
            case EwayMicRunState_Initialized_RdAngle:
        
                 //!< Read Mic Angle 
                res = sysEmbReadMicWakeupAngle(MIC_Start_ID);
        
                if(res != ERR_NONE)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        SysLogWrite(LogNormal,"Mic-1-5 Send Read_Mic_Wakeup_Angle Pkt failed,rt:%d",res);
                            
                        Bsp_printf("Mic-1-5 Send Read_Mic_Wakeup_Angle Pkt failed,rt:%d",res);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                    {
                        //Bsp_printf("Mic-1-5 Send Read_Mic_Wakeup_Angle Pkt Suc!");
                    }
                }
                    
                break;

            case EwayMicRunState_Run_SendReset:           //!< ???Mic?????????

                //!< reset Mic board
                res = sysEmbResetMicBoard(MIC_Start_ID);

                if(res != ERR_NONE)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        SysLogWrite(LogNormal,"Mic-1-6 RunState Send Reset_Mic pkt failed,rt:%d",res);
                            
                        Bsp_printf("Mic-1-6 RunState Send Reset_Mic pkt failed,rt:%d",res);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                    {
                        //Bsp_printf("Mic-1-6 RunState Send Reset_Mic pkt Suc!");
                    }
                }
            
                break;
        
            default:
                    
                break;
        }
    }  

}
/* --------------------------------------------------------------------------*/
/**
* @name sysProcessRecvDataFromMotors
* @brief 
* @details ϵͳ�����֡��硢ͷ��צ�����ӵȵ��������������
*
* @param[in] None
*
* @returns ERR_None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void sysProcessRecvDataFromMotors(void)
{
    u8 i;
    s8 res;
    
    //!< ����Ƿ��е���ϱ�״̬:ArmL&R Shoulder Head
    for(i=EWAYBOT_ARMS_LEFT;i<=EWAYBOT_HEAD;i++)
    {
        res = sysGeneralMotorRecvDataProcess((EwayMotor_Device_Type)i);
            
        if( ERR_NONE != res )
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("2-1 sysGeneralMotorRecvDataProcess(%d) pkt failed,rt:%d.",i,res);
            }
        }
    }

    //!< ����Ƿ��յ�צ�ӵ�����
    res = sysProcessDynamixelRecvdDataPacket(EWAYBOT_GRIPPERS);
    if( ERR_NONE != res )
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            res = Bsp_printf("2-1 Recv sysProcessDynamixelRecvdDataPacket() pkt failed,rt:%d",res);
        }
    }
    
    //< ������ӵ���Ľ��ջ���
    res = sysGeneralMotorRecvDataProcess(EWAYBOT_WHEEL);            
    if( ERR_NONE != res )
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            res = Bsp_printf("2-1 Recv sysGeneralMotorRecvDataProcess() pkt failed,rt:%d.",res);
        }
    }

}



/* --------------------------------------------------------------------------*/
/**
* @name sysPcCmdProcess
* @brief 
* @details ϵͳ������λ�������֡��硢ͷ��צ���ֵ�ָ�����
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void sysPcCmdProcess(void)
{    
    //!< ����Ƿ�����λ���·���left arm��ָ��
    sysPcToEmbCmdProcess(EWAYBOT_ARMS_LEFT,&PCToArmsQueHdl);     //!< ������ʹ��ͬһ������
    
    //!< ����Ƿ�����λ���·���shoulder��ָ��
    sysPcToEmbCmdProcess(EWAYBOT_SHOULDER,&PCToShoulderQueHdl);
    
    //!< ����Ƿ�����λ���·���head��ָ��
    sysPcToEmbCmdProcess(EWAYBOT_HEAD,&PCToHeadsQueHdl);

    //!< ����Ƿ�����λ���·��� gripper ��ָ��
    sysPcToEmbCmdProcess(EWAYBOT_GRIPPERS,&PCToGrippersQueHdl);    

    //!< ����Ƿ�����λ���·��� wheel ��ָ��
    sysPcToEmbCmdProcess(EWAYBOT_WHEEL,&PCToWheelsQueHdl);

    //!< ����Ƿ�����λ���·�����λ����ָ��
    sysPcToEmbCmdProcess(EWAYBOT_EMB,&PCToEmbSysQueHdl);
}

/* --------------------------------------------------------------------------*/
/**
* @name sysCmdToMotorProcess
* @brief 
* @details ϵͳ������λ�������֡��硢ͷ��צ���ֵ�ָ�����
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void sysCmdToMotorProcess(void)
{    
    sysEmbToMotorCmdProcess(EWAYBOT_ARMS_LEFT); 

    sysEmbToMotorCmdProcess(EWAYBOT_ARMS_RIGHT);
    
    sysEmbToMotorCmdProcess(EWAYBOT_SHOULDER);
    
    sysEmbToMotorCmdProcess(EWAYBOT_HEAD);

    sysEmbToMotorCmdProcess(EWAYBOT_GRIPPERS);

    sysEmbToMotorCmdProcess(EWAYBOT_WHEEL);
}




void sysPrintErrorStatus(void)
{
    u8 i;
    u16 st;
    //!< ��Դ״̬����� fCode=00 00
    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
    (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_PSU_MASK))
    {
        if((EwayEmbSys.Comm.wPsu&EMB_COMM_ERR_BITS_3_TIMES)==0)     //!< ��PSUͨ�Ų�����
        {
            Bsp_printf("PSU offline,wPsu:0x%x.",EwayEmbSys.Comm.wPsu);
        }
        else
        {
            //!< ���PSU�Ĺ���״̬λ�����й������ӡ����
            if((EwayBatteryCharger.sBattCharger.unStatus&0x00FF)!=0)
            {
                Bsp_printf("BattCharger Status:0x%x.",EwayBatteryCharger.sBattCharger.unStatus);
            }        
        }
    }

    //!< ��Դ���ذ�״̬�������ڴ�ӡ��Ϣ���޹���״̬��ʾ


    //!< IMUͨ��״̬
    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
    (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_IMU_MASK))
    {
        if(EwayImu.Comm.sta!=0)
        {
            Bsp_printf("EwayIMU Comm Sta:0x%x.",EwayImu.Comm.sta);
        }
    }

    //!< TOF���ݣ������ڴ�ӡ���޹���״̬��ʾ

    //!< RDS���ݣ������ڴ�ӡ���޹���״̬��ʾ

    //!< ���״̬
    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
    (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&(pDebug->jointSw&0x000F))
    {
        for(i=0;i<15;i++)       //���֡����֡����ͷ��
        {
            //if(((EwayEmbSys.Comm.wMotors[i]&EMB_COMM_ERR_BITS_3_TIMES)==0))   //!< ��Motorsͨ�Ų�����
            if((EwayEmbSys.Comm.wMotors[i]&0x01)==0)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    Bsp_printf("Motor[%d] offline,wMotor[]:0x%x.",i,EwayEmbSys.Comm.wMotors[i]);
                    /*if(i != 1)
                    {
                            st = ArmsReg[i].mRegs[0] + (ArmsReg[i].mRegs[1]<<8);
                        
                            Bsp_printf("Mo[%d] offlin,Sta:%0xx",(i+1),st);
                    }*/
                }
            }
            else
            {   
                if(i<12)//!< id=1-12 left and right arm
                {   
                    if(pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)
                    {
                        st = ArmsReg[i].mRegs[0] + (ArmsReg[i].mRegs[1]<<8);
                        if(st!=0)
                        {
                            Bsp_printf("Arm[%d] Err Status:0x%x.",(i+1),st);
                        }
                    }
                }
                else if(i==12)
                {
                    if(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK)
                    {
                        st = ShoulderReg.mRegs[0] + (ShoulderReg.mRegs[1]<<8);
                        if(st!=0)
                        {
                            Bsp_printf("Shoulder Err Status:0x%x.",st);
                        }
                    }
            }
            else if(i>12)//!< 13,14
            {
                if(pDebug->jointSw&JointSwDebugCtrl_Head_MASK)
                {
                    st = HeadsReg[(i-0x0D)].mRegs[0] + (HeadsReg[(i-0x0D)].mRegs[1]<<8);
                    if(st!=0)
                    {
                        Bsp_printf("Head[%d] Err Status:0x%x.",(i+1),st);
                    }
                }
            }
        }
    }
        }

    //!< ����״̬
    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
    (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
    {
        for(i=0;i<2;i++)
        {
            if((EwayEmbSys.Comm.wMotors[i+EMB_COMM_WHEEL_START_BIT]&EMB_COMM_ERR_BITS_3_TIMES)==0)
            //if((EwayEmbSys.Comm.wMotors[i+EMB_COMM_WHEEL_START_BIT]&0x01)==0)
            {
                Bsp_printf("Wheel Motor[%d] offlin,wMo:0x%02x",(i+1),EwayEmbSys.Comm.wMotors[i+EMB_COMM_WHEEL_START_BIT]);
            }
            else
            {
                st = WheelsReg[i].mRegs[0] + (WheelsReg[i].mRegs[1]<<8);
                if(st!=0)
                {
                    Bsp_printf("Wheel[%d] Err Status:0x%x.",(i+1),st);
                }
            }
        }
    }

    //!< צ���޷��صĹ���״̬
    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
    (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&(pDebug->jointSw&JointSwDebugCtrl_Gripper_MASK))
    {
        for(i=0;i<2;i++)
        {
            if((EwayEmbSys.Comm.wMotors[i+EMB_COMM_GRIP_START_BIT]&EMB_COMM_ERR_BITS_3_TIMES)==0)     //!< ��PSUͨ�Ų�����
            {
                Bsp_printf("Gripper[%d] offline,wMotor[]:0x%x.",(i+1),EwayEmbSys.Comm.wMotors[i+EMB_COMM_GRIP_START_BIT]);
            }
        }    
    }

}

/*

*/
void NetCommMaintain(void)
{

#if Emb_PC_HeartBeatPack_DEBUG

    //!< ��������ͨ�ų�ʱ������    
    if(EwayEmbTCP.TCP_State == ES_TCPSERVER_CONNECTED)
    {
        EwayEmbTCP.runTimeout += 1;         //!< ���ӳ�ʱ������������δ����ʱ������
    }
    else
    {
        if(EwayEmbTCP.runTimeout > 0)       //!< ����δ����ʱ�����������
        {
            EwayEmbTCP.runTimeout = 0;
        }
    }

    //!< ����Ƿ�ʱ��Ҫ�ر�����
    if((EwayEmbTCP.TCP_State == ES_TCPSERVER_CONNECTED)&&(EwayEmbTCP.runTimeout > EMB_PCHEARTBEAT_TIMEOUT_INTERVAL))
    {
        EwayEmbTCP.runTimeout = 0;
        
        EwayEmbTCP.runStatus = 'C';

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            Bsp_printf("PC HeartBeat Timeout,Close the TCP connection.");
        }        
    }
    
#endif

    //!< ���ڼ����������״̬
    CableLink_Check();
    
}

int main(void)
{     
    u8 i;
    bsp_Init();    

    my_mem_init(SRAMIN);    
    
    /* Create the Queues */
    EmbToLogWrQueHdl = xQueueCreate(CMD_EMBWRLOG_QUEU_NUM,CMD_EMBWRLOG_INFO_LEN);
    
    PCToShoulderQueHdl = xQueueCreate(CMD_PC2SHOULDER_QUEU_NUM,CMD_PC2SHOULDER_INFO_LEN);
    ShoulderToMotorQueHdl = xQueueCreate(CMD_SHOULDER2MOTOR_QUEU_NUM,CMD_SHOULDER2MOTOR_INFO_LEN);

    PCToArmsQueHdl = xQueueCreate(CMD_PC2ARMS_QUEU_NUM,CMD_PC2ARMS_INFO_LEN);
    ArmsLeftToMotorQueHdl = xQueueCreate(CMD_ARMS2MOTOR_QUEU_NUM,CMD_ARMS2MOTOR_INFO_LEN);
    ArmsRightToMotorQueHdl = xQueueCreate(CMD_ARMS2MOTOR_QUEU_NUM,CMD_ARMS2MOTOR_INFO_LEN);

    PCToHeadsQueHdl = xQueueCreate(CMD_PC2HEAD_QUEU_NUM,CMD_PC2HEAD_INFO_LEN);
    //HeadsToMotorQueHdl = xQueueCreate(CMD_HEAD2MOTOR_QUEU_NUM,CMD_HEAD2MOTOR_INFO_LEN);

    PCToWheelsQueHdl = xQueueCreate(CMD_PC2WHEEL_QUEU_NUM,CMD_PC2WHEEL_INFO_LEN);
    WheelsToMotorQueHdl = xQueueCreate(CMD_WHEEL2MOTOR_QUEU_NUM,CMD_WHEEL2MOTOR_INFO_LEN);

    PCToGrippersQueHdl = xQueueCreate(CMD_PC2GRIPPER_QUEU_NUM,CMD_PC2GRIPPER_INFO_LEN);
    GrippersToMotorQueHdl = xQueueCreate(CMD_GRIPPER2MOTOR_QUEU_NUM,CMD_GRIPPER2MOTOR_INFO_LEN);

    PCToEmbSysQueHdl = xQueueCreate(CMD_PC2EMBSYS_QUEU_NUM,CMD_PC2EMBSYS_INFO_LEN);
    EmbSysToMotorQueHdl = xQueueCreate(CMD_EMBSYSPROC_QUEU_NUM,CMD_EMBSYSPROC_INFO_LEN);
    
    EmbSysToPSMPSUHdl = xQueueCreate(CMD_EMBSYS2PSM_QUEU_NUM,CMD_EMBSYS2PSM_INFO_LEN);

    /* Create the semaphores */  
    osSemaphoreDef(NetSend_BinarySem);
    NetSend_BinarySemHandle = osSemaphoreCreate(osSemaphore(NetSend_BinarySem), 1);     //����֪ͨ���緢�����������ݸ���λ��
	
	/*Create the Mutex Objects*/     
    for(i=0;i<Motor_Num_ArmLeft;i++)
    {
        osMutexDef(i);
        PCCmdBuffOpHandleAml[i] = osMutexCreate(osMutex(i));        
    }
    for(i=0;i<Motor_Num_ArmRight;i++)
    {
        osMutexDef(i);
        PCCmdBuffOpHandleAmr[i] = osMutexCreate(osMutex(i));        
    }
    
    osMutexDef(0);
    PCCmdBuffOpHandleShd = osMutexCreate(osMutex(0));
    
    for(i=0;i<Motor_Num_Head;i++)
    {
        osMutexDef(i);
        PCCmdBuffOpHandleHed[i] = osMutexCreate(osMutex(i));        
    }
    for(i=0;i<Motor_Num_Wheel;i++)
    {
        osMutexDef(i);
        PCCmdBuffOpHandleWhl[i] = osMutexCreate(osMutex(i));        
    }
    for(i=0;i<Motor_Num_Gripper;i++)
    {
        osMutexDef(i);
        PCCmdBuffOpHandleGrp[i] = osMutexCreate(osMutex(i));        
    } 

    osMutexDef(NetCommSendOp);        
    NetCommSendOpHandle = osMutexCreate(osMutex(NetCommSendOp));   

    /* Create the tasks */    
    osThreadDef(TaskLogProc, StartTaskLogProc, osPriorityLow, 0, 1280);
    TaskLogProcHandle = osThreadCreate(osThread(TaskLogProc), NULL);
    
    //osThreadDef(TaskNetCommSend, StartTaskNetCommSend, osPriorityHigh, 0, 1280);
    //TaskNetCommSendHandle = osThreadCreate(osThread(TaskNetCommSend), NULL);
    
    osThreadDef(TaskNetCommRecv, StartTaskNetCommRecv,osPriorityHigh, 0, 1280);
    TaskNetCommRecvHandle = osThreadCreate(osThread(TaskNetCommRecv), NULL);   

    osThreadDef(TaskCOMSend, StartTaskCOMSend, osPriorityAboveNormal, 0, 1024);
    TaskCOMSendHandle = osThreadCreate(osThread(TaskCOMSend), NULL);

    osThreadDef(TaskSysRegular, StartTaskSysRegular, osPriorityNormal, 0, 2560);
    TaskSysRegularHandle = osThreadCreate(osThread(TaskSysRegular), NULL);  
    
#if Emb_System_TaskMonitor_Enable
    osThreadDef(TaskMonitor, StartTaskMonitor, osPriorityRealtime, 0, 512);
    TaskNetCommSendHandle = osThreadCreate(osThread(TaskMonitor), NULL);
#endif
    
//osThreadDef(TaskMain, StartTaskMain, osPriorityBelowNormal, 0, 2560);
    //TaskMainHandle = osThreadCreate(osThread(TaskMain), NULL);
        
    if(osOK==osKernelStart())
    {
#if PRINT_ETH_DEBUG_INFO
        Bsp_printf("freeRTOS Kernel Start successful.");        //!< can't print this if osKernel Start
#endif
    }
    else
    {
#if PRINT_ETH_DEBUG_INFO
        Bsp_printf("freeRTOS Kernel Start Failed.");
#endif
    }

}

void StartTaskCOMSend(void const * argument)
{
    u8 dat[CMD_SHOULDER2MOTOR_INFO_LEN]={0};
    u8 i;
    COM_PORT_E unPort;
    s8 res;
    osStatus tSta = osOK;

    if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
    {
        //res = Bsp_printf("4-0 TaskCOMSend Start.");
    }
    
    while(1)
    {        
testCnt[4] = 1;

        for(i=0;i<6;i++)
        {        
            if(EwayEmbSys.COMSendCtrl[i].tCtr.en > 0)                            //!< ���ʱ�����ģ���Ƿ�ʹ�ܣ�ģ�鹦��δʹ�ܲ��ܷ��Ͷ�������
            {          
testCnt[4] = 2;

                if(EwayEmbSys.COMSendCtrl[i].tCtr.Cnt > 0)                        //!< ���Cnt�Ƿ����0
                {                    
testCnt[4] = 3;

                    EwayEmbSys.COMSendCtrl[i].tCtr.Cnt -= 1;
                }
                else                                                            //!< Cnt == 0
                {        
testCnt[4] = 4;
                    while(xQueueReceive(*(EwayEmbSys.COMSendCtrl[i].pComQ),dat,0) == pdPASS)  //!< �������Ƿ��д���������
                    {     
testCnt[4] = 5;

                        if(sysGetMotorCommPort(EwayEmbSys.COMSendCtrl[i].devTy,&unPort)==ERR_NONE)    //!< ��������
                        {
testCnt[4] = 6;

                            if(unPort == COM7)
                            {
                                comSendBuf(COM7,&dat[2],dat[1]);

                                res = ERR_NONE;
                            }
                            else
                            {
                                res = UartDmaTransferConfig(unPort,&dat[2],dat[1]); 
                            }

                            if(ERR_NONE == res)
                            {    
testCnt[4] = 7;

                                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_2_Motor_MASK)&&(pDebug->secondFun&SecFunDebugCtrl_COM_Send_MASK))
                                {
                                    if(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK)
                                    {
                                        if((i==0)&&(dat[0]==0))
                                        {
                                            res = Bsp_printf("COMSend to COM2 iD:0x%x,Len:%d,Code:0x%x,RegStart:0x%x,RegN:%d",dat[4],dat[5],dat[6],dat[7],dat[8]);
                                        }                                
                                    }
    
                                    if(pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)
                                    {
                                        if((i==1)&&(dat[0]==0))
                                        {
                                            res = Bsp_printf("COMSend to COM3 iD:0x%x,Len:%d,Code:0x%x,RegN:%d.",dat[4],dat[5],dat[6],dat[8]);
                                        }
                                    }
                                
                                    if(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)
                                    {                                
                                        if((i==2)&&(dat[0]==0))
                                        {
                                            res = Bsp_printf("COMSend to COM4 iD:0x%x,Len:%d,Code:0x%x,RegN:%d.",dat[4],dat[5],dat[6],dat[8]);
                                        }
                                    }

                                    if(pDebug->jointSw&JointSwDebugCtrl_Gripper_MASK)
                                    {
                                        if((i==3)&&(dat[0]==0))
                                        {
                                            res = Bsp_printf("COMSend to COM5 iD:0x%x,Len:%d,Code:0x%x,RegN:%d.",dat[4],dat[5],dat[6],dat[8]);
                                        }
                                    }
                                
                                    if(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK)
                                    {
                                        if((i==4)&&(dat[0]==0))
                                        {
                                            res = Bsp_printf("COMSend to COM6 iD:0x%x,Len:%d,Code:0x%x,RegN:%d.",dat[4],dat[5],dat[6],dat[8]);
                                        }
                                    }
                                
                                    if((pDebug->jointSw&JointSwDebugCtrl_PSM_MASK)||(pDebug->jointSw&JointSwDebugCtrl_PSU_MASK))
                                    {
                                        if((i==5)&&(dat[0]==0))
                                        {
                                            res = Bsp_printf("COMSend to COM7 iD:0x%x,Len:%d,Code:0x%x,RegN:%d.",dat[4],dat[5],dat[6],dat[8]);
                                        }
                                    }

                                }
                            }
                            else
                            {
testCnt[4] = 8;

                                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                                {
                                    res = Bsp_printf("COMSend to COM%d failed.",((u8)unPort)+1);
                                }
                            }                    
                        }
testCnt[4] = 9;
                        if(0 != dat[0]) //!< �����ظ���ָ��
                        {         
testCnt[4] = 10;
                            if(unPort == COM7)
                            {
                                if(dat[0]>COM_SEND_INTERVAL_PSM_QUERY)
                                {
                                    dat[0] = COM_SEND_INTERVAL_PSM_QUERY;
                                }
                                
                                EwayEmbSys.COMSendCtrl[i].tCtr.Cnt = dat[0];
                            }
                            else
                            {
                                EwayEmbSys.COMSendCtrl[i].tCtr.Cnt = EwayEmbSys.COMSendCtrl[i].tCtr.tCnt-1;    //!< ????Cnt
                            }
testCnt[4] = 11;
                            break;    //!< ����while()ѭ��
                        }

                    }//!< �������޴���������
                }
            }
        }
testCnt[4] = 12;

        EwayEmbSys.iWdg.CurCnt[EMB_SYSTEM_TASK_COMSend] += 1;

        tSta = osDelay(2);
    
        if(osOK != tSta)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                Bsp_printf("3-1 osDelay(2) failed.");
            }
        }
    }

}

/*
    1. ���tcp_tmp.bsy �� tcp_tmp.SndCnt�Ƿ��ܹ���ʾ�з�������
        ��û�У����˳�
        ���У������2.

    2. ��鵱ǰ�����Ƿ������ӵ�
        �������ӵģ������3.
        ���ǲ����ӵģ������4.

    3. 

    4. ��ǰ����δ���ӣ��������������

*/
void NetCommSend(void)
{
    osStatus re;
    s8 res;

    if((EwayEmbTCP.tcp_tmp.bsy == 1)&&(EwayEmbTCP.tcp_tmp.SndCnt > 0))
    {        
        //!< �����������״̬
        if(EwayEmbTCP.TCP_State == ES_TCPSERVER_CONNECTED)
        {      
            re = osMutexWait(NetCommSendOpHandle,1);

            if(re == osOK)
            {
                res = SysGeneralTcpSendData(&EwayEmbTCP.tcp_tmp);

                if( ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("SysGeneralTcpSendData() failed.rt:%d.",res);
                    }
                }
            
                EwayEmbTCP.tcp_tmp.SndCnt = 0;              //!< ����������ݸ��������ͻ���        
                EwayEmbTCP.tcp_tmp.bsy = 0;
                memset(EwayEmbTCP.tcp_tmp.pSnd,0,TCP_SERVER_RX_BUFSIZE);  

                osMutexRelease(NetCommSendOpHandle);                
            }
            else
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    res = Bsp_printf("6.3 Get NetCommSendOpHandle-1 failed.rt:%d.",re);
                }
            }
        }
        else            //!< ����δ����ʱ��������������ݷ�ֹ����
        {        
            if(EwayEmbTCP.tcp_tmp.bsy!=0)
            {            
                re = osMutexWait(NetCommSendOpHandle,1);

                if(re == osOK)
                {
                    EwayEmbTCP.tcp_tmp.SndCnt = 0;              //!< ����������ݸ��������ͻ���        
                    EwayEmbTCP.tcp_tmp.bsy = 0; 
                    memset(EwayEmbTCP.tcp_tmp.pSnd,0,TCP_SERVER_RX_BUFSIZE);      

                    osMutexRelease(NetCommSendOpHandle);                
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("6.3 Get NetCommSendOpHandle-2 failed.rt:%d.",re);
                    }
                }
            }
        }
    }    
  
}


void StartTaskNetCommSend(void const * argument)
{
    s8 res;

    if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
    {
        //res = Bsp_printf("5-0 TaskNetCommSend Start.");   
    }
    
    while(1)
    {
testCnt[3] = 1;
        if(pdTRUE == xSemaphoreTake(NetSend_BinarySemHandle, portMAX_DELAY))
        {        
            //!< �����������״̬
            if(EwayEmbTCP.TCP_State == ES_TCPSERVER_CONNECTED)
            {
                ////////////////////////////�жϵ�ǰ���緢��״̬                
                res = SysGeneralTcpSendData(&EwayEmbTCP.TCP_Trans);                
testCnt[3] = 2;

                if( ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("6.3 SysGeneralTcpSendData() failed.rt:%d.dNums:%d.",res,EwayEmbTCP.TCP_Trans.SndCnt);
                    }
                }
                else
                {
                    //res = Bsp_printf("6.3 SysGeneralTcpSendData() successful.dNums:%d.",EwayEmbTCP.TCP_Trans.SndCnt);
                }
                
                EwayEmbTCP.TCP_Trans.SndCnt = 0;              //!< ����������ݸ��������ͻ���        
                EwayEmbTCP.TCP_Trans.bsy = 0;
                memset(EwayEmbTCP.TCP_Trans.pSnd,0,TCP_SERVER_RX_BUFSIZE);     
testCnt[3] = 3;
            }
            else
            {
testCnt[3] = 4;

                if(EwayEmbTCP.TCP_Trans.bsy!=0)
                {
testCnt[3] = 5;

                    EwayEmbTCP.TCP_Trans.SndCnt = 0;              //!< ����������ݸ��������ͻ���        
                    EwayEmbTCP.TCP_Trans.bsy = 0;
                    memset(EwayEmbTCP.TCP_Trans.pSnd,0,TCP_SERVER_RX_BUFSIZE);        
                }
testCnt[3] = 6;
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    res = Bsp_printf("6.3 TCP is not connected,at line %d in %s.",__LINE__, __FILE__);
                }                
            }
        }    
    }
}




void StartTaskNetCommRecv(void const * argument)
{
    s8 res;

    osStatus tSta;

    if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
    {
        //res = Bsp_printf("6-0 TaskNetCommRecv Start.");   //
    }  

    res = LwipComm_Init();
    
    if(0 != res)
    {
        Bsp_printf("6-1 LWIP Comm Init failed.");
    }

    TcpServer_Init();

    while(1)
    {        
testCnt[2] = 1;
        tSta = osDelay(TCP_TMR_INTERVAL);     //!< ÿ15ms����һ��
        if(osOK != tSta)
        {
            res = Bsp_printf("6-1 osDelay(15) failed.");
        }
testCnt[2] = 2;

        ETH_DMAITConfig(ETH_DMA_IT_NIS|ETH_DMA_IT_R,DISABLE);
        
        LwipHandle_Periodically();    //!< ������ѯ����  
        
        ETH_DMAITConfig(ETH_DMA_IT_NIS|ETH_DMA_IT_R,ENABLE);
        
testCnt[2] = 3;

        sysExtractLegalPktFromTCP(&EwayEmbTCP.Recs);
testCnt[2] = 4;

        //!< ���PC���͵�ָ��:�֡�ͷ���硢צ��(���ӡ�ϵͳδ���)
        sysPcCmdProcess();  
		
        NetCommSend();

        NetCommMaintain();
        
        EwayEmbTCP.runCnt ++;

        EwayEmbSys.iWdg.CurCnt[EMB_SYSTEM_TASK_NetCommRecv] += 1;
    }

}



void StartTaskSysRegular(void const * argument)
{    
    osStatus tSta = osOK;
    char Ver[100]={0};

    EmbSys_Init();

    if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
    {
        sprintf(&Ver[0],"\r\nCy:%d,sysCnt:%d Emb System start running.\r\n",MySysTick,EwayEmbSys.sysTimCnt);        
        PrintfUart1DmaTransferConfig((u8*)(&Ver[0]),strlen((&Ver[0])));
    }
    
    /*
    pDebug->sysDebugCtrl |= (SysDebugCtrl_Emb_Proc_PC_Cmd_MASK|SysDebugCtrl_Motor_2_Emb_MASK|SysDebugCtrl_EmbSysInterLogicProc);
    pDebug->secondFun |= (SecFunDebugCtrl_XMovementProcess_MASK|SecFunDebugCtrl_PeriodCommStatus_MASK);
    pDebug->jointSw = 0x0000;//(JointSwDebugCtrl_Wheel_MASK|JointSwDebugCtrl_ArmRight_MASK|JointSwDebugCtrl_ArmLeft_MASK|JointSwDebugCtrl_Shoulder_MASK|JointSwDebugCtrl_Head_MASK);
*/

/* ���Լ��ͷ�� 
    pDebug->sysDebugCtrl |= (SysDebugCtrl_Emb_2_Motor_MASK);
    pDebug->secondFun |= (SecFunDebugCtrl_COM_Send_MASK);
    pDebug->jointSw |= JointSwDebugCtrl_Shoulder_MASK;     
*/                         

/*  ��������
    pDebug->sysDebugCtrl |= (SysDebugCtrl_Emb_Proc_PC_Cmd_MASK);
    pDebug->secondFun |= (SecFunDebugCtrl_XMovementProcess_MASK);
    pDebug->jointSw = JointSwDebugCtrl_Wheel_MASK;
*/

/* ����Mic*/
    
    //pDebug->sysDebugCtrl |= (SysDebugCtrl_Emb_Proc_PC_Cmd_MASK);
    //pDebug->secondFun |= (SecFunDebugCtrl_CmdProcInfo_MASK);
    //pDebug->jointSw = JointSwDebugCtrl_MIC_MASK;//(JointSwDebugCtrl_Wheel_MASK|JointSwDebugCtrl_ArmRight_MASK|JointSwDebugCtrl_ArmLeft_MASK|JointSwDebugCtrl_Shoulder_MASK|JointSwDebugCtrl_Head_MASK);



/* ����צ���˶�
            pDebug->sysDebugCtrl = 0x8005;
            pDebug->jointSw = 0x20;
            pDebug->secondFun = 0x00000010;
*/
    
/*  �޸ĳ����ư岨����
        SendWriteBatteryChargerBaudRate();
*/
    
/*  ����Radar  */
    // pDebug->sysDebugCtrl |= (SysDebugCtrl_EmbSysInterLogicProc);
    ////////pDebug->jointSw |= (JointSwDebugCtrl_ObstacleAvoid_MASK);
    //pDebug->secondFun |= (SecFunDebugCtrl_CmdProcInfo_MASK);//SecFunDebugCtrl_PeriodCommStatus_MASK
    
    //EwayRadarObstAvoid.ObstAvoid.Swi = 1;

    //sysGeneralMotorBroadcastRead(0x0D,1,EMB_READ_WHEELMODE_POS_PID_START_REG,EMB_READ_WHEELMODE_POS_PID_REG_NUM,ArmSvdelayTm,EWAYBOT_SHOULDER);
    //FF FF FE 0B 04 21 06 0D 01 19 C0
    while(1)
    {        
    testCnt[0] = 1;
        ////// 1.�����ͨ��״̬��ϵͳ�������ֵ�����״̬  ////////////////////
        //---  ������ֱۡ����ͷ�����,����λ����PSM��PSU��ͨ��״̬
        sysEmbCommStateUpdateAndMonitor(); 
testCnt[0] = 2;

        //sysPrintErrorStatus();//!< �����״̬λ�д�����һֱ��ӡ
testCnt[0] = 3;

        ////// 2. ����ά��IMU,������λ״̬�������ߴ�����  /////////////////////
        sysMaintainWorkPeriodically(MySysTick);
testCnt[0] = 4;

        ////// 3. �����·������(�ּ�ͷצ��)��ָ���    /////////////////////
        sysCmdToMotorProcess();
testCnt[0] = 5;

        ////// 4. ���״̬�ռ�
        sysEmbMotorStatusCollect();
testCnt[0] = 6;

        ////// 4.���ڷ�״̬��Ϣ����λ��                   /////////////////////
        sysSendEmbInfoToPC();
testCnt[0] = 7;
        ////// 5.���͵��״̬��ѯָ��                      /////////////////////
        //---  5.1 Clearϵͳʵʱͨ��״̬ ---//
        if((EwayEmbSys.sysTimCnt%100)==0)        //������ʾĿǰ�Ѿ��յ���ָ����        
        {
            //res = Bsp_printf("3-6 Emb Head recvd Instructions:%d.",EwayHeads.mIns,0,0,0,0,0);
                
            //res = SysLogWrite(LogMajor,"3-6 EmbState1:wPC:0x%x,wMotors[0,14]:0x%x,wWheels:0x%x,wGrip:0x%x,wPSM:0x%x,wPSU:0x%x.",(EwayEmbSys.rtComm&0x00080000),\
            //(EwayEmbSys.rtComm&0x00007FFF),(EwayEmbSys.rtComm&0x00018000),(EwayEmbSys.rtComm&0x00060000),(EwayEmbSys.rtComm&0x00100000),(EwayEmbSys.rtComm&0x00200000));
            
            //res = Bsp_printf("DebugCtrl:0x%08x,Sec:0x%08x,EmbSta1:wPC:0x%x,wMots[0,14]:0x%x,wWhls:0x%x,wGrip:0x%x,wPSM:0x%x,wPSU:0x%x.",((pDebug->sysDebugCtrl << 16) + pDebug->jointSw),pDebug->secondFun,\
            //(EwayEmbSys.rtComm&(0x00000001<<EMB_COMM_PC_START_BIT)),(EwayEmbSys.rtComm&0x00007FFF),(EwayEmbSys.rtComm&0x00018000),(EwayEmbSys.rtComm&0x00060000),EwayEmbSys.Comm.wPsm,EwayEmbSys.Comm.wPsu);//,sysEwServo[12].uPosWheel

            //Bsp_printf("DebugCtrl:0x%08x,Sec:0x%08x,EmbSta:PC:%02x,Mo[0,14]:%x,Whl:%x,Gr:%x,PSM:%x,PSU:%x,Mic:%x,WhlMd:%d.",((pDebug->sysDebugCtrl << 16) + pDebug->jointSw),pDebug->secondFun,\
            //EwayEmbSys.Comm.wPc,(EwayEmbSys.rtComm&0x00007FFF),((EwayEmbSys.rtComm&0x00018000)>>15),((EwayEmbSys.rtComm&0x00060000)>>17),EwayEmbSys.Comm.wPsm,EwayEmbSys.Comm.wPsu,EwayEmbSys.Comm.wMic,EwayWheels.mControl.mCtrlMode);
        
            Bsp_printf("DbCt:0x%08x,0x%x.PC:%02x,Mo[0,14]:%x,Whl:%x,Gr:%x,PSM:%x,PSU:%x,Mic:%x,WhlMd:%d,tC:%d.",((pDebug->sysDebugCtrl << 16) + pDebug->jointSw),pDebug->secondFun,\
            EwayEmbSys.Comm.wPc,(EwayEmbSys.rtComm&0x00007FFF),((EwayEmbSys.rtComm&0x00018000)>>15),((EwayEmbSys.rtComm&0x00060000)>>17),EwayEmbSys.Comm.wPsm,EwayEmbSys.Comm.wPsu,\
            EwayEmbSys.Comm.wMic,EwayWheels.mControl.mCtrlMode,EwayEmbTCP.runTimeout);

        }
            //!< ���ٶ�Ax,Ay,Az,���ٶ�Gx,Gy,Gz,�Ƕ�Roll,Pitch,Yaw
       /* if((EwayEmbSys.sysTimCnt%40)==0)
        {
            Bsp_printf("ImuComm:0x%x,Ax:%d,Ay:%d,Az:%d,Gx:%d,Gy:%d,Gz:%d,Roll:%d,Pitch:%d,Yaw:%d.",EwayImu.Comm.sta,EwayImu.Res[0],EwayImu.Res[1],EwayImu.Res[2],\
                EwayImu.Res[3],EwayImu.Res[4],EwayImu.Res[5],EwayImu.Res[6],EwayImu.Res[7],EwayImu.Res[8]);
        }*/
        //���ϵͳʵʱͨ��״̬����,����ͨ��������PC,Motors(),PSM,PSU��ͨ�����ݣ�������Ӧ�ı�־λ
        EwayEmbSys.rtComm = 0;

        //---  5.2 ���ڶ����(�����֡����ͷ����צ�ӡ�����)��״̬�Ĵ���        
        sysReadMotorsStatus();   
testCnt[0] = 8;

        
        //---  5.3 ���ڶ�PSM PSU״̬ 
        sysReadPsmPsuStatus();
testCnt[0] = 9;
        
		//---  5.4 ���ڶ�Mic״̬
        sysReadMicStatus();
testCnt[0] = 10;
        
        ////// 6. �ӳٵȴ�               //////////////////////////////////////        
        tSta = osDelay(15);
        if(osOK != tSta)
        {
            Bsp_printf("3-1 osDelay(15) failed,rt:%d.");
        }
testCnt[0] = 11;

        ////// 7. ������յ��ĵ������:�֡�ͷ���硢צ�ӡ�����  ////////////////
        sysProcessRecvDataFromMotors(); 
testCnt[0] = 12;

        ////// 8. �����յ���PSM,PSU����                        ///////////////
        sysProcessRecvDataFromPsmPsu();
testCnt[0] = 13;

		////// 9. �����յ���MIC����                            ///////////////
        sysProcessRecvDataFromMic();
testCnt[0] = 14;

        ////// 10.����ϵͳ������                               ////////////////
        EwayEmbSys.sysTimCnt++;
        
        SysTimeStamp = EwayEmbSys.sysTimCnt;    //!< ����ϵͳʱ���

        EwayEmbSys.iWdg.CurCnt[EMB_SYSTEM_TASK_SysRegular] += 1;
        
testCnt[0] = 15;

        tSta = osDelay(15);        
        if(osOK != tSta)
        {
            Bsp_printf("3-1 osDelay(15) failed,rt:%d.");
        }
    }
}


/* 
��ʱ����
*/
s8 sysEmbLogFileNameInit(void)
{
    EwayEmbSys.tStmp.Init = ((EMB_System_SoftVer_Year-2018)<<28)+((EMB_System_SoftVer_Month&0x0F)<<24)+((EMB_System_SoftVer_Day&0xFF)<<16);

    return ERR_NONE;
}



void StartTaskLogProc(void const * argument)
{
    s8 res;
    char* pDat;
    char dat[128]={0};//{"TaskLogProc1234567890."};    
    char Ver[100]={0};    

    osDelay(100);                //!< �ȴ���λ������λ������������

    sysEmbLogFileNameInit();    //!< �Ƚ��汾��������Ϊ�ļ���

    if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
    {
        //res = Bsp_printf("1-0 TaskLogProc Start.");
    }

    ///!< ��ʼ��TF���ӿ�
    res = sysEmbLogFileInit(&EwayEmbFileSys);
    
    if(res != ERR_NONE)
    {
        Bsp_printf("1-0 StartTaskLogProc()--->sysEmbLogFileInit() failed.");
    }
    else    // ��¼��λ������汾
    {
        if((EwayEmbFileSys.tfEn&0x0F)==LOG_FILE_TF_WR_EN)
        {
            pDat = EwayEmbSysInfo.SoftVersion;
            
            sprintf(&Ver[0],"\r\nCy:%d,sysCnt:%d Emb System start running.Date:%d-%d-%d,%d.",MySysTick,EwayEmbSys.sysTimCnt,(pDat[2]+(pDat[3]<<8)),pDat[4],pDat[5],pDat[6]);

            f_puts((const TCHAR*)(&Ver[0]),&(EwayEmbFileSys.fSrc.efile));
        }        
    }

    while(1)
    {
testCnt[1] = 1;

        //!< Log��д����״̬ά��
        //!< ��TF����;���γ��������Ӵ��������������Ӧ��־λ��ʱ����
        if(EwayEmbFileSys.errCnt>=2)
        {
            EwayEmbFileSys.errCnt = 0;

            EwayEmbFileSys.tfEn = 0x00;
        }
testCnt[1] = 2;

        //!< ���ڼ��Log�ļ��Ƿ��Ѿ�����ʼ��������Ƶ������̫�ߣ�ע����ƽ���
        if((EwayEmbFileSys.fTimCnt%200)==0)
        {
            res = sysEmbLogFileChecking(&EwayEmbFileSys);
            if(res != ERR_NONE)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    Bsp_printf("1-1 sysEmbLogFileChecking() failed.");
                }        
            }
        }
testCnt[1] = 3;

        //!< �����Ϣ�����Ƿ��д�д���Log��¼
        while(xQueueReceive(EmbToLogWrQueHdl,dat,0) == pdPASS)  //!< �������Ƿ��д���������
        {
            if((dat[0]>0)&&((EwayEmbFileSys.tfEn&0x0F)==LOG_FILE_TF_WR_EN))
            {                
                //!< �������ݳ��ȣ�д���ݵ�SD��                
                res = f_puts((const TCHAR*)(&dat[1]),&(EwayEmbFileSys.fSrc.efile));

                if(res == dat[0])
                {
                    osDelay(2);//!< ��ʱʱ����д������ʱ����ȷ��
                }
                else
                {
                    EwayEmbFileSys.errCnt++;
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        Bsp_printf("1-2 f_puts() failed.errCnt:%d.",EwayEmbFileSys.errCnt);
                    }
                }
            }
        }
testCnt[1] = 4;

        //!< ����Ƿ���Ҫ��ʱͬ���ļ�����
        if(((EwayEmbFileSys.fTimCnt%2)==0)&&((EwayEmbFileSys.tfEn&0x0F)==LOG_FILE_TF_WR_EN))
        {
            res = sysEmbLogFileSync();

            if(res!=ERR_NONE)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    Bsp_printf("1-3 sysEmbLogFileSync() failed.\n");
                }
            }            
        }
testCnt[1] = 5;

        EwayEmbFileSys.fTimCnt++;

        EwayEmbSys.iWdg.CurCnt[EMB_SYSTEM_TASK_LogProc] += 1;

        osDelay(5);    //!< Ŀǰ�ȶ�5ms���ɱ�֤д��һ������
    }   
}



void sysMonitorTasksProcess(void)
{
    u8 i;
    u8 CurCnt0[10]={0};

    memcpy(CurCnt0,&EwayEmbSys.iWdg.CurCnt[0],5);   

    //!< ����������ִ�е����
    for( i = 0; i< (EMB_SYSTEM_TASKS_NUMS-2);i++)       //!< ����� StartTaskMonitor()
    {
        if(EwayEmbSys.iWdg.CurCnt[i] < EwayEmbSys.iWdg.InitCnt[i])
        {  
            break;
        }
        else
        {
            EwayEmbSys.iWdg.CurCnt[i] = 0;
        }
    }

    if( i >= (EMB_SYSTEM_TASKS_NUMS-2))    
    {
        //!< ι��
        IWDG_Feed();
    }
    else
    {
        SysLogWrite(LogCiritcal,"SysTask(%d) Run Away!CurCnt:%d-%d-%d-%d.Reset Emb System later.",(i+1),CurCnt0[0],CurCnt0[1],CurCnt0[2],CurCnt0[3]);
        Bsp_printf("SysTask(%d) Run Away!CurCnt:%d-%d-%d-%d.Reset Emb System later.",(i+1),CurCnt0[0],CurCnt0[1],CurCnt0[2],CurCnt0[3]);
    }
    
}

void StartTaskMonitor(void const * argument)
{
    //Bsp_printf("1-0 TaskMonitor Start.");

    while(1)
    {
        osDelay(500);
        
        sysMonitorTasksProcess();      //!< ����������ִ�е����        
    }
}


/*
void Assertfailed(u8 *file,u32 line)
{ 
    //char dat[100]={0};
    s8 res;
    // 
    //    �û���������Լ��Ĵ��뱨��Դ�����ļ����ʹ����кţ����罫�����ļ����кŴ�ӡ������
    //
    // ����һ����ѭ��������ʧ��ʱ������ڴ˴��������Ա����û���� //
        while(1)
        {
#if PRINT_ASSERT_DEBUG
        //sprintf(dat," %s:%d\r\n",   __FILE__,__LINE__);
        res = Bsp_printf(" %s:%d\r\n",   __FILE__,__LINE__);
#endif
        }
}
*/

/***************************** END OF FILE *********************************/

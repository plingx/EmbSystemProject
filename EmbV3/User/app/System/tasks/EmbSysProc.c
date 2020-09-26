/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file EmbSysProc.c
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author 
* @version 0.0.1
* @date 2018-01-29
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/01/18 | 0.0.1 | Ling       | Create file
*
*/
#include "includes.h"


extern EwayEmbSysModule EwayEmbSys;
extern EwayEmbTCPModule EwayEmbTCP;
extern EwayPSMModule EwayPSM;
extern EwayArmModule EwayArms;
extern EwayShoulderModule EwayShoulder;
extern EwayHeadModule EwayHeads;
extern EwayGripperModule EwayGrippers;
extern EwayWheelModule EwayWheels;
extern EwayBatteryChargerModule EwayBatteryCharger;
extern EwayEmbSysInfoModule EwayEmbSysInfo;
extern UART_T g_tUart7;
extern UART_T g_tUart8;
extern EwayEmbSysDebugModule* pDebug;
extern EwayMicModule EwayMic;
extern const float MotorReductionRatio[6];
extern u8 ArmSvdelayTm[6];
extern __IO u32 SysTimeStamp;

extern QueueHandle_t PCToArmsQueHdl;
extern QueueHandle_t PCToShoulderQueHdl;
extern QueueHandle_t PCToHeadsQueHdl;
extern QueueHandle_t PCToGrippersQueHdl;
extern QueueHandle_t PCToWheelsQueHdl;


u8 MotorCfgReg[SYS_MAX_GENERAL_SERVO_NUMS][GENERAL_SERVO_CFG_REGS]={0};
EwayMotor_Device_Type EmbSysPartType[5]={EWAYBOT_ARMS_LEFT,EWAYBOT_SHOULDER,EWAYBOT_HEAD,EWAYBOT_GRIPPERS,EWAYBOT_WHEEL};
QueueHandle_t* EmbSysPartQue[5]={&PCToArmsQueHdl,&PCToShoulderQueHdl,&PCToHeadsQueHdl,&PCToGrippersQueHdl,&PCToWheelsQueHdl};
MotorCMDBuff** EmbSysPartCmdBuff[5]={&EwayArms.mControl.pToMotorCmd,&EwayShoulder.mControl.pToMotorCmd,&EwayHeads.mControl.pToMotorCmd,&EwayGrippers.mControl.pToMotorCmd,&EwayWheels.mControl.pToMotorCmd};




s8 PrintfUart1DmaTransferConfig(u8 *pSndBuf,u16 len);
s8 l_FillMotorSendingCfgRegsBuffer(NodeCtrlModule* pmCtrl,u8* pDat);
s8 l_CheckMotorsCfgRegComparedResultAndPackSettingData(EwayMotor_Device_Type devType,NodeCtrlModule* pMoCtrl,u32 mReadCfgRegCommSta);
s8 g_EmbSendQueryPacketToReadMotorCfgRegisters(EwayMotor_Device_Type devType);
s8 sysGetMotorSendRecordPointer(EwayMotor_Device_Type devType,MotorRecordBuff** pMRec,u8* max_iD,u8* startId);


/* --------------------------------------------------------------------------*/
/**
* @name sysInit
* @brief 
* @details 
*
* @param[in] None
*
* @returns res
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 EmbSys_Init(void)
{
    s8 res=ERR_NONE;
    char* pDat;
    EwayEmbSysMotorInfoModule *pMotor;
    char Ver[100]={0};

    //!< 上电后先读E2prom内容，若通过再执行其他操作
    res = sysReadEmbSystemInfoFromE2prom();

    if(res!=ERR_NONE)
    {
        LedsStatus_Set(LED0,LIGHT_ON);
        LedsStatus_Set(LED1,LIGHT_ON);
        while(1)
        {
            LedsStatus_Toggle(LED0);
            LedsStatus_Toggle(LED1);
            
            Bsp_printf("Read E2Prom Error.Emb System Stop Running.");
            
            osDelay(2000);
        }
    }
    else
    {
        pMotor = &EwayEmbSysInfo.mMotor[0];     //.mReductionRatioMode;

        pDat = EwayEmbSysInfo.SoftVersion;

        if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
        {
        sprintf(&Ver[0],"\r\n::::EmbInfo:Type1:0x%x,Type2:0x%x,Date:%d-%d-%d,%d.\r\n",pDat[0],pDat[1],(pDat[2]+(pDat[3]<<8)),pDat[4],pDat[5],pDat[6]);        
        PrintfUart1DmaTransferConfig((u8*)(&Ver[0]),strlen((&Ver[0])));
        osDelay(2);
        
        sprintf(&Ver[0],"\r\n::::RdsInfo:En:%d,init:%d,HigLmt:%d,LowLmt:%d.\r\n",EwayEmbSysInfo.Rds.En,EwayEmbSysInfo.Rds.init,EwayEmbSysInfo.Rds.HigLmt,EwayEmbSysInfo.Rds.LowLmt);
        PrintfUart1DmaTransferConfig((u8*)(&Ver[0]),strlen((&Ver[0])));
        osDelay(2);

        sprintf(&Ver[0],"\r\n::::Motor Type:tA1-(%d),tA2-(%d),tA3-(%d),tA4-(%d),tALift-(%d).",(int)(tA1),(int)(tA2),(int)(tA3),(int)(tA4),(int)(tALift));
        PrintfUart1DmaTransferConfig((u8*)(&Ver[0]),strlen((&Ver[0])));
        osDelay(2);

        sprintf(&Ver[0],"\r\n::::ArmL:(%d)-(%d)-(%d)-(%d)-(%d)-(%d).Head:(%d)-(%d).",(int)(pMotor->mReductionRatioMode),(int)((pMotor+1)->mReductionRatioMode),(int)((pMotor+2)->mReductionRatioMode),\
                                                            (int)((pMotor+3)->mReductionRatioMode),(int)((pMotor+4)->mReductionRatioMode),(int)((pMotor+5)->mReductionRatioMode),\
                                                            (int)((pMotor+13)->mReductionRatioMode),(int)((pMotor+14)->mReductionRatioMode));
        PrintfUart1DmaTransferConfig((u8*)(&Ver[0]),strlen((&Ver[0])));
        osDelay(2);

        sprintf(&Ver[0],"\r\n::::ArmR:(%d)-(%d)-(%d)-(%d)-(%d)-(%d).Shou:(%d).\r\n",(int)((pMotor+6)->mReductionRatioMode),(int)((pMotor+7)->mReductionRatioMode),(int)((pMotor+8)->mReductionRatioMode),\
                                                            (int)((pMotor+9)->mReductionRatioMode),(int)((pMotor+10)->mReductionRatioMode),(int)((pMotor+11)->mReductionRatioMode),(int)((pMotor+12)->mReductionRatioMode));
        PrintfUart1DmaTransferConfig((u8*)(&Ver[0]),strlen((&Ver[0])));
        osDelay(2);        
        
        }
    
    }

    return res;
}



s8 l_ComparePsmPsuResponseCount(PSMPSUReRecord* pRe)
{
    u8 tmp1,tmp2;

    tmp1 = pRe->sQueryRecd >> 4;
    tmp2 = pRe->sQueryRecd & 0x0F;

    if(tmp1!=tmp2)
    {
        return ERR_RESULT_NOT_EQUAL;
    }

    tmp1 = pRe->sExeCmRecd >> 4;
    tmp2 = pRe->sExeCmRecd & 0x0F;

    if(tmp1!=tmp2)
    {
        return ERR_RESULT_NOT_EQUAL;
    }

    return ERR_NONE;
}


void l_ClearPsmPsuResponseCount(PSMPSUReRecord* pRe)
{
    pRe->sExeCmRecd = 0;
    pRe->sQueryRecd = 0;
}


s8 l_CompareMotorsExeCmResponseCount(MotorReRecord* pRe)
{
    u8 tmp1,tmp2;

    tmp1 = pRe->mExeCmRecd[0] >> 4;
    tmp2 = pRe->mExeCmRecd[0] & 0x0F;

    if(tmp1!=tmp2)
    {
        return ERR_RESULT_NOT_EQUAL;
    }

    return ERR_NONE;
}


/* --------------------------------------------------------------------------*/
/**
* @name l_CompareMotorsQueryCmResponseCount
* @brief 
* @details 
*
* @param[in] None
*
* @returns res
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 l_CompareMotorsQueryCmResponseCount(MotorReRecord* pRe)
{

return ERR_NONE;

}


void l_ClearMotorResponseCount(MotorReRecord* pRe)
{
    pRe->mExeCmRecd[0] = 0;
    pRe->mExeCmRecd[1] = 0;
    //pRe->sQueryRecd = 0;
}



/* --------------------------------------------------------------------------*/
/**
* @name sysEmbCommStateUpdate
* @brief 
* @details 更新下位机与各个外部设备通信状态，将当前状态存入EwayEmbSys.rtComm中
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
#if Emb_Wheel_Offline_Debug
extern u32 EwayWheelCommState[2];
#endif

void sysEmbCommStateUpdate(void)
{
    u8 i;
    ///即EwayEmbSys.rtComm变量的更新
    ////// 更新系统实时与外部的(与PC,与每个Motor,与PSMPSU,与)通信状态变量EwayEmbSys.rtComm
    
    // 每周期更新与PC的通信状态 update Comm Status with PC
    if(EwayEmbTCP.TCP_State == ES_TCPSERVER_CONNECTED)
    {
       EwayEmbSys.rtComm |= (0x00000001<<EMB_COMM_PC_START_BIT);
    }    
 
    // 每周期更新与电机的通信状态 update Comm Status with Motors
    
    ////// 手臂、肩膀、头部
    for(i=0;i<Motor_Num_Arm;i++)
    {
        if((EwayArms.mCommStatus&(0x0001<<i))!=0)
        {
            EwayEmbSys.rtComm |= 0x00000001<<(i+EMB_COMM_ARM_START_BIT);
        }        
    }
    
    for(i=0;i<Motor_Num_Shoulder;i++)
    {
        if((EwayShoulder.mCommStatus&(0x0001<<i))!=0)
        {
            EwayEmbSys.rtComm |= 0x00000001<<(i+EMB_COMM_SHOULDER_START_BIT);
        }
        
        //!< 目前只有1个肩膀电机，因此直接写。
        //
        if((EwayShoulder.mResp.mExeCmRecd[0]>>4) != 0)     //!< 本周期内有发送模式切换指令
        {
            if(l_CompareMotorsExeCmResponseCount(&EwayShoulder.mResp)==ERR_NONE)   //!< 目前只对肩膀的模式更改有记录，后续若有更多的变更指令，需重新定义功能码
            {
                switch((Motor_CtrlMode)(EwayShoulder.mResp.mExeCmRecd[1]))
                {
                    case Ctrl_Mode_Posit:

                        EwayShoulder.mControl.mCtrlMode = Ctrl_Mode_Posit;
            
                        Bsp_printf("Shoulder CtrlMode Changed to Posit,ExeCmRecd[0]:0x%x,ExeCmRecd[1]:%d.",EwayShoulder.mResp.mExeCmRecd[0],EwayShoulder.mResp.mExeCmRecd[1]);
                        
                        break;
                    case Ctrl_Mode_Speed:
                        
                        EwayShoulder.mControl.mCtrlMode = Ctrl_Mode_Speed;
            
                        Bsp_printf("Shoulder CtrlMode Changed to Speed,,ExeCmRecd[0]:0x%x,ExeCmRecd[1]:%d.",EwayShoulder.mResp.mExeCmRecd[0],EwayShoulder.mResp.mExeCmRecd[1]);
                        
                        break;
                        
                    default:
                        
                        break;
                }
            }

            l_ClearMotorResponseCount(&EwayShoulder.mResp);
            
        }
        
    }
    
    for(i=0;i<Motor_Num_Head;i++)
    {
        if((EwayHeads.mCommStatus&(0x0001<<i))!=0)
        {
            EwayEmbSys.rtComm |= 0x00000001<<(i+EMB_COMM_HEAD_START_BIT);
        }        
    }
        
        ////// 爪子
    for(i=0;i<Motor_Num_Gripper;i++)
    {
        if((EwayGrippers.mCommStatus&(0x0001<<i))!=0)
        {
            EwayEmbSys.rtComm |= 0x00000001<<(i+EMB_COMM_GRIP_START_BIT);
        }        
    }
    
    //// 轮子
    for(i=0;i<Motor_Num_Wheel;i++)
    {
        if((EwayWheels.mCommStatus&(0x0001<<i))!=0)
        {
            EwayEmbSys.rtComm |= 0x00000001<<(i+EMB_COMM_WHEEL_START_BIT);
#if Emb_Wheel_Offline_Debug
            EwayWheelCommState[i] = (EwayWheelCommState[i] << 1) + 1;
#endif 
        }
        else
        {
#if Emb_Wheel_Offline_Debug
            EwayWheelCommState[i] = EwayWheelCommState[i] << 1;
#endif
        }
    }

    //检查轮子是否有需要回复的指令发送给轮子电机，且轮子电机是否回复     
    if(((EwayWheels.mResp[0].mExeCmRecd[0]>>4) != 0)||((EwayWheels.mResp[1].mExeCmRecd[0]>>4)!=0))     //!< 本周期内有发送模式切换指令or自由停止启停指令
    {
        if((l_CompareMotorsExeCmResponseCount(&EwayWheels.mResp[0])==ERR_NONE)&&(l_CompareMotorsExeCmResponseCount(&EwayWheels.mResp[1])==ERR_NONE)) 
        {
            switch((Motor_CtrlMode)(EwayWheels.mResp[0].mExeCmRecd[1]))
            {
                case Ctrl_Mode_Posit:

                    EwayWheels.mControl.mCtrlMode = Ctrl_Mode_Posit;
                        
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                    {
                        Bsp_printf("Wheel CtrlMode Chged to Pos,ExeCmRecd[0]x:0x%x,%d.ExeCmRecd[1]x:0x%x,%d.",EwayWheels.mResp[0].mExeCmRecd[0],EwayWheels.mResp[0].mExeCmRecd[1],EwayWheels.mResp[1].mExeCmRecd[0],EwayWheels.mResp[1].mExeCmRecd[1]);
                    }
                        
                    break;
                 case Ctrl_Mode_Speed:
                        
                    EwayWheels.mControl.mCtrlMode = Ctrl_Mode_Speed;

                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                    {
                        Bsp_printf("Wheel CtrlMode Chged to Spd,ExeCmRecd[0]x:0x%x,%d.ExeCmRecd[1]x:0x%x,%d.",EwayWheels.mResp[0].mExeCmRecd[0],EwayWheels.mResp[0].mExeCmRecd[1],EwayWheels.mResp[1].mExeCmRecd[0],EwayWheels.mResp[1].mExeCmRecd[1]);
                    }            
                        
                    break;
                        
                default:
                    break;
            }
        }

        l_ClearMotorResponseCount(&EwayWheels.mResp[0]);
        l_ClearMotorResponseCount(&EwayWheels.mResp[1]);            
    }
        
    // 每周期更新与PSM&PSU的通信状态 update Comm Status with PSM&PSU
    if((EwayEmbSys.sysTimCnt%EMB_READ_PSM_PSU_INTERVAL)==0)
    {
        if(l_ComparePsmPsuResponseCount(&EwayPSM.reRecord)==ERR_NONE)
        {
            EwayPSM.mCommStatus = 1;
        }
            
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_PSM_MASK)||(pDebug->jointSw&JointSwDebugCtrl_PSU_MASK)))
        {
            Bsp_printf("EmbSysRegu 1-1:PSM reRecord,Query:0x%x,ExeCmd:0x%x,PSU reRecord,Query:0x%x,ExeCmd:0x%x.",EwayPSM.reRecord.sQueryRecd,EwayPSM.reRecord.sExeCmRecd,EwayBatteryCharger.reRecord.sQueryRecd,EwayBatteryCharger.reRecord.sExeCmRecd);
        }
    
        l_ClearPsmPsuResponseCount(&EwayPSM.reRecord);
            
        if(EwayPSM.mCommStatus != 0)
        {
            EwayEmbSys.rtComm |= (0x00000001<<EMB_COMM_PSM_START_BIT);
        }
        else
        {
            EwayEmbSys.rtComm &= ~(0x00000001<<EMB_COMM_PSM_START_BIT);
        }
    
        if(l_ComparePsmPsuResponseCount(&EwayBatteryCharger.reRecord)==ERR_NONE)
        {
            EwayBatteryCharger.mCommStatus = 1;
        }
    
        l_ClearPsmPsuResponseCount(&EwayBatteryCharger.reRecord);
            
        if(EwayBatteryCharger.mCommStatus != 0)
        {
            EwayEmbSys.rtComm |= (0x00000001<<EMB_COMM_PSU_START_BIT);
        }
        else
        {
            EwayEmbSys.rtComm &= ~(0x00000001<<EMB_COMM_PSU_START_BIT);
        }
    }

    //Mic 通信状态监控
    if((EwayEmbSys.sysTimCnt%EMB_READ_MIC_INTERVAL)==0)
    {
        if(EwayMic.mCommStatus != 0)        //!< 记录上一通信周期的通信状态
        {
            EwayEmbSys.rtComm |= 0x00000001<<EMB_COMM_MIC_START_BIT;
        }
    }
}

/* --------------------------------------------------------------------------*/
/**
* @name sysEmbCommStateMonitor
* @brief 
* @details 下位机与各个外部设备通信状态监测，包括与上位机、各个电机、PSM&PSU
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void sysEmbCommStateMonitor(void)
{
    u8 i=0;

///////!< 1.将变量EwayEmbSys.tComm中存放的本周期各个电机的通信状态记录到变量EwayEmbSys.Comm中

    //!< 1.1 记录电机的状态
    for( i = EMB_COMM_ARM_START_BIT; i < (SYS_MAX_GENERAL_SERVO_NUMS+Motor_Num_Gripper); i++ )
    {
        EwayEmbSys.Comm.wMotors[i] = EwayEmbSys.Comm.wMotors[i]<<1;

        if((EwayEmbSys.rtComm&(0x00000001<<i)) != 0)
        {
            EwayEmbSys.Comm.wMotors[i] += 1;
        }
    }

    //!< 1.2 记录上位机的通信状态
    EwayEmbSys.Comm.wPc = EwayEmbSys.Comm.wPc<<1;
    if((EwayEmbSys.rtComm&(0x00000001<<EMB_COMM_PC_START_BIT)) != 0)
    {
        EwayEmbSys.Comm.wPc += 1;
    }

    //!< 1.3 记录上一个通信周期(480ms)PSMPSU的通信状态
    if((EwayEmbSys.sysTimCnt%EMB_READ_PSM_PSU_INTERVAL)==0) //!< psm通信状态监控，检查上一次通信周期内，发送的数据包是否都收到了回复。
    {
        EwayEmbSys.Comm.wPsm = EwayEmbSys.Comm.wPsm<<1;
        if((EwayEmbSys.rtComm&(0x00000001<<EMB_COMM_PSM_START_BIT)) != 0)
        {
            EwayEmbSys.Comm.wPsm += 1;
        }

        EwayEmbSys.Comm.wPsu= EwayEmbSys.Comm.wPsu<<1;              //!< 记录PSU的通信状态
        if((EwayEmbSys.rtComm&(0x00000001<<EMB_COMM_PSU_START_BIT)) != 0)
        {
            EwayEmbSys.Comm.wPsu += 1;
        }

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_PSM_MASK)||(pDebug->jointSw&JointSwDebugCtrl_PSU_MASK)))
        {
            Bsp_printf("EmbSysRegu 1-2:PSMPSU Status:rtComm:0x%x,wPsm:0x%x,wPsu:0x%x.",EwayEmbSys.rtComm,EwayEmbSys.Comm.wPsm,EwayEmbSys.Comm.wPsu);
        }        
    }

    //!< 1.4 记录Mic的通信状态
    if((EwayEmbSys.sysTimCnt%EMB_READ_MIC_INTERVAL)==0)
    {
        EwayEmbSys.Comm.wMic = EwayEmbSys.Comm.wMic<<1;
        if((EwayEmbSys.rtComm&(0x00000001<<EMB_COMM_MIC_START_BIT)) != 0)
        {
            EwayEmbSys.Comm.wMic += 1;
        }

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
        {
            Bsp_printf("EmbSysRegu 1-2:Mic Status:rtComm:0x%x,wMic:0x%x.",EwayEmbSys.rtComm,EwayEmbSys.Comm.wMic);
        } 
    }
}



void sysEmbCommStateUpdateAndMonitor(void)
{
    sysEmbCommStateUpdate();//!< 检查手臂、肩膀、头部电机,与上位机、PSM、PSU的通信状态

    sysEmbCommStateMonitor();//!< 更新系统的通信状态
}


void sysEmbMotorsCommSupervise(void)
{
    //u8 i;
    //u8 slvNums=0;
	  //u8 slvID[Motor_Num_Wheel];
    //u8 tmpDat[10]={0};
    //s8 res;
    
    /*
    //!< 检查最近3个通信周期的手臂电机通信是否正常
    for(i=0;i<Motor_Num_Arm;i++)
    {
        if((EwayEmbSys.Comm.wMotors[i]&EMB_COMM_ERR_BITS_3_TIMES)==0x01)//!< 与Motors通信恢复正常
        {
            //!< 清除lastPosCmd的available参数，以应对Emb与PC网络正常，电机停在某处，然后电机掉电重新上电后，PC仍无指令，当PC网络异常时，不下发异常的指令。
            if(i<Motor_Num_ArmLeft)
            {
                EmbClrLastPosCmdAvailableFlag(EwayArms.mControl.pmlstPos,i);
            }
            else
            {
                EmbClrLastPosCmdAvailableFlag(EwayArms.mControl.pmlstPos,(i+Motor_Num_ArmLeft));
            }
            
            Bsp_printf("armMotorComm(%d) resume, Clr lastPosCmd avail flag",i);
        }
    }

    
    //!< 检查最近3个通信周期的肩膀电机通信是否正常
    if((EwayEmbSys.Comm.wMotors[12]&EMB_COMM_ERR_BITS_3_TIMES)==0x01)//!< 与Motor通信恢复正常
    {
        EmbClrLastPosCmdAvailableFlag(EwayShoulder.mControl.pmlstPos,0);
        
        Bsp_printf("ShoulderMotorComm resume, Clr lastPosCmd avail flag.");
    }

    //!< 检查最近3个通信周期的头部电机通信是否正常    
    for(i=0;i<Motor_Num_Head;i++)
    {
        if((EwayEmbSys.Comm.wMotors[i+(Motor_Num_Arm+Motor_Num_Shoulder)]&EMB_COMM_ERR_BITS_3_TIMES)==0x01)//!< 与Motors通信恢复正常
        {
            //!< 清除lastPosCmd的available参数，以应对Emb与PC网络正常，电机停在某处，然后电机掉电重新上电后，PC仍无指令，当PC网络异常时，不下发异常的指令。
            EmbClrLastPosCmdAvailableFlag(EwayHeads.mControl.pmlstPos,i);
                    
            Bsp_printf("HeadMotorComm(%d) resume, Clr lastPosCmd avail flag",i);
        }
    }*/

    //!< 检查最近3个通信周期的轮子电机通信是否正常   
    /*
    for(i=0;i<Motor_Num_Wheel;i++)
    {
        if((EwayEmbSys.Comm.wMotors[i+15]&0xFF)==0x03)//!< 与Motors通信恢复正常
        {
            //!< 清除lastPosCmd的available参数，以应对Emb与PC网络正常，电机停在某处，然后电机掉电重新上电后，PC仍无指令，当PC网络异常时，不下发异常的指令。
            //EmbClrLastPosCmdAvailableFlag(EwayHeads.mControl.pmlstPos,i);
                    
            //Bsp_printf("HeadMotorComm(%d) resume, Clr lastPosCmd avail flag",i);
            //slvID[slvNums] = i;

            slvNums += 1;
        }
    }
    

    if(slvNums>0)       //!< 若轮子有刚上电，则需要对轮子发送一个自由停止使能的指令(向0x51reg写2)
    {
        res = sysSendFreeStopToWheel(1);         //!< 

        if(res==ERR_NONE)
        {
            EwayWheels.MotorStop = StopMode_FreeOn;
            
            Bsp_printf("wheelsMotorComm resume,send(0x51-FreeStop On) Suc!");
	
        }
        else
        {            
            EwayWheels.MotorStop = StopMode_FreeOff;
            
            Bsp_printf("wheelsMotorComm resume,send(0x51-FreeStop On) failed!");
        }        
    }*/

}


void sysEmbCheckWheelMotorsStatus(void)
{
    s8 res;
    u8 i;
    
    //!< 周期检查Ready位是否均已经置位
    if((EwayEmbSys.sysTimCnt%EMB_READ_WHEEL_MOTORS_READY_REG_INTERVAL)==0)      //!< 每隔240ms查询一次
    {
        if((EwayWheels.mRdyStatus&0x0003)!=0x0003)      //!< 2个轮子电机需要同时准备好
        {
            //!< 发送查询包
            res = sysGeneralMotorBroadcastRead(Emb_StartID_Wheel,Motor_Num_Wheel,EwayMotorReg_WheelReady,1,ArmSvdelayTm,EWAYBOT_WHEEL);
            if( ERR_NONE != res )
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    res = Bsp_printf("3-2 Send CheckWheelMotorsStatus failed,rt:%d.",res);
                }
            }
            else
            {
                //if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK)&&\
                    //()&&())
                //{
                    res = Bsp_printf("Periodly:Send CheckWheelMotorsStatus succ!");
                //}
            }
        }
    }

    //!< 检测轮子电机是否有通信不正常
    for(i=0;i<Motor_Num_Wheel;i++)
    {
        if((EwayEmbSys.Comm.wMotors[i+EMB_COMM_WHEEL_START_BIT]&0x0F)==0x80)//!< 与Motors通信断开
        {
            EwayWheels.mRdyStatus = 0;            

            Bsp_printf("Wheel(%d) Comm is break,Clr Rdy Status.",(i+1));
        }
    }    

}

//!< 检查当轮子通信异常时，需要清除PC Cmd buffer & 其他的指令buffer
void sysEmbCheckWheelMotorsCommuStatus(void)
{
    u8 tmp0,tmp1;
    UBaseType_t nums;
    BaseType_t re;
    u16 i;
    s8 res;

    tmp0 = EwayEmbSys.Comm.wMotors[EMB_COMM_WHEEL_START_BIT]&EMB_COMM_ERR_BITS_3_TIMES;
    tmp1 = EwayEmbSys.Comm.wMotors[EMB_COMM_WHEEL_START_BIT+1]&EMB_COMM_ERR_BITS_3_TIMES;

    if((tmp0==0)||(tmp1==0))    //!< 有轮子掉线
    {
        nums = uxQueueMessagesWaiting(PCToWheelsQueHdl);
        if(nums > 0)
        {
            re = xQueueReset(PCToWheelsQueHdl);
            if(re != pdPASS)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    Bsp_printf("sysEmbCheckWhlMotsCommStatus Clr Queue PCToWheelsQueHdl failed.");
                }
            }
        }

        //!< 清各个电机的指令待执行缓存
        for(i=0;i<Motor_Num_Wheel;i++)
        {
            res = g_ClearPcCmdtoNodeCmdBuff(EwayWheels.mControl.pToMotorCmd,i);
            if(res!=ERR_NONE)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    Bsp_printf("sysEmbCheckWhlMotsCommStatus Clr Wheel(%d) ToMotorCmd Cmd Buf failed,rt:%d.",i,res);
                }
            }
        }       
    }
}



s8 sysSendStopCmdToWheel(u8 en)
{
    u8 tmpDat[20]={0};
    s8 res;

    if((en!=EwayMotor_Stop_Off)&&(en!=EwayMotor_Stop_SlowDown)&&(en!=EwayMotor_Stop_UrgentStop)&&(en!=EwayMotor_Stop_FreeStop))
    {
        return ERR_INPUT_PARAMETERS;
    }    
        
    tmpDat[0] = en;                  //!< reg0x51:1-减速停止,2-紧急制动,3-自由停止
    tmpDat[1] = en;
           
    res = sysGeneralMotorBroadcastWrite(Emb_StartID_Wheel,Motor_Num_Wheel,EwayMotor_Stop,1,tmpDat,EWAYBOT_WHEEL);

    return res;
}


void sysPutWheelMotorsIntoFreeStopMode(void)
{
    s8 res;
    
    res = sysSendStopCmdToWheel(EwayMotor_Stop_FreeStop);
    
    if(res==ERR_NONE)
    {
        EwayWheels.MotorStop = StopMode_FreeOn;
                
        Bsp_printf("Put Wheel Motors into FreeStop,send(0x51-FreeStop On) Suc!"); 
    }
    else
    {            
        EwayWheels.MotorStop = StopMode_Off;
                
        Bsp_printf("Put Wheel Motors into FreeStop,send(0x51-FreeStop On) failed!");
    }   
}


void sysPutWheelMotorsIntoStopOffMode(void)
{
    s8 res;
    
    res = sysSendStopCmdToWheel(EwayMotor_Stop_Off);
    
    if(res==ERR_NONE)
    {
        EwayWheels.MotorStop = StopMode_Off;
                
        Bsp_printf("Put Wheel Motors into StopOff Mode,send(0x51-FreeStop Off) Suc!");    
    }
    else
    {            
        EwayWheels.MotorStop = StopMode_FreeOn;
                
        Bsp_printf("Put Wheel Motors into StopOff Mode,send(0x51-FreeStop Off) failed!");
    }    
}



void sysCyclicalCheckEmbMotorsCfgRegs(void)
{
    u8 i;
    u8 status[2];
    u32 tmp[2];
    EwayMotor_Device_Type chkdev[5]={EWAYBOT_ARMS_LEFT,EWAYBOT_ARMS_RIGHT,EWAYBOT_SHOULDER,EWAYBOT_HEAD,EWAYBOT_WHEEL};
    NodeCtrlModule* devCtrlMd[5]={&EwayArms.mControl,&EwayArms.mControl,&EwayShoulder.mControl,&EwayHeads.mControl,&EwayWheels.mControl};
    
    //!< 每隔约4.8s钟检查一次

    //!< 肩膀电机处理
    status[0] = EwayEmbSys.genmCommSta.mCfgInitSta&0x00000003;     //!< 获取肩膀cfgRegs的初始化状态
    tmp[0] = EwayEmbSys.genmCommSta.mCfgRegCommSta&0x00001000; 

    switch(status[0])
    {
        case 0:
                if(tmp[0] != 0)   //!< 上周期有读回cfgReg
                {
                    sysEmbCheckMotorAppCtrlModeInConfigRegs(&EwayShoulder.mControl,0,&MotorCfgReg[12][0]);

                    EwayEmbSys.genmCommSta.mCfgInitSta &= 0xFFFFFFFC;    //!< 更新状态

                    EwayEmbSys.genmCommSta.mCfgInitSta |= 0x00000001;

                    //!< 检查肩膀电机的cfgRegs是否有不符合要求的，若有，则发送广播写的指令
                    l_CheckMotorsCfgRegComparedResultAndPackSettingData(chkdev[2],devCtrlMd[2],EwayEmbSys.genmCommSta.mCfgRegCommSta);                    
                }
                
                EwayEmbSys.genmCommSta.mCfgRegCommSta &= ~(0x00001000);     //!< 清读cfgReg状态，并继续每周期读cfgReg
                
                g_EmbSendQueryPacketToReadMotorCfgRegisters(chkdev[2]);
                
            break;
        case 1:
                if((EwayEmbSys.sysTimCnt%EMB_READ_MotorsCfgReg_INTERVAL)==2)        //!< Emb已在上电后初始化过肩膀电机的状态，只需要周期检查就ok
                {
                    if(tmp[0] != 0)
                    {
                        sysEmbCheckMotorAppCtrlModeInConfigRegs(&EwayShoulder.mControl,0,&MotorCfgReg[12][0]);
                        
                        l_CheckMotorsCfgRegComparedResultAndPackSettingData(chkdev[2],devCtrlMd[2],EwayEmbSys.genmCommSta.mCfgRegCommSta);
                    }
                    else
                    {
                        EwayEmbSys.genmCommSta.mCfgInitSta &= ~(0x00000003);        //!< 若周期查询中有未读回cfgReg的情况，则将状态重新转到0进行周期读取
                    }

                    EwayEmbSys.genmCommSta.mCfgRegCommSta &= ~(0x00001000);     //!< 清读cfgReg状态，并继续每周期读cfgReg
                
                    g_EmbSendQueryPacketToReadMotorCfgRegisters(chkdev[2]);
                }
            break;
        default:
            break;
    }

    //!< 轮子电机处理，以id=1轮子为准，status=0时
    for(i=0;i<Motor_Num_Wheel;i++)
    {
        status[i] = EwayEmbSys.genmCommSta.mCfgInitSta&(0x00000003<<((i+1)<<1));     //!< 获取轮子cfgRegs的初始化状态
        tmp[i] = EwayEmbSys.genmCommSta.mCfgRegCommSta&(0x00000001<<(i+EMB_COMM_WHEEL_START_BIT)); 
    }

    if((status[0]!=0)&&(status[1]!=0))          //!< 2个轮子电机都已经读回过cfgReg，进入了定期(4.8s)查询阶段
    {
        if((EwayEmbSys.sysTimCnt%EMB_READ_MotorsCfgReg_INTERVAL)==2)
        {
            for(i=0;i<Motor_Num_Wheel;i++)
            {
                if(tmp[i]==0)                   //!< 若周期查询中有未读回cfgReg的情况，则将状态重新转到0进行周期读取
                {
                    EwayEmbSys.genmCommSta.mCfgInitSta &= (~(0x00000003<<((i+1)<<1)));
                }
                else                            //!< 若有收到，则检查AppCtrl是否一致
                {
                    sysEmbCheckMotorAppCtrlModeInConfigRegs(&EwayWheels.mControl,i,&MotorCfgReg[(i+EMB_COMM_WHEEL_START_BIT)][0]);
                }
            }
        
            if((tmp[0]!=0)||(tmp[1]!=0))        //!< 若有通信正常的电机，则检查是否需要发送广播写指令来更改轮子电机的cfgRegs
            {
                l_CheckMotorsCfgRegComparedResultAndPackSettingData(chkdev[4],devCtrlMd[4],EwayEmbSys.genmCommSta.mCfgRegCommSta);
            }
            
            EwayEmbSys.genmCommSta.mCfgRegCommSta &= ~(0x00018000);     //!< 清读cfgReg状态，并继续每周期读cfgReg
                
            g_EmbSendQueryPacketToReadMotorCfgRegisters(chkdev[4]);

        }
    }
    else                                        //!< 2个轮子电机并没有全部读回过cfgReg，仍旧进行每周期查询操作
    {
        for(i=0;i<Motor_Num_Wheel;i++)
        {
            if(tmp[i]==0)
            {
                continue;
            }

            sysEmbCheckMotorAppCtrlModeInConfigRegs(&EwayWheels.mControl,i,&MotorCfgReg[(i+EMB_COMM_WHEEL_START_BIT)][0]);

            EwayEmbSys.genmCommSta.mCfgInitSta &= (~(0x00000003<<((i+1)<<1)));    //!< 更新状态

            EwayEmbSys.genmCommSta.mCfgInitSta |= (0x00000001<<((i+1)<<1));
        }

        if((tmp[0]!=0)||(tmp[1]!=0))
        {
            //!< 检查肩膀电机的cfgRegs是否有不符合要求的，若有，则发送广播写的指令
            l_CheckMotorsCfgRegComparedResultAndPackSettingData(chkdev[4],devCtrlMd[4],EwayEmbSys.genmCommSta.mCfgRegCommSta);  
        }

        EwayEmbSys.genmCommSta.mCfgRegCommSta &= ~(0x00018000);     //!< 清读cfgReg状态，并继续每周期读cfgReg
                
        g_EmbSendQueryPacketToReadMotorCfgRegisters(chkdev[4]);
        
    }

    //!< 检查手臂电机、头部电机的cfgRegs
    if((EwayEmbSys.sysTimCnt%EMB_READ_MotorsCfgReg_INTERVAL)==2)
    {
        //!< 检查是否收到上一周期电机回复的配置寄存器
            //!< 检查手臂电机的配置寄存器        

        //Bsp_printf("Check cfg regs,Comm:0x%x",EwayEmbSys.genmCommSta.mCfgRegCommSta);
        
        for(i=0;i<5;i++)
        {
            if((i!=2)&&(i!=4))        //!< 除去肩膀电机与轮子电机
            {
                l_CheckMotorsCfgRegComparedResultAndPackSettingData(chkdev[i],devCtrlMd[i],EwayEmbSys.genmCommSta.mCfgRegCommSta);    
            }
            else
            {
                continue;
            }
        }        
    }

    if((EwayEmbSys.sysTimCnt%EMB_READ_MotorsCfgReg_INTERVAL)==0)
    {
        EwayEmbSys.genmCommSta.mCfgRegCommSta &= (0x00000001<<Motor_Num_Arm) + (0x03<<Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head);      //!< 清除接收包的状态，对肩膀与轮子电机的此处不做处理
        
        //!< 向各个电机发送查询配置寄存器的包
        for(i=0;i<5;i++)
        {
            if((i!=2)&&(i!=4))        //!< 除去肩膀电机与轮子电机
            {
                g_EmbSendQueryPacketToReadMotorCfgRegisters(chkdev[i]);
            }
            else
            {
                continue;
            }
        }
    }

}


/*
void sysCyclicalCheckEmbMotorsCfgRegs(void)
{
    u8 i;
    EwayMotor_Device_Type chkdev[5]={EWAYBOT_ARMS_LEFT,EWAYBOT_ARMS_RIGHT,EWAYBOT_SHOULDER,EWAYBOT_HEAD,EWAYBOT_WHEEL};
    NodeCtrlModule* devCtrlMd[5]={&EwayArms.mControl,&EwayArms.mControl,&EwayShoulder.mControl,&EwayHeads.mControl,&EwayWheels.mControl};
    
    //!< 每隔约10s钟检查一次
    if((EwayEmbSys.sysTimCnt%EMB_READ_MotorsCfgReg_INTERVAL)==2)
    {
        //!< 检查是否收到上一周期电机回复的配置寄存器
            //!< 检查手臂电机的配置寄存器        

        //Bsp_printf("Check cfg regs,Comm:0x%x",EwayEmbSys.genmCommSta.mCfgRegCommSta);
        
        for(i=0;i<5;i++)
        {
            l_CheckMotorsCfgRegComparedResultAndPackSettingData(chkdev[i],devCtrlMd[i],EwayEmbSys.genmCommSta.mCfgRegCommSta);            
        }        
    }

    if((EwayEmbSys.sysTimCnt%EMB_READ_MotorsCfgReg_INTERVAL)==0)
    {
        EwayEmbSys.genmCommSta.mCfgRegCommSta = 0;      //!< 清除接收包的状态
        
        //!< 向各个电机发送查询配置寄存器的包
        for(i=0;i<5;i++)
        {
            g_EmbSendQueryPacketToReadMotorCfgRegisters(chkdev[i]);
        }
    }

}*/
/* --------------------------------------------------------------------------*/
/**
* @name sysReadAndCheckEmbMotorsCfgRegs
* @brief 
* @details 按顺序单读各个Motor的配置寄存器并比较寄存器是否与默认值相匹配
*           若不匹配，则需要更改寄存器的值。
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysInitReadAndCheckEmbMotorsCfgRegs(void)  //!< 目前将SingleRead的结果全部放入全局变量MotorCfgReg[][]中，以后根据需要再更改
{
    u8 dat[10]={7,6};//{0,0x2D,0x3C,7,0x50,16,0x78,21,0xB4,12};     //!< 目前先只读reg:[7-12]共6个寄存器
    s8 res=ERR_NONE;
    u8 i;
    //osStatus tSta = osOK; 

    EwayEmbSys.genmCommSta.mCfgRegCommSta = 0;      //!< 清接收标志位

    //!< 读寄存器，左手臂
    for(i=0;i<Motor_Num_ArmLeft;i++)
    {
        res = g_GeneralMotorSingleRead((Emb_StartID_ArmL+i),1,dat,EWAYBOT_ARMS_LEFT);
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("1-1 init g_GeneralMotorSingleRead(EWAYBOT_ARMS_LEFT) failed.");
            }            
        }
    
        osDelay(20);
            
        res = sysGeneralMotorRecvRegsProcess(EWAYBOT_ARMS_LEFT);
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("1-1 init sysGeneralMotorRecvRegsProcess(EWAYBOT_ARMS_LEFT) failed.");
            }
        }
    }   

    //!< 读寄存器，右手臂
    for(i=0;i<Motor_Num_ArmRight;i++)
    {
        res = g_GeneralMotorSingleRead((Emb_StartID_ArmR+i),1,dat,EWAYBOT_ARMS_RIGHT);
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("1-1 init g_GeneralMotorSingleRead(EWAYBOT_ARMS_RIGHT) failed.");
            }
        }
            
        osDelay(20);        

        res = sysGeneralMotorRecvRegsProcess(EWAYBOT_ARMS_RIGHT);
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("1-1 init sysGeneralMotorRecvRegsProcess(EWAYBOT_ARMS_RIGHT) failed.");
            }
        }
    }

    //!< 读寄存器，肩膀和头
    for(i=0;i<(Motor_Num_Shoulder+Motor_Num_Head);i++)
    {
        res = g_GeneralMotorSingleRead((Emb_StartID_Shoulder+i),1,dat,EWAYBOT_SHOULDER);
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("1-1 init g_GeneralMotorSingleRead(EWAYBOT_SHOULDER) failed.");
            }
        }
        
        osDelay(20);       
        
        res = sysGeneralMotorRecvRegsProcess(EWAYBOT_SHOULDER);
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("1-1 init sysGeneralMotorRecvRegsProcess(EWAYBOT_ARMS_RIGHT) failed.");
            }
        }
    
    }

    //!< 读寄存器，轮子
    for(i=0;i<(Motor_Num_Wheel);i++)
    {
        res = g_GeneralMotorSingleRead((Emb_StartID_Wheel+i),1,dat,EWAYBOT_WHEEL);
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("1-1 init g_GeneralMotorSingleRead(EWAYBOT_WHEEL) failed.");
            }
        }
        
        osDelay(20);        
        
        res = sysGeneralMotorRecvRegsProcess(EWAYBOT_WHEEL);
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("1-1 init sysGeneralMotorRecvRegsProcess(EWAYBOT_WHEEL) failed.");
            }
        }
    
    }   

    //!< 读寄存器，爪子


    //!< 检查各个部分的配置寄存器
        //!< 检查手臂电机的配置寄存器
    for(i=0;i<Motor_Num_Arm;i++)
    {        
        if((EwayEmbSys.genmCommSta.mCfgRegCommSta&((0x00000001<<i)))!=0)
        {            
            res = sysEmbCheckMotorConfigRegs(&EwayArms.mControl,i,&MotorCfgReg[i][0]);
        }
        else
        {
            res = ERR_PROCESS_FAILED;
        }
        
        if(res!=ERR_NONE)
        {
            EwayEmbSys.genmCommSta.mCfgCompared |= (0x00000001<<i);
        }
    }

        //!< 检查肩膀电机的配置寄存器
    if((EwayEmbSys.genmCommSta.mCfgRegCommSta&(0x00000001<<12))!=0)
    {            
        res = sysEmbCheckMotorConfigRegs(&EwayShoulder.mControl,0,&MotorCfgReg[Emb_StartID_Shoulder-1][0]);
    }
    else
    {
        res = ERR_PROCESS_FAILED;
    }

    if(res!=ERR_NONE)
    {
        EwayEmbSys.genmCommSta.mCfgCompared |= (0x00000001<<(Emb_StartID_Shoulder-1));
    } 

        //!< 检查头部电机的配置寄存器
    for(i=0;i<Motor_Num_Head;i++)
    {        
        if((EwayEmbSys.genmCommSta.mCfgRegCommSta&(0x00000001<<(Emb_StartID_Head-1+i)))!=0)
        {            
            res = sysEmbCheckMotorConfigRegs(&EwayHeads.mControl,i,&MotorCfgReg[(Emb_StartID_Head-1+i)][0]);
        }
        else
        {
            res = ERR_PROCESS_FAILED;
        }    

        if(res!=ERR_NONE)
        {
            EwayEmbSys.genmCommSta.mCfgCompared |= (0x00000001<<(Emb_StartID_Head-1+i));
        }        
    }

        //!< 检查轮子电机的配置寄存器
    for(i=0;i<Motor_Num_Wheel;i++)
    {        
        if((EwayEmbSys.genmCommSta.mCfgRegCommSta&(0x00000001<<(15+i)))!=0)
        {            
            res = sysEmbCheckMotorConfigRegs(&EwayWheels.mControl,i,&MotorCfgReg[(i+15)][0]);
        }
        else
        {
            res = ERR_PROCESS_FAILED;
        }  
    
        if(res!=ERR_NONE)
        {
            EwayEmbSys.genmCommSta.mCfgCompared |= (0x00000001<<(15+i));
        }        
    }
    
    return ERR_NONE;
}

/* --------------------------------------------------------------------------*/
/**
* @name sysEmbCheckMotorConfigRegs
* @brief 
* @details 根据检查各个电机配置寄存器的结果，发送广播包or打印错误信息
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 g_EmbSendQueryPacketToReadMotorCfgRegisters(EwayMotor_Device_Type devType)
{
    s8 res=ERR_NONE;
    u8 startID,devNum;

    switch(devType)
    {
        case EWAYBOT_ARMS_LEFT:
                startID = Emb_StartID_ArmL;     //!< 左臂 bit0.1.2.3.4.5
                devNum = Motor_Num_ArmLeft;
                
            break;

        case EWAYBOT_ARMS_RIGHT:
                startID = Emb_StartID_ArmR;     //!< 右臂 bit6.7.8.9.10.11
                devNum = Motor_Num_ArmRight;
                
            break;

        case EWAYBOT_SHOULDER:
                startID = Emb_StartID_Shoulder; //!< 肩膀 bit12
                devNum = Motor_Num_Shoulder;
                
            break;

        case EWAYBOT_HEAD:
                startID = Emb_StartID_Head;     //!< 头部 bit13.14
                devNum = Motor_Num_Head;
                
            break;

        case EWAYBOT_WHEEL:
                startID = Emb_StartID_Wheel;     //!< 轮子 bit15.16
                devNum = Motor_Num_Wheel;
                
            break;

        default:                                //!< 对于其他电机，无此功能
                res = ERR_INPUT_PARAMETERS;
            break;

    }

    if(res==ERR_NONE)
    {
        res = sysGeneralMotorBroadcastRead(startID,devNum,EMB_SET_GENERAL_MOTOR_CFG_START_REG,EMB_READ_GENERAL_MOTOR_CFG_REG_NUM,ArmSvdelayTm,devType);

        //Bsp_printf("Send read Motors cfg registers pkt.startID:%d,devNum:%d.\r\n",startID,devNum);
    }

    return res;  

}

/* --------------------------------------------------------------------------*/
/**
* @name sysEmbCheckMotorConfigRegs
* @brief 
* @details 根据检查各个电机配置寄存器的结果，发送广播包or打印错误信息
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 l_CheckMotorsCfgRegComparedResultAndPackSettingData(EwayMotor_Device_Type devType,NodeCtrlModule* pMoCtrl,u32 mReadCfgRegCommSta)
{
    u8 i,sBit,ssBit,devNum,slvNums=0;
	u8 slvID[6]={0};
    s8 res=ERR_NONE;
    u8 sndDat[30]={0};						//!< motorsNums*regNums 6*3  18byte
    u32 MskBit=0,mPrint=0,CfgRegCompRes=0;

    switch(devType)
    {
        case EWAYBOT_ARMS_LEFT:
                MskBit = 0x0000003F;     //!< 左臂 bit0.1.2.3.4.5
                devNum = Motor_Num_ArmLeft;
                sBit = 0;
                ssBit = 0;
            break;

        case EWAYBOT_ARMS_RIGHT:
                MskBit = 0x00000FC0;     //!< 右臂 bit6.7.8.9.10.11
                devNum = Motor_Num_ArmRight;
                sBit = Motor_Num_ArmLeft;
                ssBit = 6;                    //!< 因左臂与右臂共用一个NodeCtrlModule，所以NodeCtrlModule中的Idx的起点不同
            break;

        case EWAYBOT_SHOULDER:
                MskBit = 0x00001000;     //!< 肩膀 bit12
                devNum = Motor_Num_Shoulder;
                sBit = Motor_Num_Arm;
                ssBit = 0;
            break;

        case EWAYBOT_HEAD:
                MskBit = 0x00006000;     //!< 头部 bit13.14
                devNum = Motor_Num_Head;
                sBit = Motor_Num_Arm+Motor_Num_Shoulder;
                ssBit = 0;
            break;

        case EWAYBOT_WHEEL:
                MskBit = 0x00018000;     //!< 轮子 bit15.16
                devNum = Motor_Num_Wheel;
                sBit = Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head;
                ssBit = 0;
            break;

        default:                                //!< 对于其他电机，无此功能
                res = ERR_INPUT_PARAMETERS;
            break;

    }

    if(res != ERR_NONE)
    {
        return res;
    }
        
    for(i=0;i<devNum;i++)
    {        
        if((mReadCfgRegCommSta&(0x00000001<<(i+sBit)))!=0)
        {            
            res = sysEmbCheckMotorConfigRegs(pMoCtrl,(i+ssBit),&MotorCfgReg[(i+sBit)][0]);
        }
        else
        {
            res = ERR_DATA_NOT_RDY;         //!< 表示未收到电机的回复包，则继续广播读配置寄存器，不需要广播写配置寄存器 与ERR_NONE有相同的结果
        }
        
        if(res == ERR_RegsCompare_0)       //!< 收到了电机的回复包，但是比较参数后，参数与设置的不正确，且是可设置的reg(除了减速比)则需要发广播包重新配置
        {
            CfgRegCompRes |= (0x00000001<<(i+sBit));                //!< 组包以待后续组包发广播写指令数据包
        }
        else if(res == ERR_RegsCompare_1)
        {                
            mPrint |= (0x00000001<<(i+sBit));
        }
    }

    if((CfgRegCompRes&MskBit)!=0)
    {
        slvNums = 0;
            
        for(i=0;i<devNum;i++)
        {
            if((CfgRegCompRes&(0x00000001<<(i+sBit)))!=0)
            {
                if(devType != EWAYBOT_WHEEL)
                {
                    slvID[slvNums] = i+sBit+1;
                }
                else
                {
                    slvID[slvNums] = i+1;
                }                

                res = l_FillMotorSendingCfgRegsBuffer(pMoCtrl,&sndDat[(slvNums*EMB_SET_GENERAL_MOTOR_CFG_REG_NUM)]);

                if(res == ERR_NONE)
                {
                    slvNums += 1;
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        Bsp_printf("devTy:%d l_FillMotorSendingCfgRegsBuffer() failed,id:%d,rt:%d",devType,i,res);
                    }
                }                    
            }
        }

        if(slvNums>0)
        {
            res = sysGeneralMotorDiscontinuousBroadcastWrite(slvID,slvNums,EwayMotor_AppMode,EMB_SET_GENERAL_MOTOR_CFG_REG_NUM,sndDat,devType);
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&((pDebug->jointSw&0x1F)!=0))
            {
                Bsp_printf("Send BrodcstWr() to devTy:%d to Set CfgRegs,id0:%d,Nums:%d",devType,slvID[0],slvNums);
            }

            if(devType == EWAYBOT_SHOULDER)
            {
                Bsp_printf("Send BrodcstWr() to Shoulder(%d,%d) Set CfgRegs:0x%02x,0x%02x,0x%02x,0x%02x.",slvID[0],slvNums,sndDat[0],sndDat[1],sndDat[2],sndDat[3]);
            }
        }            
    }

    if((mPrint&MskBit)!=0)           //!< 对于减速比等寄存器不适宜直接更改其数值，因此将周期打印此错误信息
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&((pDebug->jointSw&0x1F)!=0))
        {
            Bsp_printf("Motors of devTy:%d,some CfgRegs value is wrong.Motors:0x%x.",devType,mPrint);
        }
    }

    return ERR_NONE;
}


/* --------------------------------------------------------------------------*/
/**
* @name l_FillMotorSendingCfgRegsBuffer
* @brief 
* @details 根据系统存储的电机配置，按照cfgReg[7-9]顺序填写寄存器数值
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 l_FillMotorSendingCfgRegsBuffer(NodeCtrlModule* pmCtrl,u8* pDat)
{

    if((pmCtrl==NULL)||(pDat==NULL))
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(pmCtrl->mAppMode==App_Mode_Joint)            // reg 7
    {
        pDat[0] = 0;
    }
    else if(pmCtrl->mAppMode==App_Mode_Wheel)
    {
        pDat[0] = 1;
    }
    else
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(pmCtrl->mCtrlMode == Ctrl_Mode_Posit)                //reg 8 目前只支持位置or速度
    {
        pDat[1] = 1;
    }
    else if(pmCtrl->mCtrlMode == Ctrl_Mode_Speed)
    {
        pDat[1] = 0;
    }
    else
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(pmCtrl->mPosClsedLoopCtrlMode == Pos_ClosedLoop_Absolute)
    {
        pDat[2] = 0;
    }
    else if(pmCtrl->mPosClsedLoopCtrlMode == Pos_ClosedLoop_Relative)
    {
        pDat[2] = 1;
    }
    else
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    return ERR_NONE;
}
/* --------------------------------------------------------------------------*/
/**
* @name sysEmbCheckMotorConfigRegs
* @brief 
* @details 
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysEmbCheckMotorConfigRegs(NodeCtrlModule* pmCtrl,u8 mIdx,u8* pReg)
{
    Motor_ApplicationMode appMd;
    Motor_CtrlMode ctlMd;
    Motor_PosClosedLoopCtrlMode PosLoopCtrlMd;
    float mReRt;

    if((pmCtrl==NULL)||(pReg==NULL))
    {
        return ERR_INPUT_PARAMETERS;
    }

    //!< 检查应用模式
    if((*pReg) == 0)                        //!< reg 7
    {
        appMd = App_Mode_Joint;
    }
    else if((*pReg) == 1)
    {
        appMd = App_Mode_Wheel;
    }
    else
    {
        return ERR_RegsCompare_0;
    }
    
    if(pmCtrl->mAppMode != appMd)           //!< 可更改寄存器
    {
        return ERR_RegsCompare_0;
    }
    
    //!< 检查控制模式
    if(pReg[1] == 0)                        //!< reg 8,电机控制模式目前只用到了速度和位置模式，目前只检查这两种类型 
    {
        ctlMd = Ctrl_Mode_Speed;
    }
    else if(pReg[1] == 1)
    {
        ctlMd = Ctrl_Mode_Posit;
    }
    else
    {
        return ERR_RegsCompare_0;           //!< 可更改寄存器
    }

    if(pmCtrl->mCtrlMode != ctlMd)
    {
        return ERR_RegsCompare_0;           //!< 可更改寄存器
    }
    
    //!< 检查位置闭环控制类型           //!< 车轮模式下才比较位置闭环控制类型
    if(pmCtrl->mAppMode ==  App_Mode_Wheel)
    {
        if(pReg[2]==0)                                          //!< reg 9
        {
            PosLoopCtrlMd = Pos_ClosedLoop_Absolute;
        }
        else if(pReg[2]==1)
        {
            PosLoopCtrlMd = Pos_ClosedLoop_Relative;            
        }
        else
        {
            return ERR_RegsCompare_0;           //!< 可更改寄存器
        }

        if(pmCtrl->mPosClsedLoopCtrlMode != PosLoopCtrlMd)
        {
            return ERR_RegsCompare_0;           //!< 可更改寄存器
        }
    }   
    
    //!< 检查减速比
    mReRt = pReg[4] + (pReg[5]<<8);

    if(((*(pmCtrl->pMotorRatio+mIdx))*100)!=mReRt)
    {
        return ERR_RegsCompare_1;               //!< 不可更改寄存器
    }

    return ERR_NONE;
    
}

/* --------------------------------------------------------------------------*/
/**
* @name sysEmbCheckShoulderOrWheelMotorConfigRegs
* @brief 
* @details 主要是检查肩膀or轮子电机的电机控制模式寄存器，
*          (1)当电机为位置or速度模式且与下位机内保存的全局变量不同，则更新下位机
*             内保存的全局变量
*          (2)当电机为其他模式时，则将下位机保存的全局变量变为默认的位置模式，待
*             后续根据下位机保存的全局变量给电机发送修改寄存器的指令
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysEmbCheckMotorAppCtrlModeInConfigRegs(NodeCtrlModule* pmCtrl,u8 mIdx,u8* pReg)
{
    Motor_CtrlMode ctlMd;

    if((pmCtrl==NULL)||(pReg==NULL))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    //!< 检查控制模式
    if(pReg[1] == 0)                        //!< reg 8,电机控制模式目前只用到了速度和位置模式，目前只检查这两种类型 
    {
        ctlMd = Ctrl_Mode_Speed;
    }
    else if(pReg[1] == 1)
    {
        ctlMd = Ctrl_Mode_Posit;        
    }
    else
    {
        ctlMd = Ctrl_Mode_Posit;            //!< 目前肩膀&轮子电机只用到了速度or位置模式，其他模式默认修改为位置模式(因为大部分的时间内都是在使用位置模式)
    }

    if(pmCtrl->mCtrlMode != ctlMd)
    {
        pmCtrl->mCtrlMode = ctlMd;
    }
    
    return ERR_NONE;
    
}



/* --------------------------------------------------------------------------*/
/**
* @name SysE2PromWriteProtectFuncSet
* @brief 
* @details Set the write protect pin of AT24C1024b Enable or Disable
*        WP pin:connected to GND,allow normal write operations.
*        connected to Vcc,all write operations to the memory are inhibited
*
* @param[in] sta   0   : Disable
*         others: Enable
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 SysE2PromWriteProtectFuncSet(u8 sta)
{    
    if(sta>0)
    {
        GPIOC->BSRRL = GPIO_Pin_13 ;     //!<  Set WP Pin high.
    }
    else
    {
        GPIOC->BSRRH = GPIO_Pin_13 ;    //!<  Set WP Pin low.
    }

    return ERR_NONE;
}

/* --------------------------------------------------------------------------*/
/**
* @name SysE2PromWriteProtectFuncGet
* @brief 
* @details Get the status of AT24C1024b Write Protect pin  
*
* @param[in] pSta pointer of pin state
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 SysE2PromWriteProtectFuncGet(u8* pSta)
{
    *pSta = (( GPIOC->IDR & GPIO_Pin_13) ? 1 : 0);

    return ERR_NONE;
}

/* --------------------------------------------------------------------------*/
/**
* @name SysE2PromWrite
* @brief 
* @details 写AT24C1024函数，但是写时间有些过长，后续再根据写数据的数目来优化
*          program time。
*
* @param[in] addr AT24C1024B的地址(at24容量为128k字节.顺序写单次最多为256bytes，
*                 否则EEPROM内部地址将会回滚)
*
* @returns nums 支持超过256字节的读写操作
* 
* @author Ling
*/
/* --------------------------------------------------------------------------*/
s8 SysE2PromWrite(u32 addr,u8* pdat,u32 nums)
{    
    u8 paddr0;
    u16 num0,num1;  //!< paddr0&num0为首页地址及首页待写数据个数,num1为尾页读数据个数
    s8 res=ERR_NONE;
    u8 pg,i;
    
    //!< 判断地址是否超限
    if(addr>EEPROM_ADDRESS_MAX)
    {
        return ERR_INPUT_PARAMETERS;
    }

    //!< 判断写入数据个数与地址的总和是否超过了当前page boundary
    paddr0 = addr%EEPROM_PAGE_CAPACITY;
    
    if((paddr0+nums)<=EEPROM_PAGE_CAPACITY)    //!< 所有待写入的数据在相同的页内，直接读并返回结果
    {
        res = I2C_Write(addr,pdat,nums);
    }
    else                    //!< 待写入的数据跨页了
    {
        num0 = (EEPROM_PAGE_CAPACITY - paddr0); //!< 获取首页数据个数，前面保证paddr0是小于EEPROM_PAGE_CAPACITY的
        pg = (nums-num0)/EEPROM_PAGE_CAPACITY;
        num1 = (nums-num0)%EEPROM_PAGE_CAPACITY;

        res = I2C_Write(addr,pdat,num0);

        if(res != ERR_NONE)
        {
            return res;
        }

        for(i=0;i<pg;i++)
        {
            res = I2C_Write(((addr+num0)+(i<<8)),((pdat+num0)+(i<<8)),EEPROM_PAGE_CAPACITY);

            if(res != ERR_NONE)
            {
                return res;
            }
        }

        res = I2C_Write((addr+nums-num1),(pdat+nums-num1),num1);

        if(res != ERR_NONE)
        {
            return res;
        }
    }

    return res;
}

/* --------------------------------------------------------------------------*/
/**
* @name 
* @brief 
* @details 
*
* @param[in] addr AT24C1024B的地址(at24容量为128k字节.顺序读单次最多为256bytes，
*                 否则EEPROM内部地址将会回滚)
*
* @returns nums 支持超过256字节的读写操作
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 SysE2PromRead(u32 addr,u8* pdat,u32 nums)
{
    u8 paddr0;
    u16 num0,num1;  //!< paddr0&num0为首页地址及首页读数据个数,num1为尾页读数据个数
    s8 res=ERR_NONE;
    u8 pg,i;
    
    //!< 判断地址是否超限
    if(addr>EEPROM_ADDRESS_MAX)
    {
        return ERR_INPUT_PARAMETERS;
    }

    //!< 判断读数据个数与地址的总和是否超过了当前page boundary
    paddr0 = addr%EEPROM_PAGE_CAPACITY;
    
    if((paddr0+nums)<=EEPROM_PAGE_CAPACITY)    //!< 所读的数据在相同的页内，直接读并返回结果
    {
        res = I2C_Read(addr,pdat,nums);
    }
    else                    //!< 所读的数据跨页了
    {
        num0 = (EEPROM_PAGE_CAPACITY - paddr0); //!< 获取首页数据个数，前面保证paddr0是小于EEPROM_PAGE_CAPACITY的
        pg = (nums-num0)/EEPROM_PAGE_CAPACITY;
        num1 = (nums-num0)%EEPROM_PAGE_CAPACITY;

        res = I2C_Read(addr,pdat,num0);

        if(res != ERR_NONE)
        {
            return res;
        }

        if(pg>0)
        {
            for(i=0;i<pg;i++)
            {
                res = I2C_Read(((addr+num0)+(i<<8)),((pdat+num0)+(i<<8)),EEPROM_PAGE_CAPACITY);

                if(res != ERR_NONE)
                {
                    return res;
                }
            }
        }

        if(num1>0)
        {
            res = I2C_Read((addr+nums-num1),(pdat+nums-num1),num1);
            
            if(res != ERR_NONE)
            {
            return res;
            }
        }    
    }

    return res;
}

/* --------------------------------------------------------------------------*/
/**
* @name sysPcToEmbsysCmdProcess
* @brief 
* @details process command pc send to embsys
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysPcToEmbsysCmdProcess(u16 Code,u32 timStamp,u16 dlen,u8* pdat)
{
    s8 res=ERR_NONE;
    //s8 res1;
        
    //!< 检查指令码 
    switch(Code)
    {
    case PCCmd_Clear:            
        res = PC_CmdClear(pdat,dlen);            
        break;
            
    case PCCmd_ObstclAvoid:            
        res = PC_CmdObstSwitch(pdat,dlen);
        break;
            
    case PCCmd_SysStop:            
        res = PC_PowerOffCMD(pdat,dlen);
        break;
            
    case PCCmd_Led:                
        res = PC_CmdLedCtrl(pdat,dlen);
        break;

    case PCCmd_DebugCtrl:
        res = PC_CmdDebugCtrl(pdat,dlen);
        break;
        
#if  Emb_LaserCtrl_Enable
    case PCCmd_Laser:
        res = PC_CmdLaserCtrl(pdat,dlen);
        break;
#endif

#if Emb_PC_HeartBeatPack_DEBUG
    case PCCmd_HeartBeat:
        res = PC_CmdHeartBeat(pdat,dlen);
        break;
#endif

    default:
        res = ERR_INPUT_PARAMETERS;
        break;
    }    
    
    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
    (pDebug->secondFun&SecFunDebugCtrl_XQueue_2_XCmdBuf_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_EmbSystem_MASK))
    {
        Bsp_printf("1.2 Recv EmbsysCmd Code:0x%x,Tim:0x%x,Len:%d",Code,timStamp,dlen);
    }

    return res;
}

/* --------------------------------------------------------------------------*/
/**
* @name g_GetPsmPsuHeader
* @brief 
* @details 检查数据是否为PSMPSU的协议头
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 g_GetPsmPsuHeader(u16 hd,u8* pf)
{    
    if(hd==PSM_HEADER_FLAG)
    {
        *pf = PSM_PACKET_HEADER;
        return ERR_NONE;
    }
    else if(hd==PSU_HEADER_FLAG)
    {
        *pf = PSU_PACKET_HEADER;
        return ERR_NONE;
    }
    else
    {
        return ERR_INPUT_PARAMETERS;
    }
}


/* --------------------------------------------------------------------------*/
/**
* @name g_DataRecvdFromPSMPSUprocess
* @brief 
* @details 检查接收数据是否符合要求:检查ID,fCode,Len,若通过则写数据到相应buffer
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 g_DataRecvdFromPSMPSUprocess(u8 fg,u8* pDat,u16 len)
{
    s8 res=ERR_NONE;

    //!<检查ID
    if((pDat[2]!=PSM_SlaveAddr)&&(pDat[2]!=BatteryCharger_SlaveAddr))
    {
        return ERR_RX_DATA_ADDR;
    }

    if(fg==PSM_PACKET_HEADER)
    {
    //!< DD DD 01 08 03 00 crc1 crc2
    //!< DD DD 01 27 02 00 xx xx....crc1 crc2
        res = g_GetPsmStatus(&EwayPSM,pDat,len);    //!< len已经在上一级代码中检查过长度        
    }
    else if(fg==PSU_PACKET_HEADER)
    {
        //!< EE EE 01 15 02 00 ...crc1 crc2
        res = g_GetBatteryStatus(&EwayBatteryCharger,pDat,len);
    }
    else
    {    
        return ERR_INPUT_PARAMETERS;
    }    

    return res;
}



/* --------------------------------------------------------------------------*/
/**
* @name sysProcessRecvDataFromPsmPsu
* @brief 
* @details 
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void sysProcessRecvDataFromPsmPsu(void)
{
    u8 dat[100]={0};
    u8* pbuf;
    u8 flag;
    u16 fLen,tmp1,tmp2;
    u16 unCheckCRC=0,usCount=0;
    u16 header;
    UART_T *pUart = &g_tUart7;

    //!<  usRxWrite 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();

    //!< 当COM7接收缓存中有数据且数据长度大于包头的长度，则开始检查数据
    while((pUart->usRxRead != pUart->usRxWrite)&&(usCount >= PSM_RESPONSE_INFO_LEN_MIN))
    {
        pbuf = pUart->pRxBuf + pUart->usRxRead;    

        if((pUart->usRxRead+1)==UART7_RX_BUF_SIZE)
        {
            header = pbuf[0]+((pUart->pRxBuf[0])<<8);
        }
        else
        {
            header = pbuf[0]+(pbuf[1]<<8);
        }

        if(g_GetPsmPsuHeader(header,&flag)!=ERR_NONE)             //!< find the pkt header
        {
            DISABLE_INT();
            pUart->usRxCount--;                    //!< 若包头不正确，则丢弃1字节数据，从下一字节开始继续寻找包头
            pUart->usRxRead++;
            if(pUart->usRxRead>=UART7_RX_BUF_SIZE)  
            {
                pUart->usRxRead = 0;
            }
            ENABLE_INT();

            usCount -= 1;

            continue;
        }

        //!< get the data pkt length                     //!< 根据接收缓存中数据的长度获取包长度
        if((pUart->usRxRead + 3) < UART7_RX_BUF_SIZE)    //!< fLen在buffer尾之前
        {
            fLen = *(pbuf+3);
        }
        else
        {
            tmp1 = UART7_RX_BUF_SIZE - pUart->usRxRead;        //!< usRxRead+3 = fLen位

            fLen = *(pUart->pRxBuf+3-tmp1);
        }

        //!< check data pkt length
        if( usCount < fLen )    //!< 当前收到的数据包不完整，退出，待收完整后再解析
        {
            break;
        }

        if(fLen < PSM_RESPONSE_INFO_LEN_MIN)
        {
            DISABLE_INT();
            pUart->usRxCount--;                    //!< 若数据长度不正确，则丢弃1字节数据，从下一字节开始继续寻找包头
            pUart->usRxRead++;
            if(pUart->usRxRead>=UART7_RX_BUF_SIZE)  
            {
                pUart->usRxRead = 0;
            }
            ENABLE_INT();

            usCount -= 1;

            continue;
        }

        //!< Get data
        if((pUart->usRxRead+fLen) <= UART7_RX_BUF_SIZE)                 //!< 获取整包数据
        {
            memcpy(dat,pbuf,fLen);
        }
        else
        {
            tmp1 = UART7_RX_BUF_SIZE - pUart->usRxRead;
            tmp2 = fLen - tmp1;

            memcpy(dat,pbuf,tmp1);
            memcpy((dat+tmp1),pUart->pRxBuf,tmp2);
        }

        unCheckCRC=CRC16_Modbus(dat, fLen-2);                //计算CRC校验位

        if(unCheckCRC != (dat[fLen-2]+(dat[fLen-1]<<8)))
        {
            DISABLE_INT();
            pUart->usRxCount--;
            pUart->usRxRead++;
            if(pUart->usRxRead>=UART7_RX_BUF_SIZE)  
            {
                pUart->usRxRead = 0;
            }
            ENABLE_INT();
            
            usCount -= 1;
            
            continue;
        }

        if(g_DataRecvdFromPSMPSUprocess(flag,dat,fLen)==ERR_NONE)        //!< 检查接收数据是否符合要求:检查ID，检查fCode,检查fLen是否超限,都通过，写数据到相应buffer
        {    
            DISABLE_INT();
            if((pUart->usRxRead + fLen) < UART7_RX_BUF_SIZE)
            {
                pUart->usRxRead += fLen;
            }
            else
            {
                tmp1 = UART7_RX_BUF_SIZE - pUart->usRxRead;
                pUart->usRxRead  = fLen - tmp1;        
            }        

            pUart->usRxCount -= fLen;
            ENABLE_INT();        

            usCount -= fLen;
            continue;
        }
        else  //!< check data error,go on at the next data sequence
        {
            DISABLE_INT();
            pUart->usRxCount--;
            pUart->usRxRead++;
            if(pUart->usRxRead>=UART7_RX_BUF_SIZE)  
            {
                pUart->usRxRead = 0;
            }
            ENABLE_INT();
            
            usCount -= 1;
                    
            continue;        
        }        
    }
}


void sysProcessRecvDataFromMic(void)
{
    u8 dat[100]={0};
    u8* pbuf;
    u16 fLen,tmp1,tmp2;
    u16 unCheckCRC=0,usCount=0;
    u16 header;
    UART_T *pUart = &g_tUart8;

    //!<  usRxWrite 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();

    //!< 当COM8接收缓存中有数据且数据长度大于包头的长度，则开始检查数据
    while((pUart->usRxRead != pUart->usRxWrite)&&(usCount >= MIC_RESPONSE_INFO_LEN_MIN))
    {
        pbuf = pUart->pRxBuf + pUart->usRxRead;    

        if((pUart->usRxRead+1)==UART8_RX_BUF_SIZE)
        {
            header = pbuf[0]+((pUart->pRxBuf[0])<<8);
        }
        else
        {
            header = pbuf[0]+(pbuf[1]<<8);
        }

        if(header!=MIC_HEADER_FLAG)             //!< find the pkt header
        {
            DISABLE_INT();
            pUart->usRxCount--;                    //!< 若包头不正确，则丢弃1字节数据，从下一字节开始继续寻找包头
            pUart->usRxRead++;
            if(pUart->usRxRead>=UART8_RX_BUF_SIZE)  
            {
                pUart->usRxRead = 0;
            }
            ENABLE_INT();

            usCount -= 1;

            continue;
        }

        //!< get the data pkt length                     //!< 根据接收缓存中数据的长度获取包长度
        if((pUart->usRxRead + 3) < UART8_RX_BUF_SIZE)    //!< fLen在buffer尾之前
        {
            fLen = *(pbuf+3);
        }
        else
        {
            tmp1 = UART8_RX_BUF_SIZE - pUart->usRxRead;        //!< usRxRead+3 = fLen位

            fLen = *(pUart->pRxBuf+3-tmp1);
        }

        //!< check data pkt length
        if( usCount < fLen )    //!< 当前收到的数据包不完整，退出，待收完整后再解析
        {
            break;
        }

        if(fLen < MIC_RESPONSE_INFO_LEN_MIN)
        {
            DISABLE_INT();
            pUart->usRxCount--;                    //!< 若数据长度不正确，则丢弃1字节数据，从下一字节开始继续寻找包头
            pUart->usRxRead++;
            if(pUart->usRxRead>=UART8_RX_BUF_SIZE)  
            {
                pUart->usRxRead = 0;
            }
            ENABLE_INT();

            usCount -= 1;

            continue;
        }

        //!< Get data
        if((pUart->usRxRead+fLen) <= UART8_RX_BUF_SIZE)                 //!< 获取整包数据
        {
            memcpy(dat,pbuf,fLen);
        }
        else
        {
            tmp1 = UART8_RX_BUF_SIZE - pUart->usRxRead;
            tmp2 = fLen - tmp1;

            memcpy(dat,pbuf,tmp1);
            memcpy((dat+tmp1),pUart->pRxBuf,tmp2);
        }

        unCheckCRC=CRC16_Modbus(dat, fLen-2);                //计算CRC校验位

        if(unCheckCRC != (dat[fLen-2]+(dat[fLen-1]<<8)))
        {
            DISABLE_INT();
            pUart->usRxCount--;
            pUart->usRxRead++;
            if(pUart->usRxRead>=UART8_RX_BUF_SIZE)  
            {
                pUart->usRxRead = 0;
            }
            ENABLE_INT();
            
            usCount -= 1;
            
            continue;
        }

        if(g_DataRecvdFromMicprocess(dat,fLen)==ERR_NONE)        //!< 检查接收数据是否符合要求:检查ID，检查fCode,检查fLen是否超限,都通过，写数据到相应buffer
        {    
            DISABLE_INT();
            if((pUart->usRxRead + fLen) < UART8_RX_BUF_SIZE)
            {
                pUart->usRxRead += fLen;
            }
            else
            {
                tmp1 = UART8_RX_BUF_SIZE - pUart->usRxRead;
                pUart->usRxRead  = fLen - tmp1;        
            }        

            pUart->usRxCount -= fLen;
            ENABLE_INT();        

            usCount -= fLen;
            continue;
        }
        else  //!< check data error,go on at the next data sequence
        {
            DISABLE_INT();
            pUart->usRxCount--;
            pUart->usRxRead++;
            if(pUart->usRxRead>=UART8_RX_BUF_SIZE)  
            {
                pUart->usRxRead = 0;
            }
            ENABLE_INT();
            
            usCount -= 1;
                    
            continue;        
        }        
    }
}



/*0605
先只配置电机控制模式、电机应用模式、减速比，位置闭环控制类型后续待定
*/
s8 EmbConfigMotorsParametersForArms(u8 stId,u8 nums,EwayEmbSysMotorInfoModule* pMotor)
{
    u8 i,j;
    NodeCtrlModule* pMotCtrl = &EwayArms.mControl;

    if((nums>Motor_Num_Arm)||(pMotor == NULL))
    {
        return ERR_INPUT_PARAMETERS;
    }

    //电机应用模式配置，默认手臂为关节模式
    pMotCtrl->mAppMode = App_Mode_Joint;

    //电机控制模式配置，默认为位置模式
    pMotCtrl->mCtrlMode = Ctrl_Mode_Posit;

    //各个电机的减速比
    for(i=(stId-1);i<nums;i++)
    {
        j = (u8)((pMotor+i)->mReductionRatioMode);

        if(j>((u8)tWhel))     //!< 若减速比不正确，则默认为1
        {
            j = (u8)tWhel;
        }        
            
        if((pMotCtrl->pMotorRatio + i) != NULL)
        {            
            *(pMotCtrl->pMotorRatio + i) = MotorReductionRatio[j];
        }
        else
        {
            break;
        }   
    }    

    return ERR_NONE;
}

s8 EmbConfigMotorsParametersForHeads(u8 stId,u8 nums,EwayEmbSysMotorInfoModule* pMotor)
{
    u8 i,j;
    NodeCtrlModule* pMotCtrl = &EwayHeads.mControl;
    
    if((nums>Motor_Num_Head)||(pMotor == NULL))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    //电机应用模式配置，默认头部为关节模式
    pMotCtrl->mAppMode = App_Mode_Joint;
    
    //电机控制模式配置，默认为位置模式
    pMotCtrl->mCtrlMode = Ctrl_Mode_Posit;
    
    //各个电机的减速比
    for(i=(stId-Emb_StartID_Head);i<nums;i++)
    {
        j = (u8)((pMotor+i)->mReductionRatioMode);
    
        if(j>((u8)tWhel))     //!< 若减速比不正确，则默认为1
        {
            j = (u8)tWhel;
        }        
                
        if((pMotCtrl->pMotorRatio + i) != NULL)
        {            
            *(pMotCtrl->pMotorRatio + i) = MotorReductionRatio[j];
        }
        else
        {
            break;
        }   
    }    

    return ERR_NONE;
}

s8 EmbConfigMotorsParametersForShoulder(u8 stId,u8 nums,EwayEmbSysMotorInfoModule* pMotor)
{
    u8 i,j;
    NodeCtrlModule* pMotCtrl = &EwayShoulder.mControl;
    
    if((nums>Motor_Num_Shoulder)||(pMotor == NULL))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    //电机应用模式配置，默认肩部为车轮模式
    pMotCtrl->mAppMode = App_Mode_Wheel;
    
    //电机控制模式配置，默认为速度模式
    pMotCtrl->mCtrlMode = Ctrl_Mode_Speed;
    
    //各个电机的减速比
    for(i=(stId-Emb_StartID_Shoulder);i<nums;i++)
    {
        j = (u8)((pMotor+i)->mReductionRatioMode);
    
        if(j>((u8)tWhel))     //!< 若减速比不正确，则默认为1
        {
            j = (u8)tWhel;
        }        

        if((pMotCtrl->pMotorRatio + i) != NULL)
        {            
            *(pMotCtrl->pMotorRatio + i) = MotorReductionRatio[j];
        }
        else
        {
            break;
        }        
    }    

    return ERR_NONE;
}

s8 EmbConfigMotorsParametersForWheels(u8 stId,u8 nums,EwayEmbSysMotorInfoModule* pMotor)
{
    u8 i,j;
    NodeCtrlModule* pMotCtrl = &EwayWheels.mControl;
    
    if((nums>Motor_Num_Wheel)||(pMotor == NULL))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    //电机应用模式配置，默认轮子为车轮模式
    pMotCtrl->mAppMode = App_Mode_Wheel;
    
    //电机控制模式配置，默认为位置模式
    pMotCtrl->mCtrlMode = Ctrl_Mode_Posit;
    
    //各个电机的减速比
    for(i=(stId-Emb_StartID_Wheel);i<nums;i++)
    {
        j = (u8)((pMotor+i)->mReductionRatioMode);
    
        if(j>((u8)tWhel))     //!< 若减速比不正确，则默认为1
        {
            j = (u8)tWhel;
        }        
                
        if((pMotCtrl->pMotorRatio + i) != NULL)
        {            
            *(pMotCtrl->pMotorRatio + i) = MotorReductionRatio[j];
        }
        else
        {
            break;
        }   
    }    

    return ERR_NONE;
}




s8 sysReadEmbSystemInfoFromE2prom(void)
{
    s8 res;
    u8 i;

    i = EMB_READ_E2PROM_MAX_TIMES;

    while(i>0)
    {
        res = ReadInfoFromE2prom();
        if(res==ERR_NONE)
        {
            break;
        }
        else
        {
            osDelay(100);       //!< 若读数据不成功，则延时后再次读
        }

        i--;        
    }

    if(res != ERR_NONE)
    {
        //sysSetEmbSystemInfotoDefault(&EwayEmbSysInfo);
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            Bsp_printf("ReadEmbSystemInfoFromE2prom failed.rt:%d",res);
        }
    }

    return res;

}



/* --------------------------------------------------------------------------*/
/**
* @name 
* @brief 
* @details 
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 g_ClearPcCmdtoNodeCmdBuff(MotorCMDBuff* pCmd,u16 id)
{
    MotorCMDBuff* pC = (pCmd+id);   
    osStatus re;

    if(pC->dCmdCnt == 0)
    {
        return ERR_NONE;
    }

    re = osMutexWait(*(pC->mCmdBuffHandle),1);

    if(re != osOK)
    {
        return ERR_PROCESS_FAILED;
    }

    pC->dCmdRe = 0;
    pC->dCmdWr = 0;
    pC->dCmdCnt = 0;
    memset(&(pC->dCmd[0][0]),0,CMD_PC_2_EMB_BUFF_NUMS*CMD_PC_2_EMB_BUFF_LEN);

    osMutexRelease(*(pC->mCmdBuffHandle));  

	return ERR_NONE;
}




s8 g_StorePcCmdtoCmdBuff(MotorCMDBuff* pCmd,u16 id,u8* pdat,u8 len)    
{	
	MotorCMDBuff* pC = (pCmd+id);	
	u8* pDest;     
    osStatus re;

	if(len>CMD_PC_2_EMB_BUFF_LEN)                         //!< code(2) + timeStamp(4) + dataUnit 
	{	
	    return ERR_INPUT_PARAMETERS;
    }

	if(pC->dCmdCnt>=CMD_PC_2_EMB_BUFF_NUMS)
    {
		return ERR_DATA_OVERFLOW;
    }

    re = osMutexWait((*(pC->mCmdBuffHandle)),1);

    if(re != osOK)
    {
        return ERR_PROCESS_FAILED;
    }
	
	pDest = &(pC->dCmd[(pC->dCmdWr)][0]);

	memcpy(pDest,pdat,len);

	pC->dCmdWr++;pC->dCmdCnt++;

	if(pC->dCmdWr>=CMD_PC_2_EMB_BUFF_NUMS)
	{
		pC->dCmdWr = 0;
	}

    osMutexRelease(*(pC->mCmdBuffHandle));
	
	return ERR_NONE;	
}



s8 g_ClearPartXPcCmdBuff(EwayMotor_Device_Type devType,QueueHandle_t* pQue,MotorCMDBuff* pCmdBuff)
{
    u8 i,mNums;
    s8 res=ERR_NONE;
    BaseType_t re;

    if(pQue==NULL)
    {
        return ERR_INPUT_PARAMETERS;
    }

    switch(devType)
    {
        case EWAYBOT_ARMS_LEFT: 
        case EWAYBOT_ARMS_RIGHT:            
            mNums = Motor_Num_Arm;
            break;
        case EWAYBOT_SHOULDER:
            mNums = Motor_Num_Shoulder;
            break;
        case EWAYBOT_HEAD:
            mNums = Motor_Num_Head;
            break;
        case EWAYBOT_GRIPPERS:
            mNums = Motor_Num_Gripper;
            break;
        case EWAYBOT_WHEEL:
            mNums = Motor_Num_Wheel;
            break;
        default:
            res = ERR_INPUT_PARAMETERS;
            break;
    }

    if(res!=ERR_NONE)
    {
        return res;
    }
    
    re = xQueueReset(*pQue);
    if(re != pdPASS)
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
                Bsp_printf("Clr Queue PCToXQueHdl failed,devTy:%d,rt:%d.",devType,re);
        }
    }   
        
    //!< 清各个电机的指令待执行缓存
    for(i=0;i<mNums;i++)
    {
        res = g_ClearPcCmdtoNodeCmdBuff(pCmdBuff,i);
        if(res!=ERR_NONE)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                Bsp_printf("Clr devType:%d iD:% ToMotorCmd Cmd Buf failed,rt:%d.",devType,i,res);
            }
        }
    }

    return ERR_NONE;
}


s8 g_GetPCCmdEmbToMotorCmd(MotorCMDBuff* pmotorCmd,u16 id,u8* pDat)
{
    MotorCMDBuff* pC = (pmotorCmd+id);   
    osStatus re;
    s8 res = ERR_NONE;    
    
    re = osMutexWait(*(pC->mCmdBuffHandle),1);

    if(re != osOK)
    {
        return ERR_PROCESS_FAILED;
    }

    if((pC->dCmdCnt)>0)
	{
	    memcpy(pDat,&(pC->dCmd[pC->dCmdRe][0]),CMD_PC_2_EMB_BUFF_LEN);

	    pC->dCmdCnt--;								 

	    pC->dCmdRe++;

	    if((pC->dCmdRe)>=CMD_PC_2_EMB_BUFF_NUMS) pC->dCmdRe = 0;
	}
    else
    {
		res = ERR_BUFFER_EMPTY;
    }

	osMutexRelease(*(pC->mCmdBuffHandle)); 

	return res;
}


void sysClrPcToEmbPartsCmdBuff(void)
{
    u8 i;
    s8 res;
    //UBaseType_t tmp;
    
    for(i=0;i<5;i++)
    {
        res = g_ClearPartXPcCmdBuff(EmbSysPartType[i],EmbSysPartQue[i],(*(EmbSysPartCmdBuff[i])));
        if(res!=ERR_NONE)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                Bsp_printf("g_ClearPartXPcCmdBuff(%d) failed,rt:%d",i,res);
            }
        }
        else
        {            
            //tmp = uxQueueMessagesWaiting(*(EmbSysPartQue[i]));
            //Bsp_printf("NetDisconn,ClrCmd,dev:%d,qCmd:%d,CmdBuff:%d.",EmbSysPartType[i],tmp,(*(EmbSysPartCmdBuff[i]))->dCmdCnt);
        }
    }

}

s8 EmbEnableLastPosCmdAvailableFlag(Motor_LastPosCmd* pLstPos,u8 Idx)       //!< 因收到速度指令而开启的功能
{
    if(pLstPos==NULL)
    {
        return ERR_INPUT_PARAMETERS;
    }

    if((pLstPos+Idx)->avail == LastPosAvailableOff)
    {
        (pLstPos+Idx)->avail = LastPosAvailableSpdOn;
    }

    return ERR_NONE;
}

s8 EmbRecordingLastPosCmdSendtoMotor(Motor_LastPosCmd* pLstPos,u8 Idx,s32 pos)
{
    if(pLstPos==NULL)
    {
        return ERR_INPUT_PARAMETERS;
    }

    if((pLstPos+Idx)->avail !=LastPosAvailablePosOn)
    {
        (pLstPos+Idx)->avail = LastPosAvailablePosOn;

        (pLstPos+Idx)->lastPos = pos;
    }
    
    (pLstPos+Idx)->tmpPos = pos;

    return ERR_NONE;
}


s8 EmbUpdateLastPosCmdSendtoMotor(Motor_LastPosCmd* pLstPos,u8 Idx)
{
    if(pLstPos==NULL)
    {
        return ERR_INPUT_PARAMETERS;
    }

    (pLstPos+Idx)->lastPos = (pLstPos+Idx)->tmpPos;

    return ERR_NONE;

}

s8 EmbClrLastPosCmdAvailableFlag(Motor_LastPosCmd* pLstPos,u8 Idx)
{
    if(pLstPos==NULL)
    {
        return ERR_INPUT_PARAMETERS;
    }

    (pLstPos+Idx)->avail = 0;

    return ERR_NONE;
}

s8 EmbGetLastPosCmdSendtoMotor(Motor_LastPosCmd* pLstPos,u8 Idx,s32* pRslt)
{
    if(pLstPos==NULL)
    {
        return ERR_INPUT_PARAMETERS;
    }

    *pRslt = (pLstPos+Idx)->lastPos;

    return  ERR_NONE;
}

void sysUpdateMotorsLastPosCmd(void)
{
    u8 i;
    
    //!< 更新手臂电机的最后一条位置指令
    for(i=0;i<Motor_Num_Arm;i++)
    {
        EmbUpdateLastPosCmdSendtoMotor(EwayArms.mControl.pmlstPos,i);
    }

    //!< 更新头部电机的最后一条位置指令
    for(i=0;i<Motor_Num_Head;i++)
    {
        EmbUpdateLastPosCmdSendtoMotor(EwayHeads.mControl.pmlstPos,i);
    }

    //!< 更新肩膀电机的最后一条位置指令


    //!< 更新轮子的最后一条位置指令
}


/* --------------------------------------------------------------------------*/
/**
* @name EmbRecordCmdSendToMotor
* @brief 
* @details 记录下发给电机(肩膀和轮子电机)的指令，包括电机ID索引号,指令类型，速度，位置，存放指令的缓存地址等
*   存储下发给电机的运动指令在下位机缓存中的格式
*
*               tStamp(4) ID(2) insType(0/1)  DnPos(2/4)  DnSpd(2)    UpPos(2/4)  UpSpd(2)
*手臂、头部(14)   4        2        0             2          2            2          2
*
*肩膀、轮子(19)   4        2        1             4          2            4          2
*
* @param[in] 
*
* @returns 
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 EmbRecordCmdSendToMotor(u8 Idx,Motor_CtrlMode sCtrlMode,s16 sSpd,s32 sPosi,EwayMotor_Device_Type devType)
{
    s8 res=ERR_NONE;
    MotorRecordBuff* pRecordBuff;    
    u8 maxId,startId;
    u8 unSendBuf[Write_Regs_Record_Max_Space]={0};

#if EMB_Upload_SingleCycle_Motor_Info_DEBUG

    //!< 检查设备类型
    if((devType != EWAYBOT_SHOULDER)&&(devType != EWAYBOT_WHEEL))
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(sysGetMotorSendRecordPointer(devType,&pRecordBuff,&maxId,&startId)==ERR_NONE)
    {           
        //!< 检查ID号
        if(Idx > maxId)
        {
            return ERR_INPUT_PARAMETERS;
        }

        //!< 检查电机控制指令类型
        switch(sCtrlMode)
        {
            case Ctrl_Mode_Posit:
                unSendBuf[6] = EwayMotor_CtrlMode_Posit;
                break;
            case Ctrl_Mode_Speed:
                unSendBuf[6] = EwayMotor_CtrlMode_Speed;
                break;
            default:                
                res = ERR_INPUT_PARAMETERS;
                break;
        }
        if(res != ERR_NONE)
        {
            return res;
        }
        
        //!< 向相应指令缓存按照规定的格式存入数据  
        unSendBuf[0] = (u8)SysTimeStamp;
        unSendBuf[1] = (u8)(SysTimeStamp>>8);
        unSendBuf[2] = (u8)(SysTimeStamp>>16);            
        unSendBuf[3] = (u8)(SysTimeStamp>>24);
    
        unSendBuf[4] = (u8)(startId + Idx);
        unSendBuf[5] = (u8)((startId + Idx)>>8);
                        
        unSendBuf[7] = (u8)sPosi;           //!< DnPos
        unSendBuf[8] = (u8)(sPosi>>8);
        unSendBuf[9] = (u8)(sPosi>>16);
        unSendBuf[10] = (u8)(sPosi>>24);

        unSendBuf[11] = (u8)sSpd;
        unSendBuf[12] = (u8)(sSpd>>8);
                        
        memcpy(&((pRecordBuff+Idx)->wRegs[0]),unSendBuf,13);        
    }
    else
    {
        return ERR_PROCESS_FAILED;
    }
    
#endif 

    return ERR_NONE;
}



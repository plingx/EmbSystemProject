/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file EmbSysProc.c
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
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

    //!< �ϵ���ȶ�E2prom���ݣ���ͨ����ִ����������
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
* @details ������λ��������ⲿ�豸ͨ��״̬������ǰ״̬����EwayEmbSys.rtComm��
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
    ///��EwayEmbSys.rtComm�����ĸ���
    ////// ����ϵͳʵʱ���ⲿ��(��PC,��ÿ��Motor,��PSMPSU,��)ͨ��״̬����EwayEmbSys.rtComm
    
    // ÿ���ڸ�����PC��ͨ��״̬ update Comm Status with PC
    if(EwayEmbTCP.TCP_State == ES_TCPSERVER_CONNECTED)
    {
       EwayEmbSys.rtComm |= (0x00000001<<EMB_COMM_PC_START_BIT);
    }    
 
    // ÿ���ڸ���������ͨ��״̬ update Comm Status with Motors
    
    ////// �ֱۡ����ͷ��
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
        
        //!< Ŀǰֻ��1������������ֱ��д��
        //
        if((EwayShoulder.mResp.mExeCmRecd[0]>>4) != 0)     //!< ���������з���ģʽ�л�ָ��
        {
            if(l_CompareMotorsExeCmResponseCount(&EwayShoulder.mResp)==ERR_NONE)   //!< Ŀǰֻ�Լ���ģʽ�����м�¼���������и���ı��ָ������¶��幦����
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
        
        ////// צ��
    for(i=0;i<Motor_Num_Gripper;i++)
    {
        if((EwayGrippers.mCommStatus&(0x0001<<i))!=0)
        {
            EwayEmbSys.rtComm |= 0x00000001<<(i+EMB_COMM_GRIP_START_BIT);
        }        
    }
    
    //// ����
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

    //��������Ƿ�����Ҫ�ظ���ָ��͸����ӵ���������ӵ���Ƿ�ظ�     
    if(((EwayWheels.mResp[0].mExeCmRecd[0]>>4) != 0)||((EwayWheels.mResp[1].mExeCmRecd[0]>>4)!=0))     //!< ���������з���ģʽ�л�ָ��or����ֹͣ��ָͣ��
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
        
    // ÿ���ڸ�����PSM&PSU��ͨ��״̬ update Comm Status with PSM&PSU
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

    //Mic ͨ��״̬���
    if((EwayEmbSys.sysTimCnt%EMB_READ_MIC_INTERVAL)==0)
    {
        if(EwayMic.mCommStatus != 0)        //!< ��¼��һͨ�����ڵ�ͨ��״̬
        {
            EwayEmbSys.rtComm |= 0x00000001<<EMB_COMM_MIC_START_BIT;
        }
    }
}

/* --------------------------------------------------------------------------*/
/**
* @name sysEmbCommStateMonitor
* @brief 
* @details ��λ��������ⲿ�豸ͨ��״̬��⣬��������λ�������������PSM&PSU
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

///////!< 1.������EwayEmbSys.tComm�д�ŵı����ڸ��������ͨ��״̬��¼������EwayEmbSys.Comm��

    //!< 1.1 ��¼�����״̬
    for( i = EMB_COMM_ARM_START_BIT; i < (SYS_MAX_GENERAL_SERVO_NUMS+Motor_Num_Gripper); i++ )
    {
        EwayEmbSys.Comm.wMotors[i] = EwayEmbSys.Comm.wMotors[i]<<1;

        if((EwayEmbSys.rtComm&(0x00000001<<i)) != 0)
        {
            EwayEmbSys.Comm.wMotors[i] += 1;
        }
    }

    //!< 1.2 ��¼��λ����ͨ��״̬
    EwayEmbSys.Comm.wPc = EwayEmbSys.Comm.wPc<<1;
    if((EwayEmbSys.rtComm&(0x00000001<<EMB_COMM_PC_START_BIT)) != 0)
    {
        EwayEmbSys.Comm.wPc += 1;
    }

    //!< 1.3 ��¼��һ��ͨ������(480ms)PSMPSU��ͨ��״̬
    if((EwayEmbSys.sysTimCnt%EMB_READ_PSM_PSU_INTERVAL)==0) //!< psmͨ��״̬��أ������һ��ͨ�������ڣ����͵����ݰ��Ƿ��յ��˻ظ���
    {
        EwayEmbSys.Comm.wPsm = EwayEmbSys.Comm.wPsm<<1;
        if((EwayEmbSys.rtComm&(0x00000001<<EMB_COMM_PSM_START_BIT)) != 0)
        {
            EwayEmbSys.Comm.wPsm += 1;
        }

        EwayEmbSys.Comm.wPsu= EwayEmbSys.Comm.wPsu<<1;              //!< ��¼PSU��ͨ��״̬
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

    //!< 1.4 ��¼Mic��ͨ��״̬
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
    sysEmbCommStateUpdate();//!< ����ֱۡ����ͷ�����,����λ����PSM��PSU��ͨ��״̬

    sysEmbCommStateMonitor();//!< ����ϵͳ��ͨ��״̬
}


void sysEmbMotorsCommSupervise(void)
{
    //u8 i;
    //u8 slvNums=0;
	  //u8 slvID[Motor_Num_Wheel];
    //u8 tmpDat[10]={0};
    //s8 res;
    
    /*
    //!< ������3��ͨ�����ڵ��ֱ۵��ͨ���Ƿ�����
    for(i=0;i<Motor_Num_Arm;i++)
    {
        if((EwayEmbSys.Comm.wMotors[i]&EMB_COMM_ERR_BITS_3_TIMES)==0x01)//!< ��Motorsͨ�Żָ�����
        {
            //!< ���lastPosCmd��available��������Ӧ��Emb��PC�������������ͣ��ĳ����Ȼ�������������ϵ��PC����ָ���PC�����쳣ʱ�����·��쳣��ָ�
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

    
    //!< ������3��ͨ�����ڵļ����ͨ���Ƿ�����
    if((EwayEmbSys.Comm.wMotors[12]&EMB_COMM_ERR_BITS_3_TIMES)==0x01)//!< ��Motorͨ�Żָ�����
    {
        EmbClrLastPosCmdAvailableFlag(EwayShoulder.mControl.pmlstPos,0);
        
        Bsp_printf("ShoulderMotorComm resume, Clr lastPosCmd avail flag.");
    }

    //!< ������3��ͨ�����ڵ�ͷ�����ͨ���Ƿ�����    
    for(i=0;i<Motor_Num_Head;i++)
    {
        if((EwayEmbSys.Comm.wMotors[i+(Motor_Num_Arm+Motor_Num_Shoulder)]&EMB_COMM_ERR_BITS_3_TIMES)==0x01)//!< ��Motorsͨ�Żָ�����
        {
            //!< ���lastPosCmd��available��������Ӧ��Emb��PC�������������ͣ��ĳ����Ȼ�������������ϵ��PC����ָ���PC�����쳣ʱ�����·��쳣��ָ�
            EmbClrLastPosCmdAvailableFlag(EwayHeads.mControl.pmlstPos,i);
                    
            Bsp_printf("HeadMotorComm(%d) resume, Clr lastPosCmd avail flag",i);
        }
    }*/

    //!< ������3��ͨ�����ڵ����ӵ��ͨ���Ƿ�����   
    /*
    for(i=0;i<Motor_Num_Wheel;i++)
    {
        if((EwayEmbSys.Comm.wMotors[i+15]&0xFF)==0x03)//!< ��Motorsͨ�Żָ�����
        {
            //!< ���lastPosCmd��available��������Ӧ��Emb��PC�������������ͣ��ĳ����Ȼ�������������ϵ��PC����ָ���PC�����쳣ʱ�����·��쳣��ָ�
            //EmbClrLastPosCmdAvailableFlag(EwayHeads.mControl.pmlstPos,i);
                    
            //Bsp_printf("HeadMotorComm(%d) resume, Clr lastPosCmd avail flag",i);
            //slvID[slvNums] = i;

            slvNums += 1;
        }
    }
    

    if(slvNums>0)       //!< �������и��ϵ磬����Ҫ�����ӷ���һ������ֹͣʹ�ܵ�ָ��(��0x51regд2)
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
    
    //!< ���ڼ��Readyλ�Ƿ���Ѿ���λ
    if((EwayEmbSys.sysTimCnt%EMB_READ_WHEEL_MOTORS_READY_REG_INTERVAL)==0)      //!< ÿ��240ms��ѯһ��
    {
        if((EwayWheels.mRdyStatus&0x0003)!=0x0003)      //!< 2�����ӵ����Ҫͬʱ׼����
        {
            //!< ���Ͳ�ѯ��
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

    //!< ������ӵ���Ƿ���ͨ�Ų�����
    for(i=0;i<Motor_Num_Wheel;i++)
    {
        if((EwayEmbSys.Comm.wMotors[i+EMB_COMM_WHEEL_START_BIT]&0x0F)==0x80)//!< ��Motorsͨ�ŶϿ�
        {
            EwayWheels.mRdyStatus = 0;            

            Bsp_printf("Wheel(%d) Comm is break,Clr Rdy Status.",(i+1));
        }
    }    

}

//!< ��鵱����ͨ���쳣ʱ����Ҫ���PC Cmd buffer & ������ָ��buffer
void sysEmbCheckWheelMotorsCommuStatus(void)
{
    u8 tmp0,tmp1;
    UBaseType_t nums;
    BaseType_t re;
    u16 i;
    s8 res;

    tmp0 = EwayEmbSys.Comm.wMotors[EMB_COMM_WHEEL_START_BIT]&EMB_COMM_ERR_BITS_3_TIMES;
    tmp1 = EwayEmbSys.Comm.wMotors[EMB_COMM_WHEEL_START_BIT+1]&EMB_COMM_ERR_BITS_3_TIMES;

    if((tmp0==0)||(tmp1==0))    //!< �����ӵ���
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

        //!< ����������ָ���ִ�л���
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
        
    tmpDat[0] = en;                  //!< reg0x51:1-����ֹͣ,2-�����ƶ�,3-����ֹͣ
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
    
    //!< ÿ��Լ4.8s�Ӽ��һ��

    //!< ���������
    status[0] = EwayEmbSys.genmCommSta.mCfgInitSta&0x00000003;     //!< ��ȡ���cfgRegs�ĳ�ʼ��״̬
    tmp[0] = EwayEmbSys.genmCommSta.mCfgRegCommSta&0x00001000; 

    switch(status[0])
    {
        case 0:
                if(tmp[0] != 0)   //!< �������ж���cfgReg
                {
                    sysEmbCheckMotorAppCtrlModeInConfigRegs(&EwayShoulder.mControl,0,&MotorCfgReg[12][0]);

                    EwayEmbSys.genmCommSta.mCfgInitSta &= 0xFFFFFFFC;    //!< ����״̬

                    EwayEmbSys.genmCommSta.mCfgInitSta |= 0x00000001;

                    //!< ���������cfgRegs�Ƿ��в�����Ҫ��ģ����У����͹㲥д��ָ��
                    l_CheckMotorsCfgRegComparedResultAndPackSettingData(chkdev[2],devCtrlMd[2],EwayEmbSys.genmCommSta.mCfgRegCommSta);                    
                }
                
                EwayEmbSys.genmCommSta.mCfgRegCommSta &= ~(0x00001000);     //!< ���cfgReg״̬��������ÿ���ڶ�cfgReg
                
                g_EmbSendQueryPacketToReadMotorCfgRegisters(chkdev[2]);
                
            break;
        case 1:
                if((EwayEmbSys.sysTimCnt%EMB_READ_MotorsCfgReg_INTERVAL)==2)        //!< Emb�����ϵ���ʼ�����������״̬��ֻ��Ҫ���ڼ���ok
                {
                    if(tmp[0] != 0)
                    {
                        sysEmbCheckMotorAppCtrlModeInConfigRegs(&EwayShoulder.mControl,0,&MotorCfgReg[12][0]);
                        
                        l_CheckMotorsCfgRegComparedResultAndPackSettingData(chkdev[2],devCtrlMd[2],EwayEmbSys.genmCommSta.mCfgRegCommSta);
                    }
                    else
                    {
                        EwayEmbSys.genmCommSta.mCfgInitSta &= ~(0x00000003);        //!< �����ڲ�ѯ����δ����cfgReg���������״̬����ת��0�������ڶ�ȡ
                    }

                    EwayEmbSys.genmCommSta.mCfgRegCommSta &= ~(0x00001000);     //!< ���cfgReg״̬��������ÿ���ڶ�cfgReg
                
                    g_EmbSendQueryPacketToReadMotorCfgRegisters(chkdev[2]);
                }
            break;
        default:
            break;
    }

    //!< ���ӵ��������id=1����Ϊ׼��status=0ʱ
    for(i=0;i<Motor_Num_Wheel;i++)
    {
        status[i] = EwayEmbSys.genmCommSta.mCfgInitSta&(0x00000003<<((i+1)<<1));     //!< ��ȡ����cfgRegs�ĳ�ʼ��״̬
        tmp[i] = EwayEmbSys.genmCommSta.mCfgRegCommSta&(0x00000001<<(i+EMB_COMM_WHEEL_START_BIT)); 
    }

    if((status[0]!=0)&&(status[1]!=0))          //!< 2�����ӵ�����Ѿ����ع�cfgReg�������˶���(4.8s)��ѯ�׶�
    {
        if((EwayEmbSys.sysTimCnt%EMB_READ_MotorsCfgReg_INTERVAL)==2)
        {
            for(i=0;i<Motor_Num_Wheel;i++)
            {
                if(tmp[i]==0)                   //!< �����ڲ�ѯ����δ����cfgReg���������״̬����ת��0�������ڶ�ȡ
                {
                    EwayEmbSys.genmCommSta.mCfgInitSta &= (~(0x00000003<<((i+1)<<1)));
                }
                else                            //!< �����յ�������AppCtrl�Ƿ�һ��
                {
                    sysEmbCheckMotorAppCtrlModeInConfigRegs(&EwayWheels.mControl,i,&MotorCfgReg[(i+EMB_COMM_WHEEL_START_BIT)][0]);
                }
            }
        
            if((tmp[0]!=0)||(tmp[1]!=0))        //!< ����ͨ�������ĵ���������Ƿ���Ҫ���͹㲥дָ�����������ӵ����cfgRegs
            {
                l_CheckMotorsCfgRegComparedResultAndPackSettingData(chkdev[4],devCtrlMd[4],EwayEmbSys.genmCommSta.mCfgRegCommSta);
            }
            
            EwayEmbSys.genmCommSta.mCfgRegCommSta &= ~(0x00018000);     //!< ���cfgReg״̬��������ÿ���ڶ�cfgReg
                
            g_EmbSendQueryPacketToReadMotorCfgRegisters(chkdev[4]);

        }
    }
    else                                        //!< 2�����ӵ����û��ȫ�����ع�cfgReg���Ծɽ���ÿ���ڲ�ѯ����
    {
        for(i=0;i<Motor_Num_Wheel;i++)
        {
            if(tmp[i]==0)
            {
                continue;
            }

            sysEmbCheckMotorAppCtrlModeInConfigRegs(&EwayWheels.mControl,i,&MotorCfgReg[(i+EMB_COMM_WHEEL_START_BIT)][0]);

            EwayEmbSys.genmCommSta.mCfgInitSta &= (~(0x00000003<<((i+1)<<1)));    //!< ����״̬

            EwayEmbSys.genmCommSta.mCfgInitSta |= (0x00000001<<((i+1)<<1));
        }

        if((tmp[0]!=0)||(tmp[1]!=0))
        {
            //!< ���������cfgRegs�Ƿ��в�����Ҫ��ģ����У����͹㲥д��ָ��
            l_CheckMotorsCfgRegComparedResultAndPackSettingData(chkdev[4],devCtrlMd[4],EwayEmbSys.genmCommSta.mCfgRegCommSta);  
        }

        EwayEmbSys.genmCommSta.mCfgRegCommSta &= ~(0x00018000);     //!< ���cfgReg״̬��������ÿ���ڶ�cfgReg
                
        g_EmbSendQueryPacketToReadMotorCfgRegisters(chkdev[4]);
        
    }

    //!< ����ֱ۵����ͷ�������cfgRegs
    if((EwayEmbSys.sysTimCnt%EMB_READ_MotorsCfgReg_INTERVAL)==2)
    {
        //!< ����Ƿ��յ���һ���ڵ���ظ������üĴ���
            //!< ����ֱ۵�������üĴ���        

        //Bsp_printf("Check cfg regs,Comm:0x%x",EwayEmbSys.genmCommSta.mCfgRegCommSta);
        
        for(i=0;i<5;i++)
        {
            if((i!=2)&&(i!=4))        //!< ��ȥ����������ӵ��
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
        EwayEmbSys.genmCommSta.mCfgRegCommSta &= (0x00000001<<Motor_Num_Arm) + (0x03<<Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head);      //!< ������հ���״̬���Լ�������ӵ���Ĵ˴���������
        
        //!< �����������Ͳ�ѯ���üĴ����İ�
        for(i=0;i<5;i++)
        {
            if((i!=2)&&(i!=4))        //!< ��ȥ����������ӵ��
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
    
    //!< ÿ��Լ10s�Ӽ��һ��
    if((EwayEmbSys.sysTimCnt%EMB_READ_MotorsCfgReg_INTERVAL)==2)
    {
        //!< ����Ƿ��յ���һ���ڵ���ظ������üĴ���
            //!< ����ֱ۵�������üĴ���        

        //Bsp_printf("Check cfg regs,Comm:0x%x",EwayEmbSys.genmCommSta.mCfgRegCommSta);
        
        for(i=0;i<5;i++)
        {
            l_CheckMotorsCfgRegComparedResultAndPackSettingData(chkdev[i],devCtrlMd[i],EwayEmbSys.genmCommSta.mCfgRegCommSta);            
        }        
    }

    if((EwayEmbSys.sysTimCnt%EMB_READ_MotorsCfgReg_INTERVAL)==0)
    {
        EwayEmbSys.genmCommSta.mCfgRegCommSta = 0;      //!< ������հ���״̬
        
        //!< �����������Ͳ�ѯ���üĴ����İ�
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
* @details ��˳�򵥶�����Motor�����üĴ������ȽϼĴ����Ƿ���Ĭ��ֵ��ƥ��
*           ����ƥ�䣬����Ҫ���ļĴ�����ֵ��
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysInitReadAndCheckEmbMotorsCfgRegs(void)  //!< Ŀǰ��SingleRead�Ľ��ȫ������ȫ�ֱ���MotorCfgReg[][]�У��Ժ������Ҫ�ٸ���
{
    u8 dat[10]={7,6};//{0,0x2D,0x3C,7,0x50,16,0x78,21,0xB4,12};     //!< Ŀǰ��ֻ��reg:[7-12]��6���Ĵ���
    s8 res=ERR_NONE;
    u8 i;
    //osStatus tSta = osOK; 

    EwayEmbSys.genmCommSta.mCfgRegCommSta = 0;      //!< ����ձ�־λ

    //!< ���Ĵ��������ֱ�
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

    //!< ���Ĵ��������ֱ�
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

    //!< ���Ĵ���������ͷ
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

    //!< ���Ĵ���������
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

    //!< ���Ĵ�����צ��


    //!< ���������ֵ����üĴ���
        //!< ����ֱ۵�������üĴ���
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

        //!< ������������üĴ���
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

        //!< ���ͷ����������üĴ���
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

        //!< ������ӵ�������üĴ���
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
* @details ���ݼ�����������üĴ����Ľ�������͹㲥��or��ӡ������Ϣ
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
                startID = Emb_StartID_ArmL;     //!< ��� bit0.1.2.3.4.5
                devNum = Motor_Num_ArmLeft;
                
            break;

        case EWAYBOT_ARMS_RIGHT:
                startID = Emb_StartID_ArmR;     //!< �ұ� bit6.7.8.9.10.11
                devNum = Motor_Num_ArmRight;
                
            break;

        case EWAYBOT_SHOULDER:
                startID = Emb_StartID_Shoulder; //!< ��� bit12
                devNum = Motor_Num_Shoulder;
                
            break;

        case EWAYBOT_HEAD:
                startID = Emb_StartID_Head;     //!< ͷ�� bit13.14
                devNum = Motor_Num_Head;
                
            break;

        case EWAYBOT_WHEEL:
                startID = Emb_StartID_Wheel;     //!< ���� bit15.16
                devNum = Motor_Num_Wheel;
                
            break;

        default:                                //!< ��������������޴˹���
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
* @details ���ݼ�����������üĴ����Ľ�������͹㲥��or��ӡ������Ϣ
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
                MskBit = 0x0000003F;     //!< ��� bit0.1.2.3.4.5
                devNum = Motor_Num_ArmLeft;
                sBit = 0;
                ssBit = 0;
            break;

        case EWAYBOT_ARMS_RIGHT:
                MskBit = 0x00000FC0;     //!< �ұ� bit6.7.8.9.10.11
                devNum = Motor_Num_ArmRight;
                sBit = Motor_Num_ArmLeft;
                ssBit = 6;                    //!< ��������ұ۹���һ��NodeCtrlModule������NodeCtrlModule�е�Idx����㲻ͬ
            break;

        case EWAYBOT_SHOULDER:
                MskBit = 0x00001000;     //!< ��� bit12
                devNum = Motor_Num_Shoulder;
                sBit = Motor_Num_Arm;
                ssBit = 0;
            break;

        case EWAYBOT_HEAD:
                MskBit = 0x00006000;     //!< ͷ�� bit13.14
                devNum = Motor_Num_Head;
                sBit = Motor_Num_Arm+Motor_Num_Shoulder;
                ssBit = 0;
            break;

        case EWAYBOT_WHEEL:
                MskBit = 0x00018000;     //!< ���� bit15.16
                devNum = Motor_Num_Wheel;
                sBit = Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head;
                ssBit = 0;
            break;

        default:                                //!< ��������������޴˹���
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
            res = ERR_DATA_NOT_RDY;         //!< ��ʾδ�յ�����Ļظ�����������㲥�����üĴ���������Ҫ�㲥д���üĴ��� ��ERR_NONE����ͬ�Ľ��
        }
        
        if(res == ERR_RegsCompare_0)       //!< �յ��˵���Ļظ��������ǱȽϲ����󣬲��������õĲ���ȷ�����ǿ����õ�reg(���˼��ٱ�)����Ҫ���㲥����������
        {
            CfgRegCompRes |= (0x00000001<<(i+sBit));                //!< ����Դ�����������㲥дָ�����ݰ�
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

    if((mPrint&MskBit)!=0)           //!< ���ڼ��ٱȵȼĴ���������ֱ�Ӹ�������ֵ����˽����ڴ�ӡ�˴�����Ϣ
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
* @details ����ϵͳ�洢�ĵ�����ã�����cfgReg[7-9]˳����д�Ĵ�����ֵ
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

    if(pmCtrl->mCtrlMode == Ctrl_Mode_Posit)                //reg 8 Ŀǰֻ֧��λ��or�ٶ�
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

    //!< ���Ӧ��ģʽ
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
    
    if(pmCtrl->mAppMode != appMd)           //!< �ɸ��ļĴ���
    {
        return ERR_RegsCompare_0;
    }
    
    //!< ������ģʽ
    if(pReg[1] == 0)                        //!< reg 8,�������ģʽĿǰֻ�õ����ٶȺ�λ��ģʽ��Ŀǰֻ������������� 
    {
        ctlMd = Ctrl_Mode_Speed;
    }
    else if(pReg[1] == 1)
    {
        ctlMd = Ctrl_Mode_Posit;
    }
    else
    {
        return ERR_RegsCompare_0;           //!< �ɸ��ļĴ���
    }

    if(pmCtrl->mCtrlMode != ctlMd)
    {
        return ERR_RegsCompare_0;           //!< �ɸ��ļĴ���
    }
    
    //!< ���λ�ñջ���������           //!< ����ģʽ�²űȽ�λ�ñջ���������
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
            return ERR_RegsCompare_0;           //!< �ɸ��ļĴ���
        }

        if(pmCtrl->mPosClsedLoopCtrlMode != PosLoopCtrlMd)
        {
            return ERR_RegsCompare_0;           //!< �ɸ��ļĴ���
        }
    }   
    
    //!< �����ٱ�
    mReRt = pReg[4] + (pReg[5]<<8);

    if(((*(pmCtrl->pMotorRatio+mIdx))*100)!=mReRt)
    {
        return ERR_RegsCompare_1;               //!< ���ɸ��ļĴ���
    }

    return ERR_NONE;
    
}

/* --------------------------------------------------------------------------*/
/**
* @name sysEmbCheckShoulderOrWheelMotorConfigRegs
* @brief 
* @details ��Ҫ�Ǽ����or���ӵ���ĵ������ģʽ�Ĵ�����
*          (1)�����Ϊλ��or�ٶ�ģʽ������λ���ڱ����ȫ�ֱ�����ͬ���������λ��
*             �ڱ����ȫ�ֱ���
*          (2)�����Ϊ����ģʽʱ������λ�������ȫ�ֱ�����ΪĬ�ϵ�λ��ģʽ����
*             ����������λ�������ȫ�ֱ�������������޸ļĴ�����ָ��
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
    
    //!< ������ģʽ
    if(pReg[1] == 0)                        //!< reg 8,�������ģʽĿǰֻ�õ����ٶȺ�λ��ģʽ��Ŀǰֻ������������� 
    {
        ctlMd = Ctrl_Mode_Speed;
    }
    else if(pReg[1] == 1)
    {
        ctlMd = Ctrl_Mode_Posit;        
    }
    else
    {
        ctlMd = Ctrl_Mode_Posit;            //!< Ŀǰ���&���ӵ��ֻ�õ����ٶ�orλ��ģʽ������ģʽĬ���޸�Ϊλ��ģʽ(��Ϊ�󲿷ֵ�ʱ���ڶ�����ʹ��λ��ģʽ)
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
* @details дAT24C1024����������дʱ����Щ�����������ٸ���д���ݵ���Ŀ���Ż�
*          program time��
*
* @param[in] addr AT24C1024B�ĵ�ַ(at24����Ϊ128k�ֽ�.˳��д�������Ϊ256bytes��
*                 ����EEPROM�ڲ���ַ����ع�)
*
* @returns nums ֧�ֳ���256�ֽڵĶ�д����
* 
* @author Ling
*/
/* --------------------------------------------------------------------------*/
s8 SysE2PromWrite(u32 addr,u8* pdat,u32 nums)
{    
    u8 paddr0;
    u16 num0,num1;  //!< paddr0&num0Ϊ��ҳ��ַ����ҳ��д���ݸ���,num1Ϊβҳ�����ݸ���
    s8 res=ERR_NONE;
    u8 pg,i;
    
    //!< �жϵ�ַ�Ƿ���
    if(addr>EEPROM_ADDRESS_MAX)
    {
        return ERR_INPUT_PARAMETERS;
    }

    //!< �ж�д�����ݸ������ַ���ܺ��Ƿ񳬹��˵�ǰpage boundary
    paddr0 = addr%EEPROM_PAGE_CAPACITY;
    
    if((paddr0+nums)<=EEPROM_PAGE_CAPACITY)    //!< ���д�д�����������ͬ��ҳ�ڣ�ֱ�Ӷ������ؽ��
    {
        res = I2C_Write(addr,pdat,nums);
    }
    else                    //!< ��д������ݿ�ҳ��
    {
        num0 = (EEPROM_PAGE_CAPACITY - paddr0); //!< ��ȡ��ҳ���ݸ�����ǰ�汣֤paddr0��С��EEPROM_PAGE_CAPACITY��
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
* @param[in] addr AT24C1024B�ĵ�ַ(at24����Ϊ128k�ֽ�.˳����������Ϊ256bytes��
*                 ����EEPROM�ڲ���ַ����ع�)
*
* @returns nums ֧�ֳ���256�ֽڵĶ�д����
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 SysE2PromRead(u32 addr,u8* pdat,u32 nums)
{
    u8 paddr0;
    u16 num0,num1;  //!< paddr0&num0Ϊ��ҳ��ַ����ҳ�����ݸ���,num1Ϊβҳ�����ݸ���
    s8 res=ERR_NONE;
    u8 pg,i;
    
    //!< �жϵ�ַ�Ƿ���
    if(addr>EEPROM_ADDRESS_MAX)
    {
        return ERR_INPUT_PARAMETERS;
    }

    //!< �ж϶����ݸ������ַ���ܺ��Ƿ񳬹��˵�ǰpage boundary
    paddr0 = addr%EEPROM_PAGE_CAPACITY;
    
    if((paddr0+nums)<=EEPROM_PAGE_CAPACITY)    //!< ��������������ͬ��ҳ�ڣ�ֱ�Ӷ������ؽ��
    {
        res = I2C_Read(addr,pdat,nums);
    }
    else                    //!< ���������ݿ�ҳ��
    {
        num0 = (EEPROM_PAGE_CAPACITY - paddr0); //!< ��ȡ��ҳ���ݸ�����ǰ�汣֤paddr0��С��EEPROM_PAGE_CAPACITY��
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
        
    //!< ���ָ���� 
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
* @details ��������Ƿ�ΪPSMPSU��Э��ͷ
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
* @details �����������Ƿ����Ҫ��:���ID,fCode,Len,��ͨ����д���ݵ���Ӧbuffer
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

    //!<���ID
    if((pDat[2]!=PSM_SlaveAddr)&&(pDat[2]!=BatteryCharger_SlaveAddr))
    {
        return ERR_RX_DATA_ADDR;
    }

    if(fg==PSM_PACKET_HEADER)
    {
    //!< DD DD 01 08 03 00 crc1 crc2
    //!< DD DD 01 27 02 00 xx xx....crc1 crc2
        res = g_GetPsmStatus(&EwayPSM,pDat,len);    //!< len�Ѿ�����һ�������м�������        
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

    //!<  usRxWrite �������жϺ����б���д���������ȡ�ñ���ʱ����������ٽ������� 
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();

    //!< ��COM7���ջ����������������ݳ��ȴ��ڰ�ͷ�ĳ��ȣ���ʼ�������
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
            pUart->usRxCount--;                    //!< ����ͷ����ȷ������1�ֽ����ݣ�����һ�ֽڿ�ʼ����Ѱ�Ұ�ͷ
            pUart->usRxRead++;
            if(pUart->usRxRead>=UART7_RX_BUF_SIZE)  
            {
                pUart->usRxRead = 0;
            }
            ENABLE_INT();

            usCount -= 1;

            continue;
        }

        //!< get the data pkt length                     //!< ���ݽ��ջ��������ݵĳ��Ȼ�ȡ������
        if((pUart->usRxRead + 3) < UART7_RX_BUF_SIZE)    //!< fLen��bufferβ֮ǰ
        {
            fLen = *(pbuf+3);
        }
        else
        {
            tmp1 = UART7_RX_BUF_SIZE - pUart->usRxRead;        //!< usRxRead+3 = fLenλ

            fLen = *(pUart->pRxBuf+3-tmp1);
        }

        //!< check data pkt length
        if( usCount < fLen )    //!< ��ǰ�յ������ݰ����������˳��������������ٽ���
        {
            break;
        }

        if(fLen < PSM_RESPONSE_INFO_LEN_MIN)
        {
            DISABLE_INT();
            pUart->usRxCount--;                    //!< �����ݳ��Ȳ���ȷ������1�ֽ����ݣ�����һ�ֽڿ�ʼ����Ѱ�Ұ�ͷ
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
        if((pUart->usRxRead+fLen) <= UART7_RX_BUF_SIZE)                 //!< ��ȡ��������
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

        unCheckCRC=CRC16_Modbus(dat, fLen-2);                //����CRCУ��λ

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

        if(g_DataRecvdFromPSMPSUprocess(flag,dat,fLen)==ERR_NONE)        //!< �����������Ƿ����Ҫ��:���ID�����fCode,���fLen�Ƿ���,��ͨ����д���ݵ���Ӧbuffer
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

    //!<  usRxWrite �������жϺ����б���д���������ȡ�ñ���ʱ����������ٽ������� 
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();

    //!< ��COM8���ջ����������������ݳ��ȴ��ڰ�ͷ�ĳ��ȣ���ʼ�������
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
            pUart->usRxCount--;                    //!< ����ͷ����ȷ������1�ֽ����ݣ�����һ�ֽڿ�ʼ����Ѱ�Ұ�ͷ
            pUart->usRxRead++;
            if(pUart->usRxRead>=UART8_RX_BUF_SIZE)  
            {
                pUart->usRxRead = 0;
            }
            ENABLE_INT();

            usCount -= 1;

            continue;
        }

        //!< get the data pkt length                     //!< ���ݽ��ջ��������ݵĳ��Ȼ�ȡ������
        if((pUart->usRxRead + 3) < UART8_RX_BUF_SIZE)    //!< fLen��bufferβ֮ǰ
        {
            fLen = *(pbuf+3);
        }
        else
        {
            tmp1 = UART8_RX_BUF_SIZE - pUart->usRxRead;        //!< usRxRead+3 = fLenλ

            fLen = *(pUart->pRxBuf+3-tmp1);
        }

        //!< check data pkt length
        if( usCount < fLen )    //!< ��ǰ�յ������ݰ����������˳��������������ٽ���
        {
            break;
        }

        if(fLen < MIC_RESPONSE_INFO_LEN_MIN)
        {
            DISABLE_INT();
            pUart->usRxCount--;                    //!< �����ݳ��Ȳ���ȷ������1�ֽ����ݣ�����һ�ֽڿ�ʼ����Ѱ�Ұ�ͷ
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
        if((pUart->usRxRead+fLen) <= UART8_RX_BUF_SIZE)                 //!< ��ȡ��������
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

        unCheckCRC=CRC16_Modbus(dat, fLen-2);                //����CRCУ��λ

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

        if(g_DataRecvdFromMicprocess(dat,fLen)==ERR_NONE)        //!< �����������Ƿ����Ҫ��:���ID�����fCode,���fLen�Ƿ���,��ͨ����д���ݵ���Ӧbuffer
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
��ֻ���õ������ģʽ�����Ӧ��ģʽ�����ٱȣ�λ�ñջ��������ͺ�������
*/
s8 EmbConfigMotorsParametersForArms(u8 stId,u8 nums,EwayEmbSysMotorInfoModule* pMotor)
{
    u8 i,j;
    NodeCtrlModule* pMotCtrl = &EwayArms.mControl;

    if((nums>Motor_Num_Arm)||(pMotor == NULL))
    {
        return ERR_INPUT_PARAMETERS;
    }

    //���Ӧ��ģʽ���ã�Ĭ���ֱ�Ϊ�ؽ�ģʽ
    pMotCtrl->mAppMode = App_Mode_Joint;

    //�������ģʽ���ã�Ĭ��Ϊλ��ģʽ
    pMotCtrl->mCtrlMode = Ctrl_Mode_Posit;

    //��������ļ��ٱ�
    for(i=(stId-1);i<nums;i++)
    {
        j = (u8)((pMotor+i)->mReductionRatioMode);

        if(j>((u8)tWhel))     //!< �����ٱȲ���ȷ����Ĭ��Ϊ1
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
    
    //���Ӧ��ģʽ���ã�Ĭ��ͷ��Ϊ�ؽ�ģʽ
    pMotCtrl->mAppMode = App_Mode_Joint;
    
    //�������ģʽ���ã�Ĭ��Ϊλ��ģʽ
    pMotCtrl->mCtrlMode = Ctrl_Mode_Posit;
    
    //��������ļ��ٱ�
    for(i=(stId-Emb_StartID_Head);i<nums;i++)
    {
        j = (u8)((pMotor+i)->mReductionRatioMode);
    
        if(j>((u8)tWhel))     //!< �����ٱȲ���ȷ����Ĭ��Ϊ1
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
    
    //���Ӧ��ģʽ���ã�Ĭ�ϼ粿Ϊ����ģʽ
    pMotCtrl->mAppMode = App_Mode_Wheel;
    
    //�������ģʽ���ã�Ĭ��Ϊ�ٶ�ģʽ
    pMotCtrl->mCtrlMode = Ctrl_Mode_Speed;
    
    //��������ļ��ٱ�
    for(i=(stId-Emb_StartID_Shoulder);i<nums;i++)
    {
        j = (u8)((pMotor+i)->mReductionRatioMode);
    
        if(j>((u8)tWhel))     //!< �����ٱȲ���ȷ����Ĭ��Ϊ1
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
    
    //���Ӧ��ģʽ���ã�Ĭ������Ϊ����ģʽ
    pMotCtrl->mAppMode = App_Mode_Wheel;
    
    //�������ģʽ���ã�Ĭ��Ϊλ��ģʽ
    pMotCtrl->mCtrlMode = Ctrl_Mode_Posit;
    
    //��������ļ��ٱ�
    for(i=(stId-Emb_StartID_Wheel);i<nums;i++)
    {
        j = (u8)((pMotor+i)->mReductionRatioMode);
    
        if(j>((u8)tWhel))     //!< �����ٱȲ���ȷ����Ĭ��Ϊ1
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
            osDelay(100);       //!< �������ݲ��ɹ�������ʱ���ٴζ�
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
        
    //!< ����������ָ���ִ�л���
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

s8 EmbEnableLastPosCmdAvailableFlag(Motor_LastPosCmd* pLstPos,u8 Idx)       //!< ���յ��ٶ�ָ��������Ĺ���
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
    
    //!< �����ֱ۵�������һ��λ��ָ��
    for(i=0;i<Motor_Num_Arm;i++)
    {
        EmbUpdateLastPosCmdSendtoMotor(EwayArms.mControl.pmlstPos,i);
    }

    //!< ����ͷ����������һ��λ��ָ��
    for(i=0;i<Motor_Num_Head;i++)
    {
        EmbUpdateLastPosCmdSendtoMotor(EwayHeads.mControl.pmlstPos,i);
    }

    //!< ���¼���������һ��λ��ָ��


    //!< �������ӵ����һ��λ��ָ��
}


/* --------------------------------------------------------------------------*/
/**
* @name EmbRecordCmdSendToMotor
* @brief 
* @details ��¼�·������(�������ӵ��)��ָ��������ID������,ָ�����ͣ��ٶȣ�λ�ã����ָ��Ļ����ַ��
*   �洢�·���������˶�ָ������λ�������еĸ�ʽ
*
*               tStamp(4) ID(2) insType(0/1)  DnPos(2/4)  DnSpd(2)    UpPos(2/4)  UpSpd(2)
*�ֱۡ�ͷ��(14)   4        2        0             2          2            2          2
*
*�������(19)   4        2        1             4          2            4          2
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

    //!< ����豸����
    if((devType != EWAYBOT_SHOULDER)&&(devType != EWAYBOT_WHEEL))
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(sysGetMotorSendRecordPointer(devType,&pRecordBuff,&maxId,&startId)==ERR_NONE)
    {           
        //!< ���ID��
        if(Idx > maxId)
        {
            return ERR_INPUT_PARAMETERS;
        }

        //!< ���������ָ������
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
        
        //!< ����Ӧָ��水�չ涨�ĸ�ʽ��������  
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



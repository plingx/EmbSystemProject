/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Emb_PC.c
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
* 2017/12/20 | 0.0.1 | Ling     | Create file
*
*/

#include "includes.h"
#include "../../Module/PSM/PSM_cmd.h"
#include "Emb_PC.h"
#include "../../Module/PSM/PSM.h"
#include "PSM_Status.h"
#include "PSM_Cmd.h"
#include "BatteryCharger_Status.h"



extern u8 tcp_server_senbuf1[];
extern QueueHandle_t PCToShoulderQueHdl;
extern QueueHandle_t PCToArmsQueHdl;;
extern QueueHandle_t PCToHeadsQueHdl;
extern QueueHandle_t PCToWheelsQueHdl;
extern QueueHandle_t PCToGrippersQueHdl;
extern QueueHandle_t PCToEmbSysQueHdl;

extern osSemaphoreId NetSend_BinarySemHandle;


extern __IO u32 SysTimeStamp;
extern EwayIMUModule EwayImu;
extern EwayArmModule EwayArms;
extern EwayHeadModule EwayHeads;
extern EwayWheelModule EwayWheels;
extern EwayGripperModule EwayGrippers;
extern EwayEmbSysModule EwayEmbSys;
extern EwayShoulderModule EwayShoulder;
extern EwayRDSModule EwayRDS;
extern EwayEmbTCPModule EwayEmbTCP;
extern EwayPSMModule EwayPSM;
extern EwayBatteryChargerModule EwayBatteryCharger;
extern EwayRadarObstAvoidModule EwayRadarObstAvoid;
extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];
extern EwayEmbSysInfoModule EwayEmbSysInfo;
extern EwayEmbSysDebugModule* pDebug;
extern EwayRadarObstAvoidModule EwayRadarObstAvoid;
extern EwayMicModule EwayMic;
extern u8 tcp_server_recbuf[TCP_SERVER_RX_BUFSIZE];



extern osMutexId PCCmdBuffOpHandleAml[Motor_Num_ArmLeft];
extern osMutexId PCCmdBuffOpHandleAmr[Motor_Num_ArmRight];
extern osMutexId PCCmdBuffOpHandleShd;
extern osMutexId PCCmdBuffOpHandleHed[Motor_Num_Head];
extern osMutexId PCCmdBuffOpHandleWhl[Motor_Num_Wheel];
extern osMutexId PCCmdBuffOpHandleGrp[Motor_Num_Gripper];

extern osMutexId NetCommSendOpHandle;




s8 sysGetSingleCycleJointMotorsStatus(u8* pdat,u16* plen);
s8 sysGetSingleCycleWheelMotorsStatus(u8* pdat,u16* plen);



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
s8 l_GetPcCmdHeader(u16 hd)
{    
    if(hd == PC_CMD_FRAME_HEADER)
    {
        return ERR_NONE;
    }
    else
    {
        return ERR_INPUT_PARAMETERS;
    }
}



s8 l_EmbProcessPcRecvdDat(u8 *pdat,u16 Nums)
{
    u8 dat[150]={0};
    u8* pbuf;
    u16 fLen;
    u16 header;
    u16 dNum = Nums;
    u16 dread=0;        //!< ��ָ�����

    //!< ��tcp���ջ����������������ݳ��ȴ��ڰ�ͷ�ĳ��ȣ���ʼ�������
    //while ((p->tcp_recWrite!=p->tcp_recRead)&&(p->tcp_recCount>PC_CMD_HEAD_LENTH))
    while (dNum > PC_CMD_HEAD_LENTH)                        //!< �����ݳ��Ȳ���Э���ͷʱ������
    {        
        pbuf = pdat + dread;                                //!< ��ʼ�����������ݵ�ͷָ��

        header = pbuf[0]+(pbuf[1]<<8);                      //!< get Header
        
        if(l_GetPcCmdHeader(header)!=ERR_NONE)             //!< find the tcp pkt header
        {
            dNum--;                                         //!< ����ͷ����ȷ������1�ֽ����ݣ�����һ�ֽڿ�ʼ����Ѱ�Ұ�ͷ
            dread++;
/*
            if(EwayEmbTCP.runTimeout > 130)
            {
                Bsp_printf("P0,len:%d,wr:%d,rd:%d.%x-%x",tcp_server_buffer.tcp_recCount,tcp_server_buffer.tcp_recWrite,tcp_server_buffer.tcp_recRead,\
                    pbuf[0],pbuf[1]);
            }*/
                
            continue;
        }


        //!< get the data pkt length
        fLen = pbuf[8]+(pbuf[9]<<8);                        //!< FE E1 10 01 xx xx xx xx lenL lenH dat0 dat1 dat2 ........
        
        //!< check data pkt length(1)        
        if((fLen>PC_CMD_PKT_LEN_MAX)||(fLen<=PC_CMD_HEAD_LENTH))
        {
            dNum--;  //!< ���ݳ��Ȳ�����Ҫ�󣬶���������Ѱ����һ���ϸ����ݰ�
            dread++;
            
            continue;
        }

        //!< check data pkt length(2)
        if(dNum < fLen)    //!< ��2�ֿ��ܣ�(1)��ǰ�յ������ݰ������� ----> �����˰�����
        {                  //!<           (2) ��ǰ�յ���fLen������Ч���ݰ��е�fLen,֮ǰ�ļ��ֻ��ǡ�÷���Ҫ�� ----> ����������Ѱ����һ���ϸ�����ݰ�
            dNum--;
            dread++;
            
            continue;
        }

        memcpy(dat,pbuf,fLen);                 //!< ��ȡ��������

        //!< check data integrity
        if(sysCmdRecvdFromPCprocess(dat,fLen)==ERR_NONE)    //!< �����������Ƿ����Ҫ��:fLen�Ƿ���,
        {
            //recCount,recRead
            dread += fLen;            
            dNum -= fLen;

            memset(dat,0,150);
            
            continue;
        }
        else  //!< check data error,go on at the next data sequence
        {
            dNum--;            
            dread++;

            memset(dat,0,150);
                            
            continue;            
        }                
    }

    return ERR_NONE;
}


/* --------------------------------------------------------------------------*/
/**
* @name sysExtractLegalPktFromTCP
* @brief 
* @details ��tcp���ջ����н�������ȷ�����ݰ�
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void sysExtractLegalPktFromTCP(EmbTcpRecvModule* pTcpRecv)
{
    u8 i;
    s8 rt=ERR_NONE;
    u16 dNum;

    for(i=0;i<2;i++)
    {
        if(tcpServerRecvGetReadyProcessedIdx(i)==ERR_NONE)       //!< ���buffer X ���Ƿ��д����������
        {                                                           //!< �п��е�buffer&�д����������
            pTcpRecv->state[i] = 2;                               //!< �����ڴ���ı�־
            
            memcpy(tcp_server_recbuf,pTcpRecv->pRev[i],pTcpRecv->rCnt[i]);

            dNum = pTcpRecv->rCnt[i];

            memset(pTcpRecv->pRev[i],0,pTcpRecv->rCnt[i]);

            pTcpRecv->rCnt[i] = 0;
            
            pTcpRecv->state[i] = 0;

            //!< �������ݴ�����
            rt = l_EmbProcessPcRecvdDat(tcp_server_recbuf,dNum);
            //Bsp_printf("l_EmbProcessPcRecvdDat(%d) len:%d.",i,dNum);

            if(rt!=ERR_NONE)
            {
                Bsp_printf("l_EmbProcessPcRecvdDat() rt:%d.",rt);
            }

            //!< �������
            memset(tcp_server_recbuf,0,TCP_SERVER_RX_BUFSIZE);            
        }        
    }

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
s8 sysCmdRecvdFromPCprocess(u8* pDat,u16 len)
{    
    //u32 fTimStamp;
    u16 fCode, fLen;
    BaseType_t rt=pdPASS;
    u8 dat[128]={0};
    
    fCode = pDat[2]+(pDat[3]<<8);

    //fTimStamp = pDat[4]+(pDat[5]<<8)+(pDat[6]<<16)+(pDat[7]<<24);

    fLen = pDat[8]+(pDat[9]<<8);

    if((len>PC_CMD_PKT_LEN_MAX)||(len<=PC_CMD_HEAD_LENTH))
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            Bsp_printf("PC COM recv data len error.L=%d",len);
        }
        return ERR_INPUT_PARAMETERS;
    }    
    
    switch(fCode&0xFFF0)
    {
        case PCCmd_ArmPosMoveMode:      //!< �ֱ۵�ָ�� 0x0110
            if(fLen<CMD_PC2ARMS_INFO_LEN)
            {
                fLen = len-2;               //!< ȥ�� FE E1
                memcpy(dat,(pDat+2),fLen);
                            
                dat[6] = (u8)fLen;          //!< modify the msg len
                dat[7] = (u8)(fLen>>8);    
                
                rt = xQueueSend(PCToArmsQueHdl,dat,0);
                
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_TcpRBuf_2_XQueue_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                {
                    Bsp_printf("1.1 rec PC to ArmsPkt,save to Que PCToArmsQueHdl,rt:%d,Len+2:%d.",rt,fLen+2);
                }
            }
            else
            {
                if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
                {
                    SysLogWrite(LogMajor,"1.1 rec PC to ArmsPkt,fLen exceed limit:%d.",fLen);
                                
                    Bsp_printf("1.1 rec PC to ArmsPkt,fLen exceed limit:%d.",fLen);
                }
            }
            
            break;

        case PCCmd_HeadPosMove:         //!< ͷ����ָ�� 0x0120  
            if(fLen<CMD_PC2HEAD_INFO_LEN)
            {
                fLen = len-2;               //!< ȥ�� FE E1
                memcpy(dat,(pDat+2),fLen);
                            
                dat[6] = (u8)fLen;          //!< modify the msg len
                dat[7] = (u8)(fLen>>8);    
                
                rt = xQueueSend(PCToHeadsQueHdl,dat,0);
                
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_TcpRBuf_2_XQueue_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
                {
                    Bsp_printf("1.1 rec PC to HdPkt,save to Que PCToHeadsQueHdl,rt:%d,Len:%d.",rt,fLen+2);
                }
            }
            else
            {
                if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
                {
                    SysLogWrite(LogMajor,"1.1 rec PC to HdPkt,fLen exceed limit:%d.",fLen);
                                                
                    Bsp_printf("1.1 rec PC to HdPkt,fLen exceed limit:%d.",fLen);
                }
            }
            
            break;

        case PCCmd_WheelPositionMove:   //!< ���ӵ�ָ�� 0x0130    
            if(fLen<CMD_PC2WHEEL_INFO_LEN)
            {
                fLen = len-2;               //!< ȥ�� FE E1
                memcpy(dat,(pDat+2),fLen);
                            
                dat[6] = (u8)fLen;          //!< modify the msg len
                dat[7] = (u8)(fLen>>8);    
                
                rt = xQueueSend(PCToWheelsQueHdl,dat,0);
                
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_TcpRBuf_2_XQueue_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                {
                    Bsp_printf("1.1 rec PC to WheelPkt,save to Que PCToWheelsQueHdl,rt:%d,Len:%d.",rt,fLen+2);
                }            
            }
            else
            {
                if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
                {
                    SysLogWrite(LogMajor,"1.1 rec PC to WheelPkt,fLen exceed limit:%d.",fLen);
                                                                
                    Bsp_printf("1.1 rec PC to WheelPkt,fLen exceed limit:%d.",fLen);
                }
            }
            
            break;

        case PCCmd_GripperMove:         //!< צ�ӵ�ָ�� 0x0140  
            if(fLen<CMD_PC2GRIPPER_INFO_LEN)
            {
                fLen = len-2;               //!< ȥ�� FE E1
                memcpy(dat,(pDat+2),fLen);
                            
                dat[6] = (u8)fLen;          //!< modify the msg len
                dat[7] = (u8)(fLen>>8);    
                
                rt = xQueueSend(PCToGrippersQueHdl,dat,0);
                
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_TcpRBuf_2_XQueue_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Gripper_MASK))
                {
                    Bsp_printf("1.1 rec PC to GripperPkt,save to Que PCToGrippersQueHdl,rt:%d,Len+2:%d.",rt,fLen+2);
                }
            }
            else
            {
                if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
                {
                    SysLogWrite(LogMajor,"1.1 rec PC to GripperPkt,fLen exceed limit:%d.",fLen);
                                                                                
                    Bsp_printf("1.1 rec PC to GripperPkt,fLen exceed limit:%d.",fLen);
                }
            }
            
            break;

        case PCCmd_ShoulderPosMoveMode: //!> ����ָ�� 0x0160
        
            if(fLen<CMD_PC2SHOULDER_INFO_LEN)
            {               
                fLen = len-2;               //!< ȥ�� FE E1
                memcpy(dat,(pDat+2),fLen);
                            
                dat[6] = (u8)fLen;          //!< modify the msg len
                dat[7] = (u8)(fLen>>8);    
                
                rt = xQueueSend(PCToShoulderQueHdl,dat,0);
                
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_TcpRBuf_2_XQueue_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                {
                    Bsp_printf("1.1 rec PC to ShlderPkt,save to Que PCToShoulderQueHdl,rt:%d,Len+2:%d.",rt,fLen+2);
                }
            }
            else
            {
                if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
                {
                    SysLogWrite(LogMajor,"1.1 rec PC to ShlderPkt,fLen exceed limit:%d.",fLen);
                
                    Bsp_printf("1.1 rec PC to ShlderPkt,fLen exceed limit:%d.",fLen);
                }
            }

            break;
            
        default://!< system cmd
#if  Emb_LaserCtrl_Enable
            if((fCode==PCCmd_Clear)||(fCode==PCCmd_ObstclAvoid)||(fCode==PCCmd_SysStop)||(fCode==PCCmd_Led)||\
                (fCode==PCCmd_DebugCtrl)||(fCode==PCCmd_HeartBeat)||(fCode==PCCmd_Laser))
#else
            if((fCode==PCCmd_Clear)||(fCode==PCCmd_ObstclAvoid)||(fCode==PCCmd_SysStop)||(fCode==PCCmd_Led)||(fCode==PCCmd_DebugCtrl)||(fCode==PCCmd_HeartBeat))
#endif
            {
                
                fLen = len-2;               //!< ȥ�� FE E1
                memcpy(dat,(pDat+2),fLen);
            
                dat[6] = (u8)fLen;          //!< modify the msg len
                dat[7] = (u8)(fLen>>8);    

                rt = xQueueSend(PCToEmbSysQueHdl,dat,0);

                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_TcpRBuf_2_XQueue_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_EmbSystem_MASK))
                {
                    Bsp_printf("1.1 rec PC to EmbSysPkt,save to Que PCToEmbSysQueHdl,rt:%d,Len:%d,Code:0x%x.",rt,fLen+2,fCode);
                }
            }
            else
            {
                rt = osErrorParameter;
                if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
                {
                    SysLogWrite(LogMajor,"1.1 rec Invalid Pc To Emb Pkt,Len:%d,fCode:0x%04x.",fLen+2,fCode);

                    Bsp_printf("1.1 rec Invalid Pc To Emb Pkt,Len:%d,fCode:0x%04x.",fLen+2,fCode);
                }
            }
            break;    
    }

    if(rt == pdPASS)
        return ERR_NONE;
    else
        return ERR_PROCESS_FAILED;    
}



/* --------------------------------------------------------------------------*/
/**
* @name sysPcToEmbCmdProcess
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
void sysPcToEmbCmdProcess(EwayMotor_Device_Type devType,QueueHandle_t* pQueHandle)
{
    u8 dat[PC_CMD_PKT_LEN_MAX]={0};
    u8 res;
    u16 fCode,fLen;
    u32 fTimStamp;    

    //!< dat: XX 1X 12 34 56 78 L0 L1 iD0 iD1 Pos0 Pos1 Pos2 Pos3 S0 S1   (��ȥ��FE E1)
    while(xQueueReceive((*pQueHandle),dat,0) == pdPASS)    //PCToShoulderQueHdl
    {
        fCode = dat[0] + (dat[1]<<8);
        fTimStamp = dat[2] + (dat[3]<<8) + (dat[4]<<16) + (dat[5]<<24);
        fLen = dat[6] + (dat[7]<<8);

        if((fLen<=(PC_CMD_HEAD_LENTH-2))||(fLen>=PC_CMD_PKT_LEN_MAX))
        {
            memset(dat,0,PC_CMD_PKT_LEN_MAX);
            
            continue;      
        }
                                                    
        switch(devType)                   //!< fLenȥ�� fCode 2byte,fTimStamp 4 bytes
        {
            case EWAYBOT_SHOULDER:
                    res = sysPcToShoulderCmdProcess(fCode,fTimStamp,(fLen-8),&dat[8]);   
                break;

            case EWAYBOT_ARMS_LEFT:
            case EWAYBOT_ARMS_RIGHT:
                    res = sysPcToArmsCmdProcess(fCode,fTimStamp,(fLen-8),&dat[8]);   //!< ����ȥ��fCode(2) fTimeStamp(4) fLen(2),����Ϊdat�ĳ���
                break;
            case EWAYBOT_HEAD:
                    res = sysPcToHeadsCmdProcess(fCode,fTimStamp,(fLen-8),&dat[8]);   //!< ����ȥ��fCode(2) fTimeStamp(4) fLen(2),����Ϊdat�ĳ���
                break;

            case EWAYBOT_GRIPPERS:
                    res = sysPcToGrippersCmdProcess(fCode,fTimStamp,(fLen-8),&dat[8]);   //!< ����ȥ��fCode(2) fTimeStamp(4) fLen(2),����Ϊdat�ĳ���
                break;

            case EWAYBOT_WHEEL:
                res = sysPcToWheelsCmdProcess(fCode,fTimStamp,(fLen-8),&dat[8]);   //!< ����ȥ��fCode(2) fTimeStamp(4) fLen(2),����Ϊdat�ĳ���
                break;
                
            case EWAYBOT_EMB:
                res = sysPcToEmbsysCmdProcess(fCode,fTimStamp,(fLen-8),&dat[8]);   //!< ����ȥ��fCode(2) fTimeStamp(4) fLen(2),����Ϊdat�ĳ���
                break;
                
            default:
                break;
        }

        memset(dat,0,PC_CMD_PKT_LEN_MAX);                   //!< clr data buffer
        
    }

    //return ERR_NONE;
}


/* --------------------------------------------------------------------------*/
/**
* @name sysSendEmbInfoToPC
* @brief ������λ�����״̬����λ��
* @details 
*            1.������������Ƿ�����
*            2.�������
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void sysSendEmbInfoToPC(void)
{
    u8 dat[1024]={0};
    u16 len;
    u8 res=ERR_NONE;
    u16 rf1=0,rf2=0;
    osStatus re;
    //BaseType_t rt;

    ////  1. ������������Ƿ�����
    if((EwayEmbSys.Comm.wPc & 0x01) == EMB_COMM_STATUS_DISCONNECTED)
    {
        //return ERR_PROCESS_FAILED;
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            //SysLogWrite(LogMajor,"6.1 Comm with PC is disconnected.");

            //Bsp_printf("6.1 Comm with PC is disconnected.");
        }
    }
    else
    {
        if(EwayEmbTCP.tcp_tmp.bsy != 0)  //!< ����Ƿ����ڷ���
        {
            //return ERR_IS_BUSY;
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                SysLogWrite(LogMajor,"6.1 Snd Info To PC failed,EwayEmbTCP.TCP_Trans is busy.");

                Bsp_printf("6.1 Snd Info To PC failed,EwayEmbTCP.TCP_Trans is busy.");
            }
        }
        else
        {
            //!< ���������
            EwayEmbTCP.TCP_Trans.SndCnt = 0;
            memset(EwayEmbTCP.TCP_Trans.pSnd,0,TCP_SERVER_RX_BUFSIZE);
            
            ////  2. �������
            memset(dat,0,100);len=0;
            res = sysGetPSUInfo(dat,&len);                          //!< PSU info 0x0000    23 bytes
            if((res==ERR_NONE)&&(len>0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);
                if(ERR_NONE != res)
                {                    
                    rf2 |= 0x0001;
                }
            }
            else if(res != ERR_NONE)
            {              
                rf1 |= 0x01;
            }

            memset(dat,0,100);len=0;
            res = sysGetPSMInfo(dat,&len);                         //!< PSM info    0x0001    42bytes
            if((res==ERR_NONE)&&(len>0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);
                if(ERR_NONE != res)
                {
                    rf2 |= 0x0002;
                }
            }
            else if(res != ERR_NONE)
            {
                rf1 |= 0x0002;
            }

            memset(dat,0,100);len=0;
            res = sysGetImuInfo(dat,&len);                        //!< IMU info     0x0002      30 bytes
            if((ERR_NONE == res)&&(len>0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);
                if(ERR_NONE != res)
                {
                    rf2 |= 0x0004;
                }
            }
            else if(res != ERR_NONE)
            {
                rf1 |= 0x0004;
            }

            memset(dat,0,100);len=0;                  
            res = sysGetRadarInfo(dat,&len);                   //!< Radar Info    0x0003    24 bytes
            if((res==ERR_NONE)&&(len>0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);
                if(ERR_NONE != res)
                {
                    rf2 |= 0x0008;
                }
            }
            else if(res != ERR_NONE)
            {
                rf1 |= 0x0008;
            }

            memset(dat,0,100);len=0;
            res = sysGetRDSInfo(dat,&len);                       //!< RDSensor info 0x0004    23 bytes    
            if((res==ERR_NONE)&&(len>0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);
                if(ERR_NONE != res)
                {
                    rf2 |= 0x0010;
                }
            }
            else if(res != ERR_NONE)
            {
                rf1 |= 0x0010;
            }

#if (Emb_MIC_Debug)
            memset(dat,0,100);len=0;
            res = sysGetMICInfo(dat,&len);                      //!< Mic info 0x0005    16 bytes   
            if((res==ERR_NONE)&&(len>0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);
                if(ERR_NONE != res)
                {
                    rf2 |= 0x0020;
                }
            }
            else if(res != ERR_NONE)
            {
                rf1 |= 0x0020;
            }
#endif
            memset(dat,0,256);len=0;
            res = sysGetMotorInfo(dat,&len);                  //!< Motor Info  0x0010   220 bytes
            if((res==ERR_NONE)&&(len>0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);
                if(ERR_NONE != res)
                {
                    rf2 |= 0x0040;
                }
            }
            else if(res != ERR_NONE)
            {
                rf1 |= 0x0040;
            }

            memset(dat,0,100);len=0;
            res = sysGetShoulderMotorInfo(dat,&len);            //!< shoulder motor info 0x0011 27 bytes                
            if((res==ERR_NONE)&&(len>0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);
                if(ERR_NONE != res)
                {
                    rf2 |= 0x0080;
                }
            }
            else if(res != ERR_NONE)
            {
                rf1 |= 0x0080;
            }               

            memset(dat,0,100);len=0;
            res = sysGetWheelInfo(dat,&len);                  //!< WheelInfo  0x0020    sum = 44 bytes
            if((res==ERR_NONE)&&(len>0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);
                if(ERR_NONE != res)
                {
                    rf2 |= 0x0100; 
                }
            }
            else if(res != ERR_NONE)
            {
                rf1 |= 0x0100; 
            }

            memset(dat,0,100);len=0;
            res = sysGetGripperInfo(dat,&len);                     //!< GripperInfo 0x0030    sum = 40bytes
            if((res==ERR_NONE)&&(len>0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);        
                if(ERR_NONE != res)
                {
                    rf2 |= 0x0200; 
                }
            }
            else if(res != ERR_NONE)
            {
                rf1 |= 0x0200; 
            }

#if EMB_Upload_SingleCycle_Motor_Info_DEBUG
            memset(dat,0,200);len=0;
            res = sysGetSingleCycleJointMotorsStatus(dat,&len);          //!< SingleCycleArmsHeadsMotorsInfo 0x0040    sum = [10-150]bytes
            if((res == ERR_NONE)&&(len != 0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);        
                if(ERR_NONE != res)
                {
                    rf2 |= 0x0400;
                }
            }
            else if(res != ERR_NONE)
            {
                rf1 |= 0x0400;
            }

            memset(dat,0,100);len=0;
            res = sysGetSingleCycleWheelMotorsStatus(dat,&len);          //!< SingleCycleShoulderWheelsMotorsInfo 0x0041    sum = 40bytes
            if((res == ERR_NONE)&&(len != 0))
            {
                res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);        
                if(ERR_NONE != res)
                {
                    rf2 |= 0x0800;                         
                }
            }
            else if(res != ERR_NONE)
            {
                rf1 |= 0x0800;                     
            }
#endif                


#if EMB_Upload_EmbSoftVersion
            if((EwayEmbSys.Comm.wPc&0x03)==0x01)        //!< ���ո�����λ����������ͨ�ţ�������λ������汾��
            {
                memset(dat,0,50);len=0;
                res = sysGetEmbSystemSoftwareVersion(dat,&len);
                if((res == ERR_NONE)&&(len != 0))
                {
                    res = sysGeneralWriteDataToTcpSendBuffer(dat,len,&EwayEmbTCP.TCP_Trans);        
                    if(ERR_NONE != res)
                    {
                        rf2 |= 0x0800;
                    }
                }
            }            
#endif

            if((rf1!=0)||(rf2!=0))
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    SysLogWrite(LogMajor,"6.1 Snd Info To PC() pack data err,rf1:0x%2x,rf2:0x%2x.",rf1,rf2);
                    Bsp_printf("6.1 Snd Info To PC() pack data err,rf1:0x%2x,rf2:0x%2x.",rf1,rf2);
                }
            }            
            
            ////  3. ������Ӧ���沢����֪ͨ�����緢������
            if((EwayEmbTCP.TCP_Trans.SndCnt>0)&&(EwayEmbTCP.tcp_tmp.bsy==0))        //!< ����Ƿ��д��������ݣ������ڷ��͵Ļ��洦�ڿ���
            {
                re = osMutexWait(NetCommSendOpHandle,1);

                if(re == osOK)
                {
                    memcpy(EwayEmbTCP.tcp_tmp.pSnd,EwayEmbTCP.TCP_Trans.pSnd,EwayEmbTCP.TCP_Trans.SndCnt);
                
                    EwayEmbTCP.tcp_tmp.SndCnt = EwayEmbTCP.TCP_Trans.SndCnt;

                    EwayEmbTCP.tcp_tmp.bsy = 1 ;                

                    osMutexRelease(NetCommSendOpHandle);    

                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_2_PC_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_NetCommSend_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Eth_MASK))
                    {
                        Bsp_printf("6.2 Copy tcp send data to tcp_tmp data buffer.Cnt:%d.",EwayEmbTCP.tcp_tmp.SndCnt);
                    }
                }
                else        //!< ��ȡ���ʧ��
                {                
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("6.2 Get NetCommSendOpHandle-3 failed.rt:%d.",re);
                    }
                }                
            }
            else if(EwayEmbTCP.tcp_tmp.bsy != 0)            //!< �ٴμ��
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    Bsp_printf("6.2 Tcp snd failed tcp_tmp.bsy:%d.",EwayEmbTCP.tcp_tmp.bsy);
                }
            }
        }    
    }
}

/* --------------------------------------------------------------------------*/
/**
* @name sysGeneralWriteDataToTcpSendBuffer
* @brief 
* @details дϵͳ�ĸ���״̬����Tcp�����ͻ�����
*
* @param[in]    pdat ��д������ָ��
*                len  ��д�����ݳ���
*
* @returns None
* 
* @author Ling
*/
/* --------------------------------------------------------------------------*/
s8 sysGeneralWriteDataToTcpSendBuffer(u8* pdat,u16 len,EmbTcpTransModule* pTrans)
{
    s8 res = ERR_NONE;

    if((NULL == pTrans)||(len>TCP_SERVER_RX_BUFSIZE)||(len < PC_CMD_HEAD_LENTH))
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(( len + pTrans->SndCnt) <= TCP_SERVER_RX_BUFSIZE )
    {
        memcpy((pTrans->pSnd + pTrans->SndCnt),pdat,len);

        pTrans->SndCnt += len;
    }
    else
    {
        res = ERR_DATA_OVERFLOW;
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
s8 sysGetImuInfo(u8* pdat,u16* plen)
{
    if((pdat==NULL)||(plen==NULL))
        return ERR_INPUT_PARAMETERS;
    
    pdat[0] = (u8)PC_CMD_FRAME_HEADER;
    pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
    pdat[2] = (u8)IMU_Status;
    pdat[3] = (u8)(IMU_Status>>8);
    pdat[4] = (u8)SysTimeStamp;
    pdat[5] = (u8)(SysTimeStamp>>8);
    pdat[6] = (u8)(SysTimeStamp>>16);
    pdat[7] = (u8)(SysTimeStamp>>24);
    pdat[8] = (u8)IMU_Status_Length ;
    pdat[9] = (u8)(IMU_Status_Length>>8);

    memcpy((pdat+10),(u8*)(&EwayImu.Res[0]),(EWAY_IMU_PARAM_NUMS<<1));  //!< ���սǶȡ����ٶȡ��ٶȵ�˳�򱣴�����ݣ���˿���ֱ��copy
    
    pdat[IMU_Status_Length-2] = (u8)(EwayImu.Comm.sta);    
    pdat[IMU_Status_Length-1] = (u8)((EwayImu.Comm.sta)>>8);

    *plen = (u8)IMU_Status_Length;

    return ERR_NONE;
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
s8 sysGetRDSInfo(u8* pdat,u16* plen)
{
    if((pdat==NULL)||(plen==NULL))
    {    
        return ERR_INPUT_PARAMETERS;
    }

    pdat[0] = (u8)PC_CMD_FRAME_HEADER;
    pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
    pdat[2] = (u8)RDS_Status;
    pdat[3] = (u8)(RDS_Status>>8);
    pdat[4] = (u8)SysTimeStamp;
    pdat[5] = (u8)(SysTimeStamp>>8);
    pdat[6] = (u8)(SysTimeStamp>>16);
    pdat[7] = (u8)(SysTimeStamp>>24);
    pdat[8] = (u8)RDS_Status_Length ;
    pdat[9] = (u8)(RDS_Status_Length>>8);

    if((EwayEmbSysInfo.Rds.En!=0)&&(EwayEmbSysInfo.Rds.init!=0))
    {
        pdat[10] |= RopeDisplacementSensorEn_MaskBit;

        pdat[11] = (u8)EwayEmbSysInfo.Rds.HigLmt;
        pdat[12] = (u8)(EwayEmbSysInfo.Rds.HigLmt>>8);
        pdat[13] = (u8)(EwayEmbSysInfo.Rds.HigLmt>>16);
        pdat[14] = (u8)(EwayEmbSysInfo.Rds.HigLmt>>24);

        pdat[15] = (u8)EwayEmbSysInfo.Rds.LowLmt;
        pdat[16] = (u8)(EwayEmbSysInfo.Rds.LowLmt>>8);
        pdat[17] = (u8)(EwayEmbSysInfo.Rds.LowLmt>>16);
        pdat[18] = (u8)(EwayEmbSysInfo.Rds.LowLmt>>24);
    }
    else
    {
        pdat[10] &= (~RopeDisplacementSensorEn_MaskBit);
        
        pdat[11] = 0x00;
        pdat[12] = 0x00;
        pdat[13] = 0x00;
        pdat[14] = 0x00;
        
        pdat[15] = 0x00;
        pdat[16] = 0x00;
        pdat[17] = 0x00;
        pdat[18] = 0x00;
    }
  

    pdat[19] = (u8)EwayRDS.Res;
    pdat[20] = (u8)(EwayRDS.Res>>8);
    pdat[21] = (u8)(EwayRDS.Res>>16);
    pdat[22] = (u8)(EwayRDS.Res>>24);
    
    *plen = (u8)RDS_Status_Length;
    
    return ERR_NONE;
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
s8 sysGetMICInfo(u8* pdat,u16* plen)
{
    if((pdat==NULL)||(plen==NULL))
    {    
        return ERR_INPUT_PARAMETERS;
    }
    
    pdat[0] = (u8)PC_CMD_FRAME_HEADER;
    pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
    pdat[2] = (u8)MIC_Status;
    pdat[3] = (u8)(MIC_Status>>8);
    pdat[4] = (u8)SysTimeStamp;
    pdat[5] = (u8)(SysTimeStamp>>8);
    pdat[6] = (u8)(SysTimeStamp>>16);
    pdat[7] = (u8)(SysTimeStamp>>24);
    pdat[8] = (u8)MIC_Status_Length ;
    pdat[9] = (u8)(MIC_Status_Length>>8);

    //!< ���ѽǶ�
    pdat[10] = (u8)EwayMic.MicInfo.wakeup_angle;
    pdat[11] = (u8)(EwayMic.MicInfo.wakeup_angle>>8);

    //!< �汾��Ϣ
    pdat[12] = (u8)EwayMic.MicInfo.verinfo[0];
    pdat[13] = (u8)EwayMic.MicInfo.verinfo[1];

    //!< ״̬
    pdat[14] = (u8)EwayMic.MicInfo.mic_online;
    pdat[15] = 0x00;

    return ERR_NONE;
}



s8 sysGetPSMInfo(u8* pdat,u16* plen)
{
    u8 uni;
    PSM sPsmStatus;
    
    if((NULL==pdat)||(NULL==plen))
    {
        return ERR_POINTER_NULL;
    }
    
    pdat[0] = (u8)PC_CMD_FRAME_HEADER;
    pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
    pdat[2] = (u8)PSM_Status;
    pdat[3] = (u8)(PSM_Status>>8);
    pdat[4] = (u8)SysTimeStamp;
    pdat[5] = (u8)(SysTimeStamp>>8);
    pdat[6] = (u8)(SysTimeStamp>>16);
    pdat[7] = (u8)(SysTimeStamp>>24);
    pdat[8] = (u8)PSM_Status_Length ;
    pdat[9] = (u8)(PSM_Status_Length>>8);

    if((EwayEmbSys.Comm.wPsm&EMB_COMM_ERR_BITS_3_TIMES) != 0)
    {
        memcpy((char*)(&sPsmStatus),(char*)(&EwayPSM.sPSM),sizeof(PSM));
    }
    else
    {
        memset((char*)(&sPsmStatus),0,sizeof(sPsmStatus));
    }

    //ReportPSMStatusToPC(&sPsmStatus,pdat,&uLen);

    //��Դ���ذ���·��ѹ����״̬:   0 sPC;   1 sLeftArm_PSM;   2 sRightArm_PSM;   3 sWheel_PSM;   4 sElse_PSM;   5 sDC5V_PSM;   6 sDC12V_PSM;
    for(uni=0;uni<7;uni++)
    {        
        pdat[uni*4+10] = (u8)sPsmStatus.sPSM_Out[uni].unVoltage;                //��ѹ����ֵ�Ŵ���100������λ0.01V//
        pdat[uni*4+11] = (u8)(sPsmStatus.sPSM_Out[uni].unVoltage>>8);
        
        /*��������ֵ�Ŵ���100������λ0.01A*/
        pdat[uni*4+12] = (u8)sPsmStatus.sPSM_Out[uni].unCurrent;
        pdat[uni*4+13] = (u8)(sPsmStatus.sPSM_Out[uni].unCurrent>>8);
    }
    
    uni = 6;
    
    //�����˵�ǰ����״̬��   0����δ����״̬��   1�����ѿ�������������   2�����ѿ�����   3��������״̬��   4����ͣ״̬��   5�ȴ��ػ�״̬
    pdat[uni*4+14] = sPsmStatus.unRobotCurrentStatus;

    //��·����״̬:      bit0������λ�������·,1����պϣ�0�����״̬��      Bit1������ۣ�  Bit2������̣�  Bit3�����ұۣ�  Bit4����������  Bit5����DCDC��  Bit6������λ���ѿ�����1���ѿ���
    pdat[uni*4+15] = sPsmStatus.unPSM_PowerSwitchStatus;

    //��Դ���ذ嵱ǰ״̬2:   bit1-0��������ģ�飻  00����������01����Ƿѹ��10���������   bit3-2����DCDC5V��·��   bit5-4����DCDC12V��·��
    pdat[uni*4+16] = sPsmStatus.unPSM_Status1;

    //��Դ���ذ嵱ǰ״̬2:  bit1-0��������ģ�飻  00����������01����Ƿѹ��10���������   bit3-2����DCDC5V��·��   bit5-4����DCDC12V��·��
    pdat[uni*4+17] = sPsmStatus.unPSM_Status2;

    *plen = (u8)PSM_Status_Length;

    return ERR_NONE;
}


/* --------------------------------------------------------------------------*/
/**
* @name sysGetPSUInfo
* @brief ��ȡPSU��Ϣ
* @details ��ȡPSU��״̬��Ϣ��������Ϣ����pdatָ��ĵ�ַ�����ȷ���plenָ���ַ
*
* @param[in] None
*
* @returns    pdat
*             plen
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysGetPSUInfo(u8* pdat,u16* plen)
{
    ModuleBatteryCharger  sPsuStatus;

    if((NULL==pdat)||(NULL==plen))
    {
        return ERR_POINTER_NULL;
    }
    
    pdat[0] = (u8)PC_CMD_FRAME_HEADER;
    pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
    pdat[2] = (u8)PSU_Status;
    pdat[3] = (u8)(PSU_Status>>8);
    pdat[4] = (u8)SysTimeStamp;
    pdat[5] = (u8)(SysTimeStamp>>8);
    pdat[6] = (u8)(SysTimeStamp>>16);
    pdat[7] = (u8)(SysTimeStamp>>24);
    pdat[8] = (u8)PSU_Status_Length ;
    pdat[9] = (u8)(PSU_Status_Length>>8);

    if((EwayEmbSys.Comm.wPsu&EMB_COMM_ERR_BITS_3_TIMES) != 0)
    {
        memcpy((char*)(&sPsuStatus),(char*)(&EwayBatteryCharger.sBattCharger),sizeof(ModuleBatteryCharger));
    }
    else
    {
        memset((char*)(&sPsuStatus),0,sizeof(sPsuStatus));
    }

    //ReportBatteryStatusToPC(&sPsuStatus,pdat,&uLen);
    ///�����ƹ�����״1      bit0�����ȣ�bit1����������������������10A����bit2:����Ƿѹ��bit3�������ѹ��bit4�����Ƿѹ��bit5�������·��bit6����ؼ�������رգ�bit7��δ�����������0�������޹���
    pdat[10] = sPsuStatus.unStatus;

    ///�����ƹ�����״2      bit0�����ع����Դ��0���ⲿ��Դ��1�ǵ�أ���bit1����س�����1�ǳ�������bit6-bit7��00��δ��磬01�ǳ�������10�ǳ���ѹ��11�ǳ�����������δ����λ��
    pdat[11] = (u8)(sPsuStatus.unStatus>>8);
    
    pdat[12] = sPsuStatus.unExVoltage;                      ///�ⲿ�����ѹ����ֵ�Ŵ���100������λ0.01V
    pdat[13] = (u8)(sPsuStatus.unExVoltage>>8);
    
    pdat[14] = sPsuStatus.unOutCurrent;                     ///���ص�������ֵ�Ŵ���100������λ0.01A
    pdat[15] = (u8)(sPsuStatus.unOutCurrent>>8);
    
    pdat[16] = (sPsuStatus.unTemperature);                  ///�����嵱ǰ�¶�
    
    pdat[17] = sPsuStatus.unBatteryVoltage;                 ///��ص�ѹ����ֵ�Ŵ���100������λ0.01V
    pdat[18] = (u8)(sPsuStatus.unBatteryVoltage>>8);
    
    pdat[19] = sPsuStatus.unBatteryInCurrent;               ///��س���������ֵ�Ŵ���100������λ0.01A
    pdat[20] = (u8)(sPsuStatus.unBatteryInCurrent>>8);
    
    pdat[21] = sPsuStatus.unBatteryCapacity;                ///��ص�ǰ��������ֵ�Ŵ���100������λ0.01%
    pdat[22] = (u8)(sPsuStatus.unBatteryCapacity>>8);
    
    *plen = (u8)PSU_Status_Length;

    return ERR_NONE;
}



s8 sysGetRadarInfo(u8* pdat,u16* plen)
{
    u8 i=0;
    u8 dat[RADAR_NUM]={0};
    s8 res;
    u16 avoidFlg;
    
    if((NULL==pdat)||(NULL==plen))
    {
        return ERR_POINTER_NULL;
    }
    
    res = GetTim9RadarInfo(RADAR_NUM,dat);
    
    if(res != ERR_NONE)
    {
        *plen = 0;    
        return ERR_PROCESS_FAILED;
    }
    
    pdat[0] = (u8)PC_CMD_FRAME_HEADER;
    pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
    pdat[2] = (u8)TOF_Status;
    pdat[3] = (u8)(TOF_Status>>8);
    pdat[4] = (u8)SysTimeStamp;
    pdat[5] = (u8)(SysTimeStamp>>8);
    pdat[6] = (u8)(SysTimeStamp>>16);
    pdat[7] = (u8)(SysTimeStamp>>24);
    pdat[8] = (u8)TOF_Status_Length ;
    pdat[9] = (u8)(TOF_Status_Length>>8);    
    
    for(i=0;i<RADAR_NUM;i++)
    {
        pdat[10+(i<<1)] = dat[i];
        pdat[11+(i<<1)] = 0;
    }

    avoidFlg = EwayRadarObstAvoid.ObstAvoid.AvoidEnableFlag;

    if(EwayRadarObstAvoid.ObstAvoid.Swi != 0)
    {
        avoidFlg |= 0x1000;
    }

    pdat[10+(RADAR_NUM<<1)] = (u8)avoidFlg;
    pdat[11+(RADAR_NUM<<1)] = (u8)(avoidFlg>>8);      

    *plen = (u8)TOF_Status_Length;

    return ERR_NONE;
}



s8 sysGetGripperInfo(u8* pdat,u16* plen)
{
    if((NULL==pdat)||(NULL==plen))
    {
        return ERR_POINTER_NULL;
    }
        
    pdat[0] = (u8)PC_CMD_FRAME_HEADER;
    pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
    pdat[2] = (u8)Manipulation_Status;
    pdat[3] = (u8)(Manipulation_Status>>8);
    pdat[4] = (u8)SysTimeStamp;
    pdat[5] = (u8)(SysTimeStamp>>8);
    pdat[6] = (u8)(SysTimeStamp>>16);
    pdat[7] = (u8)(SysTimeStamp>>24);
    pdat[8] = (u8)(0xFF&Manipulation_Status_Length);
    pdat[9] = (u8)(0xFF&(Manipulation_Status_Length>>8));

    ReportGripperStatusToPC(pdat,plen);

    *plen = (u8)Manipulation_Status_Length;

    return ERR_NONE;
}


s8 sysGetMotorInfo(u8* pdat,u16* plen)
{
    u8 unj;
    u16 len=0;
    //s16 sSpd;
    u8* pd = (pdat+10);
    
    if((NULL==pdat)||(NULL==plen))
    {
        return ERR_POINTER_NULL;
    }
    
    pdat[0] = (u8)PC_CMD_FRAME_HEADER;
    pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
    pdat[2] = (u8)Motor_Status;
    pdat[3] = (u8)(Motor_Status>>8);
    pdat[4] = (u8)SysTimeStamp;
    pdat[5] = (u8)(SysTimeStamp>>8);
    pdat[6] = (u8)(SysTimeStamp>>16);
    pdat[7] = (u8)(SysTimeStamp>>24);

    /*
    1.���  1-6�ŵ��
    2.�ұ�  7-12�ŵ��
    3.�粿������� 13��
    4.ͷ�� yaw/pitch ������� 14-15�ŵ��
    */
    
    for(unj=0;unj<(Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head);unj++)
    {
        pd[0] = unj+1;              //!< Arm 1-6,7-12 Shoulder:13,Head:14-15     ling-20170427 modified
        pd[1] = 0;

        if((Motor_Num_Arm+Motor_Num_Shoulder-1)!=unj)
        {
            //sSpd = (sysEwServo[unj].uSpdRPML + (sysEwServo[unj].uSpdRPMH<<8))*MOTOR_SPEED_COEFFICIENT;
            
            pd[2] =  sysEwServo[unj].uPosJointL; 
			pd[3] =  sysEwServo[unj].uPosJointH;	
            pd[4] =  sysEwServo[unj].uSpdRPML;       //!< �ϱ�����Reg0x83,0x84��ֵ���Ѿ�������100����RPM��ֵ��
		    pd[5] =  sysEwServo[unj].uSpdRPMH;
		    pd[6] =  sysEwServo[unj].uStaL;     //������ص�״̬��2���ֽ�
		    pd[7] =  sysEwServo[unj].uStaH;
            
            if((EwayEmbSys.Comm.wMotors[unj]&EMB_COMM_ERR_BITS_3_TIMES)!=0)         //!< ling-20180427 modify
            {
                pd[8] = 0;          // status1
                pd[9] = 0;          // status2
            }
            else
            {
                pd[8] |= 0x01;      // status1
                pd[9] = 0;          // status2
            }        
        
		    pd[10] = sysEwServo[unj].uinVolL; 
		    pd[11] = sysEwServo[unj].uinVolH;
		    pd[12] = sysEwServo[unj].uinCurtL; 
		    pd[13] = sysEwServo[unj].uinCurtH;
		    pd[14] = sysEwServo[unj].udevTmpt;

            pd += EWAYSERVO_PAYLOAD_BYTES;

            len += EWAYSERVO_PAYLOAD_BYTES;            
        }        
	}

    len += 10;

    pdat[8] = (u8)len;
    pdat[9] = (u8)(len>>8);

    *plen = len;

    return ERR_NONE;
}

s8 sysGetShoulderMotorInfo(u8* pdat,u16* plen)
{        
    s32 sSpd;
    
    if((NULL==pdat)||(NULL==plen))
    {
        return ERR_POINTER_NULL;
    }

    sSpd = (s32)(((s16)(sysEwServo[Emb_StartID_Shoulder-1].uSpdRPML+(sysEwServo[Emb_StartID_Shoulder-1].uSpdRPMH<<8)))*MOTOR_SPEED_COEFFICIENT);
        
    pdat[0] = (u8)PC_CMD_FRAME_HEADER;
    pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
    pdat[2] = (u8)Shoulder_Status;
    pdat[3] = (u8)(Shoulder_Status>>8);
    pdat[4] = (u8)SysTimeStamp;
    pdat[5] = (u8)(SysTimeStamp>>8);
    pdat[6] = (u8)(SysTimeStamp>>16);
    pdat[7] = (u8)(SysTimeStamp>>24);
    pdat[8] = (u8)Shoulder_Status_Lenght;
    pdat[9] = (u8)(Shoulder_Status_Lenght>>8);

    pdat[10] = Emb_StartID_Shoulder;
    pdat[11] = 0;
    pdat[12] = (u8)(sysEwServo[Emb_StartID_Shoulder-1].uPosWheel); 
    pdat[13] = (u8)(sysEwServo[Emb_StartID_Shoulder-1].uPosWheel>>8); 
    pdat[14] = (u8)(sysEwServo[Emb_StartID_Shoulder-1].uPosWheel>>16); 
    pdat[15] = (u8)(sysEwServo[Emb_StartID_Shoulder-1].uPosWheel>>24);
                
    pdat[16] = (u8)sSpd; 
    pdat[17] = (u8)(sSpd>>8);
    pdat[18] = (u8)(sSpd>>16);
    pdat[19] = (u8)(sSpd>>24);
    
    pdat[20] =  sysEwServo[Emb_StartID_Shoulder-1].uStaL;   //������ص�״̬��2���ֽ�
    pdat[21] =  sysEwServo[Emb_StartID_Shoulder-1].uStaH;
                
    if((EwayEmbSys.Comm.wMotors[Emb_StartID_Shoulder-1]&EMB_COMM_ERR_BITS_3_TIMES)!=0)         //!< ling-20180427 modify
    {
        pdat[22] = 0;                                     // status1
    }
    else
    {
        pdat[22] |= Shoulder_Status_CommErr_MaskBit;      // status1
    }        

    if((EwayShoulder.shLimit.pinState[1]&SHOULDER_LIMIT_TRIGGER_BIT)==0)
    {
        pdat[23] |= UpLimitPinTouched_MaskBit;
    }

    if((EwayShoulder.shLimit.pinState[0]&SHOULDER_LIMIT_TRIGGER_BIT)==0)
    {
        pdat[23] |= DnLimitPinTouched_MaskBit;
    }
                        
    pdat[24] = sysEwServo[Emb_StartID_Shoulder-1].uinVolL; 
    pdat[25] = sysEwServo[Emb_StartID_Shoulder-1].uinVolH;
    pdat[26] = sysEwServo[Emb_StartID_Shoulder-1].uinCurtL; 
    pdat[27] = sysEwServo[Emb_StartID_Shoulder-1].uinCurtH;
    pdat[28] = sysEwServo[Emb_StartID_Shoulder-1].udevTmpt;
    
    *plen = (u8)Shoulder_Status_Lenght;

    return ERR_NONE;
}

s8 sysGetWheelInfo(u8* pdat,u16* plen)
{
    if((NULL == pdat)||(NULL == plen))
    {
        return ERR_POINTER_NULL;
    }
    
    pdat[0] = (u8)PC_CMD_FRAME_HEADER;
    pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
    pdat[2] = (u8)Wheel_Status;
    pdat[3] = (u8)(Wheel_Status>>8);
    pdat[4] = (u8)SysTimeStamp;
    pdat[5] = (u8)(SysTimeStamp>>8);
    pdat[6] = (u8)(SysTimeStamp>>16);
    pdat[7] = (u8)(SysTimeStamp>>24);
    pdat[8] = (u8)Wheel_Status_Length ;
    pdat[9] = (u8)(Wheel_Status_Length>>8);

    ReportWheelsStatusToPC(pdat,plen);

    *plen = (u8)Wheel_Status_Length;

    return ERR_NONE;
}



s8 sysGetSingleCycleJointMotorsStatus(u8* pdat,u16* plen)
{
    u8 i;
    u8* pd0;
    u8* pd1;
    u32 tSmp;
#if (EMB_Upload_SingleCycle_Motor_Info_Print_ArmLeft||EMB_Upload_SingleCycle_Motor_Info_Print_ArmRight\
    ||EMB_Upload_SingleCycle_Motor_Info_Print_Head)
    
    char str0[100]={0};
    s16 pos0,pos1;
    s16 spd0,spd1;

#endif

    
    if((NULL == pdat)||(NULL == plen))
    {
        return ERR_POINTER_NULL;
    }

    if((EwayArms.mCommStatus == 0)&&(EwayHeads.mCommStatus == 0))     //!< ����Ƿ��е������
    {
        *plen = 0;
        
        return ERR_NONE;
    }

    pd0 = pdat+10;    
    *plen = 10;

    for(i=0;i<Motor_Num_Arm;i++)               //!< �����ֱ۵�
    {
        if((EwayArms.mCommStatus&(0x0001<<i)) != 0)           //!< ���i����
        {
            pd1 = &((EwayArms.mState.pMotorRecord+i)->wRegs[0]);    //!< ָ���ŵ������ֱ۵��ָ��״̬�����ָ��    //!< tStamp(4) + Id(2) + DnPos(2/4) + DnSpd(2) + UpPos(2/4) + UpSpd(2)
            
            tSmp = (*pd1)+((*(pd1+1))<<8)+((*(pd1+2))<<16)+((*(pd1+3))<<24);
            
            if(tSmp!=SysTimeStamp)                          //!< ���ʱ���
            {
                continue;
            }

            memcpy(pd0,(pd1+4),SingleCyDataUnit_ArmHead);
            
            pd0 += SingleCyDataUnit_ArmHead;
            *plen = (*plen) + SingleCyDataUnit_ArmHead;
#if (EMB_Upload_SingleCycle_Motor_Info_Print_ArmLeft||EMB_Upload_SingleCycle_Motor_Info_Print_ArmRight\
                ||EMB_Upload_SingleCycle_Motor_Info_Print_Head)

#if ((EMB_Upload_SingleCycle_Motor_Info_Print_ArmLeft)&&(EMB_Upload_SingleCycle_Motor_Info_Print_ArmRight==0))
            if(i<Motor_Num_ArmLeft)
#elif ((EMB_Upload_SingleCycle_Motor_Info_Print_ArmRight)&&(EMB_Upload_SingleCycle_Motor_Info_Print_ArmLeft==0))
            if((i>=Motor_Num_ArmLeft)&&(i<Motor_Num_Arm))
#elif ((EMB_Upload_SingleCycle_Motor_Info_Print_ArmLeft!=0)&&(EMB_Upload_SingleCycle_Motor_Info_Print_ArmRight!=0))
            
#endif
            {
                pos0 = pd1[6]+(pd1[7]<<8); spd0 = pd1[8]+(pd1[9]<<8);
                pos1 = pd1[10]+(pd1[11]<<8);spd1 = pd1[12]+(pd1[13]<<8);
            
                sprintf(&str0[0],"JintCmd,id:%d,DnP:%x,DnS:%x,UpP:%x,UpS:%x.",pd1[4],pos0,spd0,pos1,spd1);            
                Bsp_printf(str0);
            }
#endif
            
        }
        else
        {
            //!< ����������ߣ����ϱ�
        }        
    }
    
    for(i=0;i<Motor_Num_Head;i++)               //!< ����ͷ����
    {
        if((EwayHeads.mCommStatus&(0x0001<<i)) != 0)           //!< ���i����
        {
            pd1 = &((EwayHeads.mState.pMotorRecord+i)->wRegs[0]);    //!< ָ���ŵ������ֱ۵��ָ��״̬�����ָ��    //!< tStamp(4) + Id(2) + DnPos(2/4) + DnSpd(2) + UpPos(2/4) + UpSpd(2)
                
            tSmp = (*pd1)+((*(pd1+1))<<8)+((*(pd1+2))<<16)+((*(pd1+3))<<24);
                
            if(tSmp!=SysTimeStamp)                          //!< ���ʱ���
            {
                continue;
            }
    
            memcpy(pd0,(pd1+4),SingleCyDataUnit_ArmHead);
                
            pd0 += SingleCyDataUnit_ArmHead;
            *plen = (*plen) + SingleCyDataUnit_ArmHead;
            
#if (EMB_Upload_SingleCycle_Motor_Info_Print_Head)
            pos0 = pd1[6]+(pd1[7]<<8); spd0 = pd1[8]+(pd1[9]<<8);
            pos1 = pd1[10]+(pd1[11]<<8);spd1 = pd1[12]+(pd1[13]<<8);
                        
            sprintf(&str0[0],"JintCmd,id:%d,DnP:%x,DnS:%x,UpP:%x,UpS:%x.",pd1[4],pos0,spd0,pos1,spd1);            
            Bsp_printf(str0);
#endif            
        }
        else
        {
           //!< ����������ߣ����ϱ�
        }        
    }    

    if(*plen==10)
    {
        memset(pdat,0,10);

        *plen = 0;
    }
    else
    {
        pdat[0] = (u8)PC_CMD_FRAME_HEADER;
        pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
        pdat[2] = (u8)SingleCycle_JointMotors_Status;
        pdat[3] = (u8)(SingleCycle_JointMotors_Status>>8);    
        pdat[4] = (u8)SysTimeStamp;
        pdat[5] = (u8)(SysTimeStamp>>8);
        pdat[6] = (u8)(SysTimeStamp>>16);
        pdat[7] = (u8)(SysTimeStamp>>24);
        pdat[8] = (u8)(*plen) ;
        pdat[9] = (u8)((*plen)>>8);
    }

    return ERR_NONE;
}




s8 sysGetEmbSystemSoftwareVersion(u8* pdat,u16* plen)
{
    if((NULL == pdat)||(NULL == plen))
    {
        return ERR_POINTER_NULL;
    }

    pdat[0] = (u8)PC_CMD_FRAME_HEADER;
    pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
    pdat[2] = (u8)EmbSystemSoftwareVersion_Info;
    pdat[3] = (u8)(EmbSystemSoftwareVersion_Info>>8);    
    pdat[4] = (u8)SysTimeStamp;
    pdat[5] = (u8)(SysTimeStamp>>8);
    pdat[6] = (u8)(SysTimeStamp>>16);
    pdat[7] = (u8)(SysTimeStamp>>24);
    pdat[8] = (u8)EmbSysSoftVer_Length ;
    pdat[9] = (u8)((EmbSysSoftVer_Length)>>8);

    memcpy((pdat+10),&EwayEmbSysInfo.SoftVersion[0],7);
    
    *plen = EmbSysSoftVer_Length;
    
    return ERR_NONE;
}

s8 sysGetSingleCycleWheelMotorsStatus(u8* pdat,u16* plen)
{
    u8 i;
    u8* pd0;
    u8* pd1;
    u32 tSmp;
#if (EMB_Upload_SingleCycle_Motor_Info_Print_Shoulder||EMB_Upload_SingleCycle_Motor_Info_Print_Wheel)    
    char str0[100]={0};
    s32 pos0,pos1;
    s16 spd0,spd1;
#endif

    if((NULL == pdat)||(NULL == plen))
    {
        return ERR_POINTER_NULL;
    }

    if((EwayShoulder.mCommStatus == 0)&&(EwayWheels.mCommStatus == 0))     //!< ����Ƿ��е������
    {
        return ERR_NONE;
    }

    pd0 = pdat+10;    
    *plen = 10;

    for(i=0;i<Motor_Num_Shoulder;i++)               //!< ���¼���
    {
        if((EwayShoulder.mCommStatus&(0x0001<<i)) != 0)           //!< ���i����
        {
            pd1 = &((EwayShoulder.mState.pMotorRecord+i)->wRegs[0]);    //!< ָ���ŵ������ֱ۵��ָ��״̬�����ָ��    //!< tStamp(4) + Id(2) + DnPos(2/4) + DnSpd(2) + UpPos(2/4) + UpSpd(2)
            
            tSmp = (*pd1)+((*(pd1+1))<<8)+((*(pd1+2))<<16)+((*(pd1+3))<<24);
            
            if(tSmp!=SysTimeStamp)                          //!< ���ʱ���
            {
                continue;
            }

            memcpy(pd0,(pd1+4),SingleCyDataUnit_ShlWhel);
            
            pd0 += SingleCyDataUnit_ShlWhel;
            *plen = (*plen) + SingleCyDataUnit_ShlWhel;
            
#if (EMB_Upload_SingleCycle_Motor_Info_Print_Shoulder)
            pos0 = pd1[6]+(pd1[7]<<8)+(pd1[8]<<16)+(pd1[9]<<24); spd0 = pd1[10]+(pd1[11]<<8);
            pos1 = pd1[12]+(pd1[13]<<8)+(pd1[14]<<16)+(pd1[15]<<24); spd1 = pd1[16]+(pd1[17]<<8);
                        
            sprintf(&str0[0],"WhelCmd,id:%d,DnP:%x,DnS:%x,UpP:%x,UpS:%x.",pd1[4],pos0,spd0,pos1,spd1);            
            Bsp_printf(str0);
#endif
            
        }
        else
        {
            //!< ����������ߣ����ϱ�
        }        
    }
    
    for(i=0;i<Motor_Num_Wheel;i++)               //!< �������ӵ����
    {
        if((EwayWheels.mCommStatus&(0x0001<<i)) != 0)           //!< ���i����
        {
            pd1 = &((EwayWheels.mState.pMotorRecord+i)->wRegs[0]);    //!< ָ���ŵ������ֱ۵��ָ��״̬�����ָ��    //!< tStamp(4) + Id(2) + DnPos(2/4) + DnSpd(2) + UpPos(2/4) + UpSpd(2)
                
            tSmp = (*pd1)+((*(pd1+1))<<8)+((*(pd1+2))<<16)+((*(pd1+3))<<24);
                
            if(tSmp!=SysTimeStamp)                          //!< ���ʱ���
            {
                continue;
            }
    
            memcpy(pd0,(pd1+4),SingleCyDataUnit_ShlWhel);
                
            pd0 += SingleCyDataUnit_ShlWhel;
            *plen = (*plen) + SingleCyDataUnit_ShlWhel;
            
#if (EMB_Upload_SingleCycle_Motor_Info_Print_Wheel)
            pos0 = pd1[7]+(pd1[8]<<8)+(pd1[9]<<16)+(pd1[10]<<24); spd0 = pd1[11]+(pd1[12]<<8);
            pos1 = pd1[13]+(pd1[14]<<8)+(pd1[15]<<16)+(pd1[16]<<24); spd1 = pd1[17]+(pd1[18]<<8);
                        
            sprintf(&str0[0],"WhelCmd,id:%d,Tp:%d(0-P,1-S),DnP:0x%x,DnS:0x%x,UpP:0x%x,UpS:0x%x.",pd1[4],pd1[6],pos0,spd0,pos1,spd1);            
            Bsp_printf(str0);
#endif
        }
        else
        {
           //!< ����������ߣ����ϱ�
        }        
    }    

    if(*plen==10)
    {
        memset(pdat,0,10);

        *plen = 0;
    }
    else
    {
        pdat[0] = (u8)PC_CMD_FRAME_HEADER;
        pdat[1] = (u8)(PC_CMD_FRAME_HEADER>>8);    
        pdat[2] = (u8)SingleCycle_WheelMotors_Status;
        pdat[3] = (u8)(SingleCycle_WheelMotors_Status>>8);    
        pdat[4] = (u8)SysTimeStamp;
        pdat[5] = (u8)(SysTimeStamp>>8);
        pdat[6] = (u8)(SysTimeStamp>>16);
        pdat[7] = (u8)(SysTimeStamp>>24);
        pdat[8] = (u8)(*plen) ;
        pdat[9] = (u8)((*plen)>>8);
    }

    return ERR_NONE;

}


void sysPcCommMaintainWork(void)
{
    //s8 res;
    //!< �������λ��ͨ��״̬����ִ����Ӧ����
    if((EwayEmbSys.Comm.wPc&0x03)==0x02)    //!<��鵱ǰ��PC������ͨ���ǸոնϿ���      11111110 �����
    {
        //SetPSMCmdTcpErr();                  //!< ��PSM�·�tcp����ʱ��ִ�еĲ���:
        
        LedsStatus_Set(LED1,LIGHT_OFF);     //!< LED1Ϩ��

        //!< ����ֱۡ����ͷ����צ�ӡ����ӵ�ָ���
        sysClrPcToEmbPartsCmdBuff();

        //!< ���¸��������lastPosCmd
        sysUpdateMotorsLastPosCmd();

        //!< ����Ͽ���������������ֹͣ��״̬
        sysPutWheelMotorsIntoFreeStopMode();
    }

    if((EwayEmbSys.Comm.wPc&0x03)==0x01)    //!< ��鵽��ǰ��PC������ͨ���Ǹո����ӵ�
    {
        //SetPSMCmdTcpReConnect();
        //!< ����ָ�����Ҫ�����ӽ������ֹͣ��״̬
        sysPutWheelMotorsIntoStopOffMode();
        
    }

}


/*                �汾��    ָ��    ʱ���    ���ݳ���    �ػ�ָ��    �ӳ�ʱ�䣨S��
�ػ�����    FE E1    FE 01    00 00 00 00    0C 00    00����01    00
                                                            1����ϵ磬0�����ϵ磻  
                                                            Bit0������ۣ�       
                                                            Bit1������̣�     
                                                            Bit2�����ұۣ�          
                                                            Bit4����DCDC��     
                                                            Bit7����������ߣ�               
                                                            Bit8�����Դ�ܿ��أ�   
                                                            ע������ͣʱ�ֱ۵��̲��ɿأ�    ��λ�루ֻ���ܿ������ã�                  
*/

s8 PC_PowerOffCMD(u8 *pUData,u16 unLen)    
{
    u8 unSwitchCmd=pUData[0];
    u8 unDelayTime=pUData[1];

    if(0x80==unSwitchCmd)  //�ĵ��ʹ��벻���ϣ�����ȷ�����ٸĳɺ궨��
    {
        if(0==unDelayTime)
        {
            SetPSMCmdPowerOff();
        }
        else
        {
            SetPSMCmdPowerOffDelay(unDelayTime);
        }
    }
    else
    {
        SetPSMCmdSwitchCtrl(unSwitchCmd);
    }

    return ERR_NONE;
}




/*                    �汾��    ָ��    ʱ���           ���ݳ���        LED    
  LED����    FE E1    50 01    00 00 00 00    0B 00        00    ע����ֻ�л�������״̬�¿��ã���ͣʱ�����ã�
                                                    0����ȫ��    
                                                    1����ȫ����˸      
                                                    2������ɫ������  
                                                    3����ȫ��������     
                                                    4����ȫ�̺�����     
                                                    5����ȫ�ƺ���    
*/

s8 PC_CmdLedCtrl(u8 *pUData,u16 unLen)
{
    s8 sRet=0;
    u8 uLedCmd = pUData[0];

    sRet=SetPSMCmdLedCtrl((LedCtrlType)uLedCmd);  //�ĵ��ʹ������󣬴˴�������Ҫת��.

    return sRet;
}

s8 PC_CmdLaserCtrl(u8 *pUData,u16 unLen)
{
    u8 laser = pUData[0];

    if((pUData==NULL)||(unLen!=1))
    {
        return ERR_INPUT_PARAMETERS;
    }

    LaserStatus_Set(laser);

    return ERR_NONE;
}


s8 PC_CmdHeartBeat(u8 *pUData,u16 unLen)
{
    if((pUData==NULL)||(unLen!=1))
    {
        return ERR_INPUT_PARAMETERS;
    }

    EwayEmbTCP.runTimeout = 0;              //!< �峬ʱ������

    return ERR_NONE;
}




//ʹ������Ϣ���к���������ʵ���������Ϣ���С�
/*                   �汾��    ָ��    ʱ���                    ���ݳ���    ID
                    
�������    FE E1    00 01    00 00 00 00           0C 00            00 00
*/
s8 PC_CmdClear(u8 *pUData,u16 unLen)
{
    u8 uClearID = pUData[0];  //�ĵ����벻������8bit����16bit.
    u8 posKp = pUData[1];
    BaseType_t re;
    u8 i;
    UBaseType_t tmp;
    s8 res;

    if(unLen!=2)
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(uClearID&PCCmd_CLEAR_ARM_MASK_BIT)              //!< Arms
    {
        re = xQueueReset(PCToArmsQueHdl);
        if(re != pdPASS)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                Bsp_printf("Clr Queue PCToArmsQueHdl failed.");
            }
        }   
        
        //!< ����������ָ���ִ�л���
        for(i=0;i<Motor_Num_Arm;i++)
        {
            res = g_ClearPcCmdtoNodeCmdBuff(EwayArms.mControl.pToMotorCmd,i);
            if(res!=ERR_NONE)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    Bsp_printf("Clr Arms(%d) ToMotorCmd Cmd Buf failed,rt:%d.",i,res);
                }
            }
        } 
        
        if(posKp&PCCmd_CLEAR_ARM_MASK_BIT)     //!< �Ƿ�λ�ñ���
        {
            EwayArms.mControl.posKeep = 0x1100;
            EwayArms.mControl.posKeep |= 'K';
        }
        else
        {
            EwayArms.mControl.posKeep = 0x1100;
            EwayArms.mControl.posKeep |= 'N';            
        }

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
        {
            tmp = uxQueueMessagesWaiting(PCToArmsQueHdl);
            Bsp_printf("recv sh ClearCmd,keep:%c,qCmd:%d,CmdBuff:%d.",EwayArms.mControl.posKeep,tmp,EwayArms.mControl.pToMotorCmd->dCmdCnt);
        }
    }

    if(uClearID&PCCmd_CLEAR_SHOULDER_MASK_BIT)          //!< Shoulder
    {
        re = xQueueReset(PCToShoulderQueHdl);
        if(re != pdPASS)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                Bsp_printf("Clr Queue PCCmdBuffOpHandleShd failed.");
            }
        }

        //!< ����������ָ���ִ�л���
        res = g_ClearPcCmdtoNodeCmdBuff(EwayShoulder.mControl.pToMotorCmd,0);
        if(res!=ERR_NONE)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                Bsp_printf("Clr Shoulder ToMotorCmd Cmd Buf failed,rt:%d.",res);
            }
        }

        if(posKp&PCCmd_CLEAR_SHOULDER_MASK_BIT)     //!< �Ƿ�λ�ñ���
        {
            EwayShoulder.mControl.posKeep = 0x0100;
            EwayShoulder.mControl.posKeep |= 'K';
        }
        else
        {
            EwayShoulder.mControl.posKeep = 0x0100;
            EwayShoulder.mControl.posKeep |= 'N';
        }

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
        {
            tmp = uxQueueMessagesWaiting(PCToShoulderQueHdl);
            Bsp_printf("recv sh ClearCmd,keep:%c,qCmd:%d,CmdBuff:%d.",EwayShoulder.mControl.posKeep,tmp,EwayShoulder.mControl.pToMotorCmd->dCmdCnt);
        }
        
    }

    if(uClearID&PCCmd_CLEAR_HEAD_MASK_BIT)          //!< Head
    {
        re = xQueueReset(PCToHeadsQueHdl);
        if(re != pdPASS)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                Bsp_printf("Clr Queue PCToHeadsQueHdl failed.");
            }
        }        

        //!< ����������ָ���ִ�л���
        for(i=0;i<Motor_Num_Head;i++)
        {
            res = g_ClearPcCmdtoNodeCmdBuff(EwayHeads.mControl.pToMotorCmd,i);
            if(res!=ERR_NONE)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    Bsp_printf("Clr Head(%d) ToMotorCmd Cmd Buf failed,rt:%d.",i,res);
                }
            }
        }        

        if(posKp&PCCmd_CLEAR_HEAD_MASK_BIT)     //!< �Ƿ�λ�ñ���
        {
            EwayHeads.mControl.posKeep = 0x0100;
            EwayHeads.mControl.posKeep |= 'K';
        }
        else
        {
            EwayHeads.mControl.posKeep = 0x0100;
            EwayHeads.mControl.posKeep |= 'N';
        }

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
        {
            tmp = uxQueueMessagesWaiting(PCToHeadsQueHdl);
            Bsp_printf("recv hd ClearCmd,keep:%c,qCmd:%d,CmdBuff:%d,%d.",EwayHeads.mControl.posKeep,tmp,EwayHeads.mControl.pToMotorCmd->dCmdCnt,(EwayHeads.mControl.pToMotorCmd+1)->dCmdCnt);
        }
    }

    if(uClearID&PCCmd_CLEAR_WHEEL_MASK_BIT)
    {    
        re = xQueueReset(PCToWheelsQueHdl);
        if(re != pdPASS)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                Bsp_printf("Clr Queue PCToWheelsQueHdl failed.");
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
                    Bsp_printf("Clr Wheel(%d) ToMotorCmd Cmd Buf failed,rt:%d.",i,res);
                }
            }
        }  

        if(posKp&PCCmd_CLEAR_WHEEL_MASK_BIT)     //!< �Ƿ�λ�ñ���
        {
            EwayWheels.mControl.posKeep = 0x0100;
            EwayWheels.mControl.posKeep |= 'K';
        }
        else
        {
            EwayWheels.mControl.posKeep = 0x0100;
            EwayWheels.mControl.posKeep |= 'N';        
        }

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
        {
            tmp = uxQueueMessagesWaiting(PCToWheelsQueHdl);
            Bsp_printf("recv Wheels ClearCmd,keep:%c,qCmd:%d,CmdBuff:%d,%d.",EwayWheels.mControl.posKeep,tmp,EwayWheels.mControl.pToMotorCmd->dCmdCnt,(EwayWheels.mControl.pToMotorCmd+1)->dCmdCnt);
        }
    }

    if(uClearID&PCCmd_CLEAR_GRIPPER_MASK_BIT)
    {    
        re = xQueueReset(PCToGrippersQueHdl);
        if(re != pdPASS)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                Bsp_printf("Clr Queue PCToGrippersQueHdl failed.");
            }
        }

        //!< ����������ָ���ִ�л���
        for(i=0;i<Motor_Num_Gripper;i++)
        {
            res = g_ClearPcCmdtoNodeCmdBuff(EwayGrippers.mControl.pToMotorCmd,i);
            if(res!=ERR_NONE)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    Bsp_printf("Clr Gripper(%d) ToMotorCmd Cmd Buf failed,rt:%d.",i,res);
                }
            }
        }  

        if(posKp&PCCmd_CLEAR_GRIPPER_MASK_BIT)     //!< �Ƿ�λ�ñ���
        {
            EwayGrippers.mControl.posKeep = 0x0100;
            EwayGrippers.mControl.posKeep |= 'K';
        }
        else
        {
            EwayGrippers.mControl.posKeep = 0x0100;
            EwayGrippers.mControl.posKeep |= 'N';           
        }
    }

    return ERR_NONE;
}

s8 PC_CmdDebugCtrl(u8 *pUData,u16 unLen)
{
    if((pUData==NULL)||(unLen!=PCCmd_DebugCtrl_Unit))
    {
        return ERR_INPUT_PARAMETERS;
    }

    pDebug->sysDebugCtrl = (pUData[0] + (pUData[1]<<8))|(SysDebugCtrl_EMB_SYS_MASK|SysDebugCtrl_EMB_Sys_Err_MASK);

    pDebug->jointSw = pUData[2] + (pUData[3]<<8);

    pDebug->secondFun = pUData[4] + (pUData[5]<<8) + (pUData[6]<<16) + (pUData[7]<<24);    

    if(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)
    {
       Bsp_printf("sysDebugCtrl:0x%x,jointSw:0x%x,secFun:0x%x.",pDebug->sysDebugCtrl,pDebug->jointSw,pDebug->secondFun);
    }    

    return ERR_NONE;
}


u8 g_unAvoidSwitch;
u8 g_unAvoidDis;
u8 g_unAvoidBrakeFlag;

/*              �汾��    ָ��    ʱ���       ���ݳ���    ����    ����    
���Ϲ�������    FE E1    01 01    00 00 00 00    0C 00      01      00    ���Ͽ�
                                                                                            00            00    ���Ϲ�
*/
/*
s8 PC_CmdObstSwitch(u8 *puData,u16 unLen)
{
    u8 uSwitch=puData[0];
    //u8 uDistance=puData[1];
    
        if(0x0==uSwitch)//�ر�
        {
            g_unAvoidSwitch=0;
            g_unAvoidDis=puData[1];   //��ֵ��ʵ��ʱû��
        }
        else if(0x1==uSwitch) //��
        {
            g_unAvoidSwitch=1;
            g_unAvoidDis=puData[1];   //��ֵ��ʵ��ʱû��
        }
        else
        {
            //chengyilog
            return ERR_DATA_OVERFLOW;
        }
    
        return ERR_NONE;
}
*/
s8_t PC_CmdObstSwitch(u8 *puData,u16 unLen)
{
    if((puData==NULL)||(unLen!=2))
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(puData[0]!=0)
    {
        EwayRadarObstAvoid.ObstAvoid.Swi = 1;

        if(puData[1]>=ObstDistance_Brake)
        {
            EwayRadarObstAvoid.ObstAvoid.distance = puData[1];
        }
        else
        {
            EwayRadarObstAvoid.ObstAvoid.distance = ObstDistance_Brake;
        }        
    }
    else
    {
        EwayRadarObstAvoid.ObstAvoid.Swi = 0;
        
        EwayRadarObstAvoid.ObstAvoid.distance = puData[1];
    }

    return ERR_NONE;
}




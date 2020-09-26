/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Mic.c
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2018-05-28
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/05/28 | 0.0.1 |          | Create file
*
*/

#include "includes.h"


extern EwayEmbSysDebugModule* pDebug;
extern EwayEmbSysModule EwayEmbSys;



EwayMicModule EwayMic={
                        {0x0000,{0x00},0x00,0x00},
                        0x0000,                        
                        EwayMicRunState_UnInitial,
                        0x0000,
                        0x0000,
                        0x0000
};




u8 l_GetMicEnhanceSoundBeam(u16 angle)
{

    if((angle<30)||(angle>=330))
    {
        return 0;
    }
    else if((angle>=30)&&(angle<90))
    {
        return 1;
    }
    else if((angle>=90)&&(angle<150))
    {
        return 2;
    }
    else if((angle>=150)&&(angle<210))
    {
        return 3;
    }
    else if((angle>=210)&&(angle<270))
    {
        return 4;
    }
    else if((angle>=270)&&(angle<330))
    {
        return 5;
    }
    else
        return 0;
}



void sysEmbMicMaintainWork(EwayMicModule* pModule)
{
    u8 b1=0,b2=0;
    
    if((EwayEmbSys.sysTimCnt%EMB_READ_MIC_INTERVAL)==20)
    {
        switch(EwayMic.runState)
        {
            case EwayMicRunState_UnInitial:
                 //!< Send PING packet
                if((EwayMic.mCommStatus & 0x01)!=0)
                {
                    if(EwayMic.MicInfo.mic_online==MIC_ONLINE)
                    {
                        EwayMic.runState = EwayMicRunState_OnLine_SendReset;

                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                        {
                            //Bsp_printf("Mic-3-1 State 0--->1");
                        }
                    }
                    else
                    {
                        EwayMic.runState = EwayMicRunState_UnInitial;
                    }
                }
                else
                {
                   EwayMic.runState = EwayMicRunState_UnInitial;
                   
                   EwayMic.MicInfo.rtReset = 0;
                }
                    
                break;
        
            case EwayMicRunState_OnLine_SendReset:        
                //!< reset Mic board
                if((EwayMic.mCommStatus & (0x01<<EwayMicRunState_OnLine_SendReset))!=0)
                {
                    if(EwayMic.MicInfo.rtReset==0x01)               //!< �յ���ȷ�ظ���������һ�׶�
                    {
                        EwayMic.runState = EwayMicRunState_OnLine_SendWake;
                        
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                        {
                            //Bsp_printf("Mic-3-2 State 1--->2");
                        }
                    }
                    else                                            //!< �յ�ʧ�ܻظ�������δ��ʼ���׶�
                    {
                        EwayMic.runState = EwayMicRunState_UnInitial;

                        EwayMic.MicInfo.rtReset = 0x00;
                    }                   
                }
                else                                                //!< δ�յ��ظ������½���δ��ʼ���׶�
                {
                   EwayMic.runState = EwayMicRunState_UnInitial;
                   
                   EwayMic.MicInfo.rtReset = 0;
                }
                    
                break;
        
            case EwayMicRunState_OnLine_SendWake:        
                //!< Set Mic Wakeup Fun
                if((EwayMic.mCommStatus & (0x01<<EwayMicRunState_OnLine_SendWake))!=0)
                {
                    if(EwayMic.MicInfo.rtWake == 0x03)
                    {
                        EwayMic.runState = EwayMicRunState_Initialized_RdVersi;

                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                        {
                            //Bsp_printf("Mic-3-3 State 2--->3");
                        }
                    }
                    else
                    {
                        EwayMic.runState = EwayMicRunState_UnInitial;

                        EwayMic.MicInfo.rtWake = 0;
                    }
                }
                else
                {
                   EwayMic.runState = EwayMicRunState_UnInitial;

                   EwayMic.MicInfo.rtWake = 0;
                }
                    
                break;

            case EwayMicRunState_Initialized_RdVersi:
                //!< Read Mic board Version
                if((EwayMic.mCommStatus & 0x08)!=0)
                {
                    EwayMic.runState = EwayMicRunState_Initialized_RdAngle;
                   
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                    {
                        //Bsp_printf("Mic-3-4 State 3--->4");
                    }
                }
                else
                {
                   EwayMic.runState = EwayMicRunState_UnInitial;
                }
                
                break;
        
            case EwayMicRunState_Initialized_RdAngle:
                //!< Read Mic Waku up Angle
                if((EwayMic.mCommStatus & 0x10)!=0)
                {
                    EwayMic.runState = EwayMicRunState_Initialized_RdAngle;

                    EwayMic.errCnt = 0;

                    if(EwayMic.Last_wkAngle != EwayMic.MicInfo.wakeup_angle)
                    {
                        b1 = l_GetMicEnhanceSoundBeam(EwayMic.Last_wkAngle);

                        b2 = l_GetMicEnhanceSoundBeam(EwayMic.MicInfo.wakeup_angle);

                        Bsp_printf("Mic Sound Beam,b1:%d,b2:%d.",b1,b2);

                        if(b1 != b2)
                        {
                            sysEmbEnhanceMicSoundBeam(MIC_Start_ID,b2);
                        }
                        
                        EwayMic.Last_wkAngle = EwayMic.MicInfo.wakeup_angle;
                    }

                    /*
                    //!< ���Խ׶Σ�����һ��ʱ�䣬�һ��ѽǶ�ֵ��û�иı䣬��������ģ����������״̬����������λ��ȷ�������߼������޸���Ӧ����
                    if((EwayMic.MicInfo.wakeup_angle == EwayMic.Last_wkAngle)&&(EwayMic.MicInfo.wakeup_angle != 0))         //!< �����ϴλ��ѽǶ���ͬ����
                    {
                        EwayMic.comCnt++;

                        if(EwayMic.comCnt >= 6)             //!< ��ʱ���˾ͽ��������״̬
                        {
                            EwayMic.comCnt++;
                            
                            EwayMic.runState = EwayMicRunState_Run_SendReset;
                        }

                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                        {
                            Bsp_printf("Mic-3-5 State 4--->5,Angle:%d.",EwayMic.MicInfo.wakeup_angle);
                        }
                    }
                    else if()       //!< ��
                    {

                    }
                    else if()
                    {

                    }
                    */

                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                    {
                        Bsp_printf("Mic-3-5 State 4--->5,Angle:%d.",EwayMic.MicInfo.wakeup_angle);
                    }
                }
                else
                {
                   EwayMic.errCnt += 1;
                   if(EwayMic.errCnt > EMB_COMMUNICATION_ERR_MIC_INTERVAL)      //!< ��4.5sͨ��ʧ�ܣ�����Ҫ���³�������������
                   {
                        EwayMic.errCnt = 0;

                        EwayMic.runState = EwayMicRunState_UnInitial;

                        EwayMic.Last_wkAngle = 0;
                   }
                }
                
                break;

            case EwayMicRunState_Run_SendReset:
            

                break;
        
            default:
                //!< Ĭ��Ϊδ��ʼ��״̬
                EwayMic.runState = EwayMicRunState_UnInitial;
                
                break;
        }
    }
        
}



s8 sysEmbResetMicBoard(u8 sAddr)
{
    u8 dat[2]={0x11,0x00};
    
    Mic_Write(sAddr,SET_RESET,1,&dat[1]);//��λmic��

    return ERR_NONE;
}




s8 sysEmbEnableMicWakeup(u8 sAddr)
{
    u8 dat[2]={SET_WAKE_MODE,0x01};
    
    Mic_Write(sAddr,SET_WAKE_MODE,1,&dat[1]);   //��λmic��

    return ERR_NONE;
}

s8 sysEmbEnhanceMicSoundBeam(u8 sAddr,u8 beam)
{
    u8 dat[2]={SET_RECE_BS_ID,0x01};        //!< ��ǿʰ������

    dat[1] = beam;
    
    Mic_Write(sAddr,SET_RECE_BS_ID,1,&dat[1]);
    
    return ERR_NONE;
}


s8 sysEmbReadMicInfo(u8 sAddr)
{    
    u8 dat[2]={GET_VERSION,0x01};           //mic�汾�Ĵ�����ַ
    
    Mic_Read(sAddr,dat,2);

    return ERR_NONE;
}


s8 sysEmbReadMicWakeupAngle(u8 sAddr)
{
    u8 dat[2]={GET_WAKE_ANGLE,0x01};        //mic������Դ�Ĵ�����ַ
    
    Mic_Read(sAddr,dat,2);
    
    return ERR_NONE;
}




s8 g_DataRecvdFromMicprocess(u8* pDat,u16 len)
{
    u8 fCode = pDat[4];

    //!<���ID
    if((pDat[2]!=MIC_Start_ID))
    {
        return ERR_RX_DATA_ADDR;
    }
    
    switch(fCode)
    {
        case MICPing_H:
            
            if(len==MIC_RESPONSE_LENGTH_PING)
            {
                EwayMic.MicInfo.mic_online = pDat[5];

                EwayMic.mCommStatus |= (0x01<<EwayMicRunState_UnInitial);

                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Mic_2_Emb_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                {
                    //Bsp_printf("Mic-2-1 Emb Recvd MICPing_H respond pkt.sta:0x%x",pDat[5]);
                }                                
            }
            else
            {
                EwayMic.MicInfo.mic_online = MIC_NOLINE;
            }
            
            break;
                
        case MICRead_H:
                
            if((len == MIC_RESPONSE_LENGTH_WkPos)||(len == MIC_RESPONSE_LENGTH_Versi))     //!< �յ���Mic �Ļظ���
            {
                if(len == MIC_RESPONSE_LENGTH_Versi)
                {
                    memcpy(&EwayMic.MicInfo.verinfo[0],(pDat+6),8);
                    
                    EwayMic.mCommStatus |= (01<<EwayMicRunState_Initialized_RdVersi);
                    
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Mic_2_Emb_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                    {
                        //Bsp_printf("Mic-2-4 Emb Recvd MICRead_H Versi respond pkt,res:0x%x-0x%x.",(pDat[6]+(pDat[7]<<8)+(pDat[8]<<16)+(pDat[9]<<24)),(pDat[10]+(pDat[11]<<8)+(pDat[12]<<16)+(pDat[13]<<24)));
                    }
                }
                else
                {
                    if(pDat[5]==0)
                    {
                        EwayMic.MicInfo.wakeup_angle = 0;
                        
                        Bsp_printf("Mic-2-5 Emb Recvd MICRead_H WkPos respond Err pkt.");
                    }
                    else 
                    {
                        EwayMic.MicInfo.wakeup_angle = pDat[6] + (pDat[7]<<8);

                        EwayMic.mCommStatus |= (0x01<<EwayMicRunState_Initialized_RdAngle);

                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Mic_2_Emb_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                        {
                            //Bsp_printf("Mic-2-5 Emb Recvd MICRead_H WkPos respond OK pkt,Angle:0x%x.",(EwayMic.MicInfo.wakeup_angle));
                        } 
                    }
                }
             }            
                
            break;
                
        case MICWrite_H:
                if(len==MIC_RESPONSE_LENGTH_WRITE)
                {
                    if(pDat[5]==0x00)               //!< Reset Mic failed
                    {
                        EwayMic.MicInfo.rtReset = 0x00;
                        
                        EwayMic.mCommStatus |= (0x01<<EwayMicRunState_OnLine_SendReset);
                        
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Mic_2_Emb_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                        {
                            //Bsp_printf("Mic-2-2 Emb Recvd MICWrite_H Reset respond pkt,res Reset Succ!");
                        }
                    }
                    else if(pDat[5]==0x01)          //!< Reset Mic Successful
                    {
                        EwayMic.MicInfo.rtReset = 0x01;
                        
                        EwayMic.mCommStatus |= (0x01<<EwayMicRunState_OnLine_SendReset);
                        
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Mic_2_Emb_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                        {
                            //Bsp_printf("Mic-2-2 Emb Recvd MICWrite_H Reset respond pkt,res Reset failed.");
                        }
                    }
                    else if(pDat[5]==0x02)          //!< open Wakeup failed
                    {
                        EwayMic.MicInfo.rtWake= 0x02;
                        
                        EwayMic.mCommStatus |= (0x01<<EwayMicRunState_OnLine_SendWake);
                        
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Mic_2_Emb_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                        {
                            //Bsp_printf("Mic-2-3 Emb Recvd MICWrite_H SetWakeup respond pkt,res SetWakeup Succ!");
                        }
                    }
                    else if(pDat[5]==0x03)          //!< open Wakeup Successful
                    {
                        EwayMic.MicInfo.rtWake = 0x03;
                                                
                        EwayMic.mCommStatus |= (0x01<<EwayMicRunState_OnLine_SendWake);
                                                
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Mic_2_Emb_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_MIC_MASK))
                        {
                            //Bsp_printf("Mic-2-3 Emb Recvd MICWrite_H SetWakeup respond pkt,res SetWakeup failed.");
                        }
                    }                                       
                }
                
                break;
                
        default:
            break;
    }


    return ERR_NONE;
}



/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Wheel_Cmd.c
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
* 2018/01/29 | 0.0.1 | Ling     | Create file
*
*/


#include "includes.h"

extern EwayEmbSysModule EwayEmbSys;
extern EwayRadarObstAvoidModule EwayRadarObstAvoid;
extern EwayWheelModule EwayWheels;
extern EwayEmbSysDebugModule* pDebug;
extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];


/*
����PCָ�������ʱ����ǰ���ϵĴ���

(1) ��鵱ǰ�Ƿ��б��ϱ�־or���Ϲ����Ƿ��
        �ޣ��򷵻�
        �У������(2)

(2) �����ٶȼ�¼�жϵ�ǰ�����˶����:
        ��ǰ or ��� or ��ֹ or ԭ�ص���תСȦ 

(3) ���ݵ�ǰ���ϱ�־λ����ǰ�����˶���������д���

        ����ǰ��������ǰ�˶��ģ�����ǰ��4�����ϱ�ʶ����������ָֹͣ�������
        ����ǰ����ʱ����˶��ģ����к�2�����ϱ�ʶ����������ָֹͣ�������
        ����ǰ�����ǽ��ƾ�ֹ�ģ��򲻷����κ�ָ��

(4) �˳�


*/
/* --------------------------------------------------------------------------*/
/**
* @name g_WheelMovementObstacleAvoidProcess
* @brief 
* @details ��Ŀǰ��PC�·���ָ�������ʱ����ǰ���ϵĴ���
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 g_WheelMovementObstacleAvoidProcess(EwayWheelModule* pModule,EwayRadarObstAvoidModule *pAvoidModule)
{
    s8 res=0,Sum=0;
    u8 i=0,Result=0;
    u8 action=0;
    u8 tmpDat[20]={0};
    u8 slvID[2]={0};
	s16 sSpd;
	s32 sPos;
    u8* pd;
    

    if((pAvoidModule->ObstAvoid.unAvoidFlag==0)||(pAvoidModule->ObstAvoid.Swi == 0))      //!< ��ǰ���Ϲ���δ��or��ǰ�ޱ��ϱ�ʶ������������ϣ�����
    {
        return ERR_NONE;
    }

    //!< ���������˶����Ʒ���
    res = g_WheelMovementStatusAnalysis(&Result,&Sum,&(pAvoidModule->WheelMov.LeftSpd[0]),&(pAvoidModule->WheelMov.RightSpd[0]));

    if(res == ERR_NONE)
    {
        pAvoidModule->ObstAvoid.WheelMove.Result = Result;
        pAvoidModule->ObstAvoid.WheelMove.Sum = Sum;
        pAvoidModule->ObstAvoid.WheelMove.ResUpdat = 0x01;

        if(Result==WHEELS_MOVEMENTSTATUS_MOVE)
        {
            if(((Sum >= 0)&&((pAvoidModule->ObstAvoid.unAvoidFlag&0x0F) != 0))||((Sum < 0)&&((pAvoidModule->ObstAvoid.unAvoidFlag&0x30) != 0)))           //!< ǰ�������ǰ��4���״�
            {                
                action = 1;             //!< action stop
                
                //!< ���±���ʹ�ܱ�ʶ
                if(Sum >= 0)
                {                    
                    pAvoidModule->ObstAvoid.AvoidEnableFlag = pAvoidModule->ObstAvoid.unAvoidFlag&0x0F;
                }
                else
                {
                    pAvoidModule->ObstAvoid.AvoidEnableFlag = pAvoidModule->ObstAvoid.unAvoidFlag&0x30;
                }
            }
            else
            {
                action = 2;             //!< action none
            }            
        }
        else if((Result&0xF0)==WHEELS_MOVEMENTSTATUS_TURN_STOP)
        {
            if(((Result == WHEELS_MOVEMENTSTATUS_TURN_AHEAD)&&((pAvoidModule->ObstAvoid.unAvoidFlag&0x0F) != 0))||((Result == WHEELS_MOVEMENTSTATUS_TURN_BACK)&&((pAvoidModule->ObstAvoid.unAvoidFlag&0x30) != 0)))
            {
                action = 1;

                //!< ���±���ʹ�ܱ�ʶ
                if(Result == WHEELS_MOVEMENTSTATUS_TURN_AHEAD)
                {                    
                    pAvoidModule->ObstAvoid.AvoidEnableFlag = pAvoidModule->ObstAvoid.unAvoidFlag&0x0F;
                }
                else
                {
                    pAvoidModule->ObstAvoid.AvoidEnableFlag = pAvoidModule->ObstAvoid.unAvoidFlag&0x30;
                }
            }
            else if((Result == WHEELS_MOVEMENTSTATUS_TURN_STOP)&&((pAvoidModule->ObstAvoid.unAvoidFlag&0x3F) != 0))
            {
                action = 1;

                //!< ���±���ʹ�ܱ�ʶ                                                    
                pAvoidModule->ObstAvoid.AvoidEnableFlag = pAvoidModule->ObstAvoid.unAvoidFlag&0x3F;                
            }
            else
            {
                action = 2;
            }
        }
        else
        {
            action = 2;
        }        
    }
    else
    {
        pAvoidModule->ObstAvoid.WheelMove.Result = 0x00;
        pAvoidModule->ObstAvoid.WheelMove.Sum = 0x00;
        pAvoidModule->ObstAvoid.WheelMove.ResUpdat = 0x00;
    }

    if(action==1)
    {
        if( pModule->mControl.mCtrlMode == Ctrl_Mode_Posit)
        {            
            for(i=0;i<Motor_Num_Wheel;i++)
            {
                slvID[i] = i+1;
                                
                pd = &(pModule->mState.pFromMotor+i)->mRegs[0] ;
                                
                sPos = pd[15] + (pd[16]<<8) + (pd[17]<<16) + (pd[18]<<24);
                sSpd = MOTOR_DEFAULT_SPEED_MIN;
                        
                tmpDat[i*8] = (u8)sSpd;
                tmpDat[(i*8)+1] = (u8)(sSpd>>8);
                tmpDat[(i*8)+2] = 0x00;
                tmpDat[(i*8)+3] = 0x00;                            
                tmpDat[(i*8)+4] = (u8)sPos;
                tmpDat[(i*8)+5] = (u8)(sPos>>8);
                tmpDat[(i*8)+6] = (u8)(sPos>>16);
                tmpDat[(i*8)+7] = (u8)(sPos>>24);                
            }
                
            res = sysGeneralMotorBroadcastWrite(slvID[0],Motor_Num_Wheel,EwayMotorReg_TargSpd_L,8,tmpDat,EWAYBOT_WHEEL);
                
            if(ERR_NONE != res)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    SysLogWrite(LogMajor,"NoWheelCmdObstAvoidProc-2:1,Wheels Send CurrPos(0x56-8)to Stop failed.rt:%d.",res);
                            
                    res = Bsp_printf("NoWheelCmdObstAvoidProc-2:1,Wheels Send CurrPos(0x56-8)to Stop failed.rt:%d.",res);
                }
            }
            else
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)\
                &&(pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
                {                                                                            
                    res = Bsp_printf("NoWheelCmdObstAvoidProc-2:1,Wheels Send CurrPos(0x56-8)to Stop succ!");
                }
            }                            
        }
        else if(pModule->mControl.mCtrlMode == Ctrl_Mode_Speed)
        {                
            sSpd = 0;

            for(i=0;i<Motor_Num_Wheel;i++)
            {
                slvID[i] = i+1;                              
                            
                tmpDat[i*2] = (u8)sSpd;
                tmpDat[(i*2)+1] = (u8)(sSpd>>8);
            }
                        
            res = sysGeneralMotorBroadcastWrite(slvID[0],Motor_Num_Wheel,EwayMotorReg_TargSpd_L,2,tmpDat,EWAYBOT_WHEEL);            
                
            if(ERR_NONE != res)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    SysLogWrite(LogMajor,"NoWheelCmdObstAvoidProc-2:1,Wheels Send Spd=0(0x56-2)to Stop failed.rt:%d.",res);
                            
                    res = Bsp_printf("NoWheelCmdObstAvoidProc-2:1,Wheels Send Spd=0(0x56-2)to Stop failed.rt:%d.",res);
                }
            }
            else
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)\
                &&(pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
                {
                    res = Bsp_printf("NoWheelCmdObstAvoidProc-2:1,Wheels Send Spd=0(0x56-2)to Stop succ!");
                }
            }                     
        }
    }
    else
    {
        //!< do nothing
    }    
    
    return ERR_NONE;
}





/*
Ĭ������Ӧ��ģʽһֱΪ����ģʽ:
      ����λ�����λ�ÿɸ�����Ҫ���ã�
      ����ģʽ���ٶ�λ��ģʽ�����ã�

�����·�ָ�������:

(1) �������λ��ͨ��״̬��
		��Ϊͨ�ŶϿ�״̬�����·�ɲ��ָ�������ERR_NONE
		��Ϊͨ������״̬�������(2)

(2) ���ָ����Ƿ���ָ��
		����ָ��򷵻�
		����ָ������(3)

(3) ȡָ����ж�ָ������
		��ֻȡ��1������ָ������(4)
		���������Ӿ���ָ����ж�2�����ӵ�ָ�������Ƿ�һ��
			����������ָ�ͬ�����ӵ�������ָ������ش������
			����������ָ����ͬ�������(4)

(4) �жϵ�ǰ����ָ��ģʽ�Ƿ����ȡ����ָ��ģʽ�Ƿ���ͬ
		��ָ��ģʽ�뵱ǰ��������ģʽ��ͬ�����·�������������ģʽָ��(λ��or�ٶȣ����or����)�������ĵ�ǰϵͳ�洢����������ģʽ
		��ָ��ģʽ�뵱ǰ��������ģʽ��ͬ�������(5)

(5) ��ָ��Ϊ�ٶ�ָ��·��ٶ�ָ��
    ��ָ��Ϊλ��ָ����·�λ��ָ��

(6) ����ERR_NONE    

*/


/*
���֣��ٶ�ģʽ�£�speed>0 ǰ���ķ���
���֣��ٶ�ģʽ�£�speed<0 ǰ���ķ���

���֣�λ��ģʽ�£�PosX > 0 ,��Pos1      > Pos0      ǰ���ķ���
���֣�λ��ģʽ�£�PosX < 0 ,��abs(Pos1) > abs(Pos0) ǰ���ķ���

*/


/* --------------------------------------------------------------------------*/
/**
* @name g_WheelMovementCmdAnalysis
* @brief 
* @details ������λ��������ָ�����ͼ����ӵ�ǰ�����ֵ�״̬���������·�����ָ�������

*
* @param[in] None
*
* @returns      *pRslt = 1  
*               *pRslt = 2
*               *pRslt = 3
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
/*
�����˶�ָ���������

(1) �жϵ�ǰ���Ϲ����Ƿ�򿪲�����6���״����м�鵽�ڹ涨��Χ�ڵ��ϰ���
        �����Ϲ���δ��or���״��ⷶΧ�����ϰ����*pRslt = 2(�����ճ��·��κ���λ����ָ��)����
        �񣬼���(2)

(2) ����ָ�����ͺ������ֵ��ٶ�λ�ã��жϵ�ǰ����ǰ��or����or��ֹ
        ��ǰ���״��⵽�ڹ涨��Χ�����ϰ���Ҵ�����ָ���뵱ǰ���ӵ�λ��or�ٶȱȽ��ж�Ϊǰ������*pRslt = 1(����ָֹͣ��)����
        �����״��⵽�ڹ涨��Χ�����ϰ���Ҵ�����ָ���뵱ǰ���ӵ�λ��or�ٶȱȽ��ж�Ϊ���ˣ���*pRslt = 1(����ָֹͣ��)����
        ��������*pRslt = 2(�����ճ��·��κ���λ����ָ��)


*/
/*
s8 g_WheelMovementCmdAnalysis(u8* pRslt,u16 fCode,u8* pLd,u8* pRd)
{
    u8 rslt=0;
    u8* pd0;
    u8* pd1;
    s16 spd[2]={0};
    s32 pos[2]={0};
    s32 deltPos[2] = {0};

    if((pRslt==NULL)||(pLd==NULL)||(pRd==NULL))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    //!< �жϵ�ǰ����״̬��־
    if((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag == 0)||(EwayRadarObstAvoid.ObstAvoid.Swi==0))       //�����迼�Ǳ��ϣ�ָ���ճ��·�
    {
        *pRslt = 2;       //!< ����2��ʾ�����ճ��·�ָ��

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-1,res2,return:AvoidSwi:%x,AvoidFlag:0x%x.",EwayRadarObstAvoid.ObstAvoid.Swi,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
                
        return ERR_NONE;
    }

    //!< �жϵ�ǰ��ָ����ǰ�����Ǻ���
    if(fCode == PCCmd_WheelPositionMove)
    {    
        pos[0] = pLd[8] + (pLd[9]<<8) + (pLd[10]<<16) + (pLd[11]<<24);      //!< ������ָ���Pos

        pos[1] = pRd[8] + (pRd[9]<<8) + (pRd[10]<<16) + (pRd[11]<<24);

        pd0 = &EwayWheels.mState.pFromMotor->mRegs[0] ;                      //!< ��ǰ���ӵ�Pos

        deltPos[0] = pos[0] - (pd0[15] + (pd0[16]<<8) + (pd0[17]<<16) + (pd0[18]<<24));
        
        pd1 = &((EwayWheels.mState.pFromMotor+1)->mRegs[0]);

        deltPos[1] = pos[1] - (pd1[15] + (pd1[16]<<8) + (pd1[17]<<16) + (pd1[18]<<24));

        if(((deltPos[0] > 0) && (deltPos[1] < 0))||((deltPos[0] == 0)&&(deltPos[1] < 0))||((deltPos[0] > 0)&&(deltPos[1] == 0)))
        {
            rslt = 2;               //!< ǰ��
        }
        else if(((deltPos[0] < 0) && (deltPos[1] > 0))||((deltPos[0] == 0) && (deltPos[1] > 0))||((deltPos[0] < 0) && (deltPos[1] == 0)))
        {
            rslt = 1;               //!< ����
        }
        else
        {
            rslt = 3;               //!< �������
        }

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMovAnaly-2.1 CurPos:0x%x-0x%x,CmdPos:0x%x-0x%x,deltPos:0x%x-0x%x.",(pd0[15] + (pd0[16]<<8) + (pd0[17]<<16) + (pd0[18]<<24)),\
                (pd1[15] + (pd1[16]<<8) + (pd1[17]<<16) + (pd1[18]<<24)),pos[0],pos[1],deltPos[0],deltPos[1]);
        }
        
    }
    else if(fCode==PCCmd_WheelSpeedMove)
    {
        spd[0] = pLd[8] + (pLd[9] << 8);

        spd[1] = pRd[8] + (pRd[9] << 8);

        if(((spd[0] < 0)&&(spd[1] > 0))||((spd[0] == 0)&&(spd[1] > 0))||((spd[0] < 0)&&(spd[1] == 0)))                      //ָ��Ϊ����
        {
            rslt = 1;               //!< ����
        }
        else if(((spd[0] > 0)&&(spd[1] < 0))||((spd[0] == 0)&&(spd[1] < 0))||((spd[0] > 0)&&(spd[1] == 0)))                 //ָ��Ϊǰ��
        {
            rslt = 2;               //!< ǰ��
        }
        else 
        {
            rslt = 3;               //!< �������
        }
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMovAnaly-2.1 CmdSpeed:0x%x-0x%x",spd[0],spd[1]);//!< rslt�ǵ�ǰָ��ᵼ�µ��˶�����
        }
    }

    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
    {
        Bsp_printf("WhlMovAnaly-2.2,0x%04x,rslt:%d(1-goback,2-goforward,3-others).",fCode,rslt);//!< rslt�ǵ�ǰָ��ᵼ�µ��˶�����
    }
    
    if(((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x0F) != 0)&&(rslt==2))       //ǰ��4�����ϱ�ʶ�д򿪣���Ϊǰ��ָ��
    {
        *pRslt = 1;             //!< ����ָֹͣ��

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-1(Stop),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }
    else if(((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x30) != 0)&&(rslt==1))//!< �󷽱�ʶ�д򿪣���Ϊ����ָ��
    {
        *pRslt = 1;             //!< ����ָֹͣ��
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-1(Stop),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }
    else 
    {
        *pRslt = 2;             //!< �ճ�����
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-2(Move),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }

    return ERR_NONE;    

}*/
s8 g_WheelMovementCmdAnalysis(u8* pRslt,u16 fCode,u8* pLd,u8* pRd)
{
    u8 rslt=0;
    u8* pd0;
    u8* pd1;
    s16 spd[2]={0};
    s32 pos[2]={0};
    s32 deltPos[2] = {0};

    if((pRslt==NULL)||(pLd==NULL)||(pRd==NULL))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    //!< �жϵ�ǰ����״̬��־
    if((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag == 0)||(EwayRadarObstAvoid.ObstAvoid.Swi==0))       //�����迼�Ǳ��ϣ�ָ���ճ��·�
    {
        *pRslt = 2;       //!< ����2��ʾ�����ճ��·�ָ��

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-1,res2(SendCmd),return:AvoidSwi:%x,AvoidFlag:0x%x.",EwayRadarObstAvoid.ObstAvoid.Swi,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
                
        return ERR_NONE;
    }

    //!< �жϵ�ǰ��ָ����ǰ�����Ǻ���
    if(fCode == PCCmd_WheelPositionMove)
    {    
        pos[0] = pLd[8] + (pLd[9]<<8) + (pLd[10]<<16) + (pLd[11]<<24);      //!< ������ָ���Pos

        pos[1] = pRd[8] + (pRd[9]<<8) + (pRd[10]<<16) + (pRd[11]<<24);

        pd0 = &EwayWheels.mState.pFromMotor->mRegs[0] ;                      //!< ��ǰ���ӵ�Pos

        deltPos[0] = pos[0] - (pd0[15] + (pd0[16]<<8) + (pd0[17]<<16) + (pd0[18]<<24));
        
        pd1 = &((EwayWheels.mState.pFromMotor+1)->mRegs[0]);

        deltPos[1] = pos[1] - (pd1[15] + (pd1[16]<<8) + (pd1[17]<<16) + (pd1[18]<<24));

        if(((deltPos[0] > 0) && (deltPos[1] < 0))||((deltPos[0] == 0)&&(deltPos[1] < 0))||((deltPos[0] > 0)&&(deltPos[1] == 0)))
        {
            rslt = 2;               //!< ǰ��
        }
        else if(((deltPos[0] < 0) && (deltPos[1] > 0))||((deltPos[0] == 0) && (deltPos[1] > 0))||((deltPos[0] < 0) && (deltPos[1] == 0)))
        {
            rslt = 1;               //!< ����
        }
        else if((deltPos[0] > 0) && (deltPos[1] > 0))
        {
            if(deltPos[0]>deltPos[1])
            {
                rslt = 2;    //!< ����ǰ
            }
            else if(deltPos[0]<deltPos[1])
            {
                rslt = 1;    //!< �����
            }
            else
            {
                rslt = 3;    //!< ԭ��ת��˳ʱ��
            }
        }
        else if((deltPos[0] < 0) && (deltPos[1] < 0))
        {
            if(abs(deltPos[0]) > abs(deltPos[1]))
            {
                rslt = 1;    //!< ���Һ�
            }
            else if(abs(deltPos[0]) < abs(deltPos[1]))
            {
                rslt = 2;    //!< ����ǰ
            }
            else
            {
                rslt = 3;    //!< ԭ��ת����ʱ��
            }

        }
        else if((deltPos[0] == 0) && (deltPos[1] == 0))
        {
            rslt = 4;       //!< ���ֵ�ǰλ��
        }

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMovAnaly-2.1 CurPos:0x%x-0x%x,CmdPos:0x%x-0x%x,deltPos:0x%x-0x%x.",(pd0[15] + (pd0[16]<<8) + (pd0[17]<<16) + (pd0[18]<<24)),\
                (pd1[15] + (pd1[16]<<8) + (pd1[17]<<16) + (pd1[18]<<24)),pos[0],pos[1],deltPos[0],deltPos[1]);
        }        
    }
    else if(fCode==PCCmd_WheelSpeedMove)
    {
        spd[0] = pLd[8] + (pLd[9] << 8);

        spd[1] = pRd[8] + (pRd[9] << 8);

        if(((spd[0] < 0)&&(spd[1] > 0))||((spd[0] == 0)&&(spd[1] > 0))||((spd[0] < 0)&&(spd[1] == 0)))                      //ָ��Ϊ����
        {
            rslt = 1;               //!< ����
        }
        else if(((spd[0] > 0)&&(spd[1] < 0))||((spd[0] == 0)&&(spd[1] < 0))||((spd[0] > 0)&&(spd[1] == 0)))                 //ָ��Ϊǰ��
        {
            rslt = 2;               //!< ǰ��
        }
        else if((spd[0] > 0)&&(spd[1] > 0))
        {
            if(spd[0] > spd[1])
            {
                rslt = 2;    //!< ����ǰ
            }
            else if(spd[0] < spd[1])
            {
                rslt = 1;    //!< �����
            }
            else
            {
                rslt = 3;    //!< ԭ��ת��˳ʱ��
            }
        }
        else if((spd[0] < 0)&&(spd[1] < 0))
        {
            if(abs(spd[0]) > abs(spd[1]))
            {
                rslt = 1;    //!< ���Һ�
            }
            else if(abs(spd[0]) < abs(spd[1]))
            {
                rslt = 2;    //!< ����ǰ
            }
            else
            {
                rslt = 3;    //!< ԭ��ת����ʱ��
            }
        }
        else if((spd[0] == 0)&&(spd[1] == 0))       //!< ���ֵ�ǰλ��
        {
            rslt = 4;
        }
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMovAnaly-2.1 CmdSpeed:0x%x-0x%x",spd[0],spd[1]);//!< rslt�ǵ�ǰָ��ᵼ�µ��˶�����
        }
    }

    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
    {
        Bsp_printf("WhlMovAnaly-2.2,0x%04x,CmdRslt:%d(1-goback,2-goforward,3-turn,4-stop,0-NoInit).",fCode,rslt);//!< rslt�ǵ�ǰָ��ᵼ�µ��˶�����
    }
    
    if(((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x0F) != 0)&&(rslt==2))       //ǰ��4�����ϱ�ʶ�д򿪣���Ϊǰ��ָ��
    {
        *pRslt = 1;             //!< ����ָֹͣ��

        //!< ���±���ʹ�ܱ�ʶ                   
        EwayRadarObstAvoid.ObstAvoid.AvoidEnableFlag = EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x0F;

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-1(Stop),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }
    else if(((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x30) != 0)&&(rslt==1))//!< �󷽱�ʶ�д򿪣���Ϊ����ָ��
    {
        *pRslt = 1;             //!< ����ָֹͣ��

        //!< ���±���ʹ�ܱ�ʶ                   
        EwayRadarObstAvoid.ObstAvoid.AvoidEnableFlag = EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x30;
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-1(Stop),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }
    else if(((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x3F) != 0)&&(rslt==3))      //!< ԭ��תȦ�����6��Radar
    {
        *pRslt = 1;             //!< ����ָֹͣ��
        
        //!< ���±���ʹ�ܱ�ʶ                   
        EwayRadarObstAvoid.ObstAvoid.AvoidEnableFlag = EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x3F;
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-1(Stop),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }
    else
    {
        *pRslt = 2;             //!< �ճ�����
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-2(Move),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }

    return ERR_NONE;    

}


/*      20180509 ��ӱ��Ϲ��ܣ��޸Ĵ����߼�      ling
Ĭ������Ӧ��ģʽһֱΪ����ģʽ:
      ����λ�����λ�ÿɸ�����Ҫ���ã�
      ����ģʽ���ٶ�λ��ģʽ�����ã�

�����·�ָ�������:

(1) �������λ��ͨ��״̬��
		��Ϊͨ�ŶϿ�״̬�����·�ɲ��ָ�������ERR_NONE
		��Ϊͨ������״̬�������(2)

(2) ���ָ����Ƿ���ָ��
		����ָ��򷵻�
		����ָ������(3)

(3) ȡָ����ж�ָ������
		��ֻȡ��1������ָ������(4)
		���������Ӿ���ָ����ж�2�����ӵ�ָ�������Ƿ�һ��
			����������ָ���벻ͬ�����ӵ�������ָ������ش������
			����������ָ������ͬ�������(4)

(4) �жϵ�ǰ����ָ��ģʽ�Ƿ����ȡ����ָ��ģʽ�Ƿ���ͬ
		��ָ��ģʽ�뵱ǰ��������ģʽ��ͬ�����·�������������ģʽָ��(λ��or�ٶȣ����or����)�������ĵ�ǰϵͳ�洢����������ģʽ
		��ָ��ģʽ�뵱ǰ��������ģʽ��ͬ�������(5)

(5) ����ָ�����͡�ָ�����ݡ�����״̬������������2��:
        ֱ���·���ǰָ�����(7)
        �·�ָֹͣ�����(6)

(6) ��Ϊ�·�ָֹͣ��
        λ��ģʽ����ָֹͣ��·��������˳�
        �ٶ�ģʽ����ָֹͣ��·��������˳�

(7) ��ֱ���·���ǰָ��
        λ��ģʽ��������·��������˳�
        �ٶ�ģʽ��������·��������˳�
        
(8) ����ERR_NONE    
*/

/*      20180525 �����Clearָ����Ƿ�λ�ñ��֣�����ģʽ�л�ָ�����ϵ��ٶ�λ��ָ���У��޸�      ling
Ĭ������Ӧ��ģʽһֱΪ����ģʽ:
      ����λ�����λ�ÿɸ�����Ҫ���ã�
      ����ģʽ���ٶ�λ��ģʽ�����ã�

�����·�ָ�������:

(1) �������λ��ͨ��״̬��
		��Ϊͨ�ŶϿ�״̬�����·�ɲ��ָ�������ERR_NONE
		��Ϊͨ������״̬�������(2)

(2) ���ָ����Ƿ���ָ��
		����ָ��򷵻�
		����ָ������(3)

(3) ȡָ����ж�ָ������
		��ֻȡ��1������ָ������(4)
		���������Ӿ���ָ����ж�2�����ӵ�ָ�������Ƿ�һ��
			����������ָ���벻ͬ�����ӵ�������ָ������ش������
			����������ָ������ͬ�������(4)

(4) �жϵ�ǰ����ָ��ģʽ�Ƿ����ȡ����ָ��ģʽ�Ƿ���ͬ
		��ָ��ģʽ�뵱ǰ��������ģʽ��ͬ�����·�������������ģʽָ��(λ��or�ٶȣ����or����)�������ĵ�ǰϵͳ�洢����������ģʽ
		��ָ��ģʽ�뵱ǰ��������ģʽ��ͬ�������(5)

(5) ����ָ�����͡�ָ�����ݡ�����״̬������������2��:
        ֱ���·���ǰָ�����(7)
        �·�ָֹͣ�����(6)

(6) ��Ϊ�·�ָֹͣ��
        λ��ģʽ����ָֹͣ��·��������˳�
        �ٶ�ģʽ����ָֹͣ��·��������˳�

(7) ��ֱ���·���ǰָ��
        λ��ģʽ��������·��������˳�
        �ٶ�ģʽ��������·��������˳�
        
(8) ����ERR_NONE    
*/
/*
20180710��Ӷ����ӵ��0xC0�Ĵ����Ĳ�ѯ���޸Ĵ�������


(1) �������λ��ͨ��״̬��
		��Ϊͨ�ŶϿ�״̬�����·�����ֹͣ(0x51-2)ָ�������ERR_NONE
		��Ϊͨ������״̬�������(2)

(2) ���ָ����Ƿ���ָ��
		����ָ������(3)
		����ָ������(4)

(3) ��鵱ǰ�Ƿ���Clearָ�������
        ��û�У��򷵻�ERR_NONE;
        ���У����鵱ǰ����2�������ReadyReg�Ƿ��Ѿ�׼���ã�
                ����û��׼���ã������Clear�����־�������ء�
                ���Ѿ�׼���ã�����Clearָ���Ƿ���Ҫλ�ñ���
        
                                    ��Ҫλ�ñ��֣���ǰΪλ��ģʽ���͵�ǰλ�ü�Ĭ���ٶȸ��������ӵ���������Clear��־������ERR_None
                                                ��ǰΪ�ٶ�ģʽ����ģʽת��Ϊλ�õ�ָ������͵�ǰλ�ø��������ӵ���������Clear��־������ERR_None
                        
                                    ����Ҫλ�ñ��֣�ʲô�������������Clear��־������ERR_None

(4) ȡָ����ж�ָ������
		��ֻȡ��1������ָ������(5)
		���������Ӿ���ָ����ж�2�����ӵ�ָ�������Ƿ�һ��
			����������ָ�ͬ�����ӵ�������ָ������ش������
			����������ָ����ͬ�������(5)

(5) �жϵ�ǰ����ָ��ģʽ�Ƿ����ȡ����ָ��ģʽ�Ƿ���ͬ
		��ָ��ģʽ�뵱ǰ��������ģʽ��ͬ���������ģʽ���ݰ�������¼������(6)
		��ָ��ģʽ�뵱ǰ��������ģʽ��ͬ�������(6)

(6) ���ñ��Ͽ��صĺ�������������ǰ�Ƿ�Ҫֹͣ�������ճ��·�ָ��
            ��Ϊ����ָֹͣ�����ָֹͣ��
            ���ճ�����ָ�����
                    ��ָ��Ϊ�ٶ�ָ�������Ƿ���ģʽת��ָ����·��ٶ�ָ��(��֤ģʽת��ָ�����ٶ�ָ����ͬһ����)
                    ��ָ��Ϊλ��ָ�������Ƿ���ģʽת��ָ����·�λ��ָ��(��֤ģʽת��ָ�����ٶ�ָ����ͬһ����)

(7) ����ERR_NONE    


*/

s8 sysModuleWheelsMovement(EwayWheelModule* pModule)
{ 	
	u8 tmpDat[20]={0};
    u8 modeDat[4]={0};
	u16 fCode;
	u8 dat[Motor_Num_Wheel][CMD_PC_2_EMB_BUFF_LEN]={0};//!< ���ڴ�Ŵӻ��������18byte����.
	u8 i,slvNums=0;
	s8 res;
    u8 rslt;
	u8 slvID[2]={0};
	s16 sSpd;
	s32 sPos;
    u8* pd;
    u8 tmp,tmp1;
	MotorCMDBuff* pmotorCmd = pModule->mControl.pToMotorCmd;
	
	//!< �������λ��ͨ��״̬����Ϊ�Ͽ����������·��ٶ�Ϊ0��ָ��orɲ����ָ��orλ��ָ���֮����Ҫͣ������
	if((EwayEmbSys.Comm.wPc & 0x01) == EMB_COMM_STATUS_DISCONNECTED)
	{	
		//!< ����δ���ӣ���������Ƿ�Ϊ����ֹͣ״̬
        if(EwayWheels.MotorStop != StopMode_FreeOn)        
        {
            sysPutWheelMotorsIntoFreeStopMode();
        }
		return ERR_NONE;
	}

	for(i=0;i<(pModule->mCount);i++)
	{		
		if(((pmotorCmd+i)->dCmdCnt) > 0 )
		{
			break;
		}
	}	

	if(i >= pModule->mCount)                    //!< ָ����в��޴���ָ��.
	{
	    //!< ��鵱ǰ�Ƿ���Clearָ�������
        if((pModule->mControl.posKeep&0xFF00)!=0)     //!< �յ���Clearָ��
        {
//   20180711
#if CheckWheelMotorReadyBit_Def
            if(pModule->mRdyStatus!=0x03)        //!< ������ӵ���Ƿ��Ѿ�׼���ã���û��׼���ã�����Clear�����־��ֱ�ӷ��أ��������κ����ݸ�����
            {
                pModule->mControl.posKeep = 0x0000; 
                
                return ERR_NONE;
            }
#endif
            tmp = (u8)pModule->mControl.posKeep;
                    
            if(tmp == 'K')       //!< ��Ҫλ�ñ���
            {
                if(pModule->mControl.mCtrlMode == Ctrl_Mode_Posit)
                {
                    //!< pos mode,send Current Pos and speed=minimum value
                    
                    for(i=0;i<Motor_Num_Wheel;i++)
                    {                                   //!< Reg:
                        tmpDat[(i<<3)] = (u8)MOTOR_DEFAULT_SPEED_MIN;          //!< 0x56
                        tmpDat[(i<<3)+1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);        //!< 0x57
            
                        tmpDat[(i<<3)+2] = 0x00;        //!< 0x58
                        tmpDat[(i<<3)+3] = 0x00;        //!< 0x59
                        memcpy(&tmpDat[(i<<3)+4],(u8*)(&sysEwServo[Emb_StartID_Wheel+14+i].uPosWheel),4);   //!< ȡPosition��4���ֽ��·���ȥ
                    }

                    res = sysGeneralMotorBroadcastWrite(Emb_StartID_Wheel,Motor_Num_Wheel,EwayMotorReg_TargSpd_L,8,tmpDat,EWAYBOT_WHEEL);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            res = Bsp_printf("2.1 Wheel Keep Pos Cmd, Posi mode,send(0x56-8) failed.");
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                        {
                            res = Bsp_printf("2.1 Wheel Keep Pos Cmd, Posi mode,send(0x56-8) spd=0,pos:0x%02x-%02x-%02x-%02x!",tmpDat[7],tmpDat[6],tmpDat[5],tmpDat[4]);
                        }
                    }                    
                }
                else if(pModule->mControl.mCtrlMode == Ctrl_Mode_Speed)
                {
                    //!< �л�ģʽΪλ��                    
                    tmpDat[0] = EWAY_MOTOR_CONTROL_MODE_POSITION;
                    tmpDat[1] = EWAY_MOTOR_CONTROL_MODE_POSITION;
                
                    res = sysGeneralMotorBroadcastWrite(Emb_StartID_Wheel,Motor_Num_Wheel,EwayMotor_CtrlMode,1,tmpDat,EWAYBOT_WHEEL);
                
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            SysLogWrite(LogMajor,"err:Wheel Keep Pos Cmd,Change Speed mode to Pos failed.rt:%d.line:%d",res,__LINE__);
                            
                            res = Bsp_printf("err:Wheel Keep Pos Cmd,Change Speed mode to Pos failed.rt:%d.line:%d",res,__LINE__);                            
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
                        &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                        {                                                                            
                            Bsp_printf("Info:Wheel Keep Pos Cmd,Change Speed mode to Pos succ!");
                        }
                    }

                    //!< ����ϵͳ�ݴ��е�ģʽ
                    EwayWheels.mControl.mCtrlMode = Ctrl_Mode_Posit;                    

                    //!< �·���ǰλ��
                    for(i=0;i<Motor_Num_Wheel;i++)
                    {                    
                        sSpd = MOTOR_DEFAULT_SPEED_MIN;
                                
                        pd = &(EwayWheels.mState.pFromMotor+i)->mRegs[0] ;                                
                        sPos = pd[15] + (pd[16]<<8) + (pd[17]<<16) + (pd[18]<<24);
                                              
                        tmpDat[i<<3] = (u8)sSpd;
                        tmpDat[(i<<3)+1] = (u8)(sSpd>>8);
                        tmpDat[(i<<3)+2]= 0x00;
                        tmpDat[(i<<3)+3]= 0x00;        
                        tmpDat[(i<<3)+4] = (u8)sPos;
                        tmpDat[(i<<3)+5] = (u8)(sPos>>8);
                        tmpDat[(i<<3)+6] = (u8)(sPos>>16);
                        tmpDat[(i<<3)+7] = (u8)(sPos>>24);            
                    }                        

                    res = sysGeneralMotorBroadcastWrite(Emb_StartID_Wheel,Motor_Num_Wheel,EwayMotorReg_TargSpd_L,8,tmpDat,EWAYBOT_WHEEL);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            SysLogWrite(LogMajor,"err:Wheel Keep Pos Cmd,Change Speed mode to Pos and send CurrPos&spd cmd to Stop failed.rt:%d.line:%d",res,__LINE__);
                            
                            res = Bsp_printf("err:Wheel Keep Pos Cmd,Change Speed mode to Pos and send CurrPos&spd cmd to Stop failed.rt:%d.line:%d",res,__LINE__);
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
                        &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                        {                                                        
                            res = Bsp_printf("Info:Wheel Keep Pos Cmd,Change Speed mode to Pos and send CurrPos:0x%x to Stop succ!",sPos);
                        }
                    }                    
                }          
            }
            else if(tmp == 'N')  //!< ����Ҫλ�ñ���
            {/*     20180525 ���۵�������Ҫλ�ñ��֣��������κβ���         ling
            
                //!< ����λ�û����ٶ�ģʽ������0x51�Ĵ����·�����ָֹͣ��
                tmpDat[0] = 2;                  //!< reg0x51:0-����ֹͣ,1-�����ƶ�,2-����ֹͣ
                tmpDat[1] = 2;
                
                res = sysGeneralMotorBroadcastWrite(Emb_StartID_Head,Motor_Num_Head,EwayMotor_Stop,1,tmpDat,EWAYBOT_HEAD);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("2.1 Hd Not Keep Pos Cmd,BrodcstWr(0x51-2) failed.");
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                    {
                        res = Bsp_printf("2.1 Hd Not Keep Pos Cmd,BrodcstWr(0x51-2) Suc!");
                    }
                }*/
            }
        
            pModule->mControl.posKeep = 0x0000; 
        
            //return ERR_NONE;
        }

        //!< ����ָ��ʱ����鵱ǰ�Ƿ���Ҫ����
        if(EwayRadarObstAvoid.ObstAvoid.Swi != 0)
        {
            g_WheelMovementObstacleAvoidProcess(pModule,&EwayRadarObstAvoid);
        }

		return ERR_NONE;    
	}
	
	slvNums = 0;
	for(i=0;i<(pModule->mCount);i++)
	{
		//!< ����id�Ų�ѯ��Ӧָ����Ƿ��д���ָ��
		if(((pmotorCmd+i)->dCmdCnt) > 0 )
		{
			//res = sysGeneralGetEmbToMotorCmd((pmotorCmd+i),&dat[slvNums][0]);	  //!< ȡ��ָ��
			res = g_GetPCCmdEmbToMotorCmd(pmotorCmd,i,&dat[slvNums][0]);

            //14Bytes//  2By    4By  2By    4By    2By            λ��ģʽ
            //10Bytes//  2By    4By  2By    0By    2By            �ٶ�ģʽ
            //!< ����ָ���16��14bytes:Code0-1 T0-3 iD0-1 Pos0-3 Speed0-1
            
			if(res==ERR_NONE)
			{
				slvID[slvNums] = dat[slvNums][6];						      //!< dat[6]&dat[7] ��ŵ���ID�ţ�ȡ���ֽ�
					
				slvNums += 1;
			}				
		}
	}

	if(slvNums>=2)
	{
		if(dat[0][0]!=dat[1][0])     //!< �Ƚ�����ָ���fCode�Ƿ���ͬ������ͬ����ֱ�ӷ���
		{
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
			    SysLogWrite(LogMajor,"WheelMov-3,CMDs not equal.");

			    res = Bsp_printf("WheelMov-3,CMDs not equal");
            }       

            return ERR_PROCESS_FAILED;
		}
	}
//   20180711
#if CheckWheelMotorReadyBit_Def

    if(pModule->mRdyStatus!=0x03)        //!< ������ӵ���Ƿ��Ѿ�׼���ã���û��׼���ã�����Clear�����־��ֱ�ӷ��أ��������κ����ݸ�����
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
            &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
        {
            Bsp_printf("Wheel Motros is not Ready(0x%x),return.",pModule->mRdyStatus);
        }
        
        return ERR_NONE;
    }
#endif

/*
#if CheckWheelMotorCommBeforeSendCmd

    tmp = EwayEmbSys.Comm.wMotors[EMB_COMM_WHEEL_START_BIT]&EMB_COMM_ERR_BITS_3_TIMES;
    tmp1 = EwayEmbSys.Comm.wMotors[EMB_COMM_WHEEL_START_BIT+1]&EMB_COMM_ERR_BITS_3_TIMES;
    if((tmp==0)||(tmp1==0))
    {
        Bsp_printf("Whl offline,throw Mov Cmd.%x,%x.",EwayEmbSys.Comm.wMotors[EMB_COMM_WHEEL_START_BIT],EwayEmbSys.Comm.wMotors[EMB_COMM_WHEEL_START_BIT+1]);
        
        return ERR_NONE;
    }

#endif
*/

	fCode = dat[0][0] + (dat[0][1]<<8);

	//!< ����ǰ���ӿ���ģʽ��ָ���ģʽ����ͬ������Ҫ���Ϳ���ģʽת��ָ��
	if(((fCode==PCCmd_WheelPositionMove)&&(pModule->mControl.mCtrlMode==Ctrl_Mode_Speed))||((fCode==PCCmd_WheelSpeedMove)&&(pModule->mControl.mCtrlMode==Ctrl_Mode_Posit)))
	{
		if(fCode==PCCmd_WheelPositionMove)
        {
            tmpDat[0] = EWAY_MOTOR_CONTROL_MODE_POSITION;
            tmpDat[1] = EWAY_MOTOR_CONTROL_MODE_POSITION;
        }
        else if(fCode==PCCmd_WheelSpeedMove)
        {
            tmpDat[0] = EWAY_MOTOR_CONTROL_MODE_SPEED;
            tmpDat[1] = EWAY_MOTOR_CONTROL_MODE_SPEED;
        }
                
        res = sysGeneralMotorBroadcastWrite(Emb_StartID_Wheel,Motor_Num_Wheel,EwayMotor_CtrlMode,1,tmpDat,EWAYBOT_WHEEL);
                
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                SysLogWrite(LogMajor,"err:Wheel Change Speed/Pos mode to %d(0-spd,1-pos) failed.rt:%d.line:%d",tmpDat[0],res,__LINE__);
                            
                res = Bsp_printf("err:Wheel Change Speed/Pos mode to %d(0-spd,1-pos) failed.rt:%d.line:%d",tmpDat[0],res,__LINE__);                            
            }
        }
        else
        {
            //!< ����ϵͳ�ݴ��е�ģʽ
            if(fCode==PCCmd_WheelPositionMove)
            {
                EwayWheels.mControl.mCtrlMode = Ctrl_Mode_Posit; 
            }
            else if(fCode==PCCmd_WheelSpeedMove)
            {
                EwayWheels.mControl.mCtrlMode = Ctrl_Mode_Speed; 
            }  
            
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
            &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
            {                                                                            
                Bsp_printf("Info:Wheel Change Speed/Pos mode to %d(0-spd,1-pos) succ!",tmpDat[0]);
            }
        }        
	}

    res = g_WheelMovementCmdAnalysis(&rslt,fCode,&dat[0][0],&dat[1][0]);        //!< ����ָ�����͡�ָ�����ݡ�����״̬������

    if(res != ERR_NONE)
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            SysLogWrite(LogNormal,"g_WheelMovementCmdAnalysis() err,rt:%d.",res);

            Bsp_printf("g_WheelMovementCmdAnalysis() err,rt:%d.",res);
        }

        return res;
    }

	//!< �·�ָ��
	memset(tmpDat,0,20);

    if(rslt==1)             //!< ����ָֹͣ�� ��λ��ģʽ�·�λ��Ϊ��ǰλ���ٶ�ΪĬ��ֵ���ٶ�ģʽ�·��ٶ�Ϊ0
    {
        if(fCode==PCCmd_WheelPositionMove)
        {            
            slvNums = 2;

            /*if(modeDat[0] != 0)
            {            
                for(i=0;i<slvNums;i++)
                {
                    memcpy(tmpDat,modeDat,3);
                    sSpd = MOTOR_DEFAULT_SPEED_MIN;
                                
                    pd = &(EwayWheels.mState.pFromMotor+i)->mRegs[0] ;
                                
                    sPos = pd[15] + (pd[16]<<8) + (pd[17]<<16) + (pd[18]<<24);

                    tmpDat[3] = EwayMotorReg_TargSpd_L;
                    tmpDat[4] = 8;                        
                    tmpDat[5] = (u8)sSpd;
                    tmpDat[6] = (u8)(sSpd>>8);
                    tmpDat[7]= 0x00;
                    tmpDat[8]= 0x00;        
                    tmpDat[9] = (u8)sPos;
                    tmpDat[10] = (u8)(sPos>>8);
                    tmpDat[11] = (u8)(sPos>>16);
                    tmpDat[12] = (u8)(sPos>>24);            

                    res = g_GeneralMotorSingleWrite((Emb_StartID_Wheel+i),13,tmpDat,EWAYBOT_WHEEL);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            SysLogWrite(LogMajor,"AvoidRs:1,Wheel(%d) change mode to Pos and Send CurrPos(0x56-8)to Stop failed.rt:0x%x.",i,res);
                            
                            res = Bsp_printf("AvoidRs:1,Wheel(%d) change mode to Pos and Send CurrPos(0x56-8)to Stop failed.rt:0x%x.",i,res);
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
                        &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                        {                                                        
                            res = Bsp_printf("AvoidRs:1,Wheel(%d) change mode to Pos and Send CurrPos(0x56-8)to Stop!",i);
                        }
                    }
                }                
            }
            else*/
            {
                for(i=0;i<slvNums;i++)
                {
                    slvID[i] = i+1;
                                
                    pd = &(EwayWheels.mState.pFromMotor+i)->mRegs[0] ;
                                
                    sPos = pd[15] + (pd[16]<<8) + (pd[17]<<16) + (pd[18]<<24);
                    sSpd = MOTOR_DEFAULT_SPEED_MIN;
                        
                    tmpDat[i*8] = (u8)sSpd;
                    tmpDat[(i*8)+1] = (u8)(sSpd>>8);
                    tmpDat[(i*8)+2] = 0x00;
                    tmpDat[(i*8)+3] = 0x00;                            
                    tmpDat[(i*8)+4] = (u8)sPos;
                    tmpDat[(i*8)+5] = (u8)(sPos>>8);
                    tmpDat[(i*8)+6] = (u8)(sPos>>16);
                    tmpDat[(i*8)+7] = (u8)(sPos>>24);                
                }
                
                res = sysGeneralMotorBroadcastWrite(slvID[0],slvNums,EwayMotorReg_TargSpd_L,8,tmpDat,EWAYBOT_WHEEL);
                
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        SysLogWrite(LogMajor,"AvoidRs:1,Wheels Send CurrPos(0x56-8)to Stop failed.rt:%d.",res);
                            
                        res = Bsp_printf("AvoidRs:1,Wheels Send CurrPos(0x56-8)to Stop failed.rt:%d.",res);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
                    &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                    {                                                                            
                        res = Bsp_printf("AvoidRs:1,Wheels Send CurrPos(0x56-8)to Stop!");
                    }
                }
            }                     
        }
        else if(fCode==PCCmd_WheelSpeedMove)
        {        
            slvNums = 2;
/*
            if(modeDat[0] != 0)
            {
                memcpy(tmpDat,modeDat,3);
                sSpd = 0;               
                
                for(i=0;i<slvNums;i++)
                {
                    tmpDat[3] = EwayMotorReg_TargSpd_L;
                    tmpDat[4] = 2; 
                    tmpDat[5] = (u8)sSpd;
                    tmpDat[6] = (u8)(sSpd>>8);

                    res = g_GeneralMotorSingleWrite((Emb_StartID_Wheel+i),7,tmpDat,EWAYBOT_WHEEL);
                    
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            SysLogWrite(LogMajor,"AvoidRs:1,Wheel(%d) change mode to Spd and Send Spd=0(0x56-2)to Stop failed.rt:0x%x.",i,res);
                            
                            res = Bsp_printf("AvoidRs:1,Wheel(%d) change mode to Spd and Send Spd=0(0x56-2)to Stop failed.rt:0x%x.",i,res);
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
                        &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                        {                                                        
                            res = Bsp_printf("AvoidRs:1,Wheel(%d) change mode to Spd and Send Spd=0(0x56-2)to Stop!",i);
                        }
                    }
                }

            }
            else*/
            {                  
                sSpd = 0;

                for(i=0;i<slvNums;i++)
                {
                    slvID[i] = i+1;                              
                            
                    tmpDat[i*2] = (u8)sSpd;
                    tmpDat[(i*2)+1] = (u8)(sSpd>>8);
                }
                        
                res = sysGeneralMotorBroadcastWrite(slvID[0],slvNums,EwayMotorReg_TargSpd_L,2,tmpDat,EWAYBOT_WHEEL);            
                
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        SysLogWrite(LogMajor,"AvoidRs:1,Wheels Send Spd=0(0x56-2)to Stop failed.rt:%d.",res);
                            
                        res = Bsp_printf("AvoidRs:1,Wheels Send Spd=0(0x56-2)to Stop failed.rt:%d.",res);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
                    &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                    {
                        res = Bsp_printf("AvoidRs:1,Wheels Send Spd=0(0x56-2)to Stop!");
                    }
                }
            }            
        }
        
        return ERR_NONE;
    }
    else if(rslt==2)        //!< �ճ�����ָ��
    {
        if(fCode==PCCmd_WheelPositionMove)
	    {
	        /*if(modeDat[0] != 0)
            {
                memcpy(tmpDat,modeDat,3);

                for(i=0;i<slvNums;i++)
		        {
		            if(slvID[i]!=(Emb_StartID_Wheel+i))        //!< �����ָ�������id�ţ����·�
                    {
                        continue;
                    }
			        sPos = dat[i][8] + (dat[i][9]<<8) + (dat[i][10]<<16) + (dat[i][11]<<24);
			        sSpd = dat[i][12] + (dat[i][13]<<8);                //!< ���Ӽ��ٱ�Ϊ1���ٶ�Ϊ����100����RPM

                    sSpd = sSpd/MOTOR_SPEED_COEFFICIENT;

                    EmbRecordCmdSendToMotor(i,Ctrl_Mode_Posit,sSpd,sPos,EWAYBOT_WHEEL);    //!< ��¼�������·���ָ��

                    tmpDat[3] = EwayMotorReg_TargSpd_L;
                    tmpDat[4] = 8;
		
			        tmpDat[5] = (u8)sSpd;
			        tmpDat[6] = (u8)(sSpd>>8);
                    tmpDat[7] = 0x00;
			        tmpDat[8] = 0x00;			
			        tmpDat[9] = (u8)sPos;
			        tmpDat[10] = (u8)(sPos>>8);
			        tmpDat[11] = (u8)(sPos>>16);
			        tmpDat[12] = (u8)(sPos>>24);

                    res = g_GeneralMotorSingleWrite((Emb_StartID_Wheel+i),13,tmpDat,EWAYBOT_WHEEL);
                    
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            SysLogWrite(LogMajor,"AvoidRs:2,Wheel(%d) change mode to Pos and Send Pos(0x56-8)to Mov failed.rt:0x%x.",i,res);
                            
                            res = Bsp_printf("AvoidRs:2,Wheel(%d) change mode to Pos and Send Pos(0x56-8)to Mov failed.rt:0x%x.",i,res);
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
                        &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                        {                                                        
                            res = Bsp_printf("AvoidRs:2,Wheel(%d) change mode to Pos and Send Pos(0x56-8)to Mov.Spd:0x%x,Pos:0x%x",i,sSpd,sPos);
                        }
                    }
		        }
            }
            else*/
            {
                //!< ָ��,ʱ���,
		        //!< fCode	fTimeStamp ID  Pos	Speed
		        //!<   2		4		2	4	  2 
		        //!< ��δ�����ֵ����Ч�ԣ�����
												//!< ��Ϊ������λ�ú��ٶ�ֵȡ�෴��
		        for(i=0;i<slvNums;i++)
		        {
		            if(slvID[i]!=(Emb_StartID_Wheel+i))        //!< �����ָ�������id�ţ����·�
                    {
                        continue;
                    }
		            
			        sPos = dat[i][8] + (dat[i][9]<<8) + (dat[i][10]<<16) + (dat[i][11]<<24);
			        sSpd = dat[i][12] + (dat[i][13]<<8);                //!< ���Ӽ��ٱ�Ϊ1 �ٶ�Ϊ����100����RPM

                    sSpd = sSpd/MOTOR_SPEED_COEFFICIENT;

                    EmbRecordCmdSendToMotor(i,Ctrl_Mode_Posit,sSpd,sPos,EWAYBOT_WHEEL);    //!< ��¼�������·���ָ��
		
			        tmpDat[i*8] = (u8)sSpd;
			        tmpDat[(i*8)+1] = (u8)(sSpd>>8);
			
			        tmpDat[(i*8)+4] = (u8)sPos;
			        tmpDat[(i*8)+5] = (u8)(sPos>>8);
			        tmpDat[(i*8)+6] = (u8)(sPos>>16);
			        tmpDat[(i*8)+7] = (u8)(sPos>>24);
		        }

		        res = sysGeneralMotorBroadcastWrite(slvID[0],slvNums,EwayMotorReg_TargSpd_L,8,tmpDat,EWAYBOT_WHEEL);	
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
			            SysLogWrite(LogMajor,"AvoidRs:2,Wheels Send Pos(0x56-8)to Move failed.rt:0x%x.",res);
			
			            res = Bsp_printf("AvoidRs:2,Wheels Send Pos(0x56-8)to Move failed.rt:0x%x.",res);
                    }
		        }

                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
                &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                {
		            if(slvNums==1)
		            {
			            res = Bsp_printf("AvoidRs:2,Wheels Send Pos(0x56-8)to Mov.Pos:0x%x,Spd:0x%x.",sPos,sSpd);
		            }
		            else
		            {
                        res = Bsp_printf("AvoidRs:2,Wheels Send Pos(0x56-8)to Mov.Pos1:0x%x,Spd1:0x%x,Pos2:0x%x,Spd2:0x%x.",(tmpDat[4]+(tmpDat[5]<<8)+(tmpDat[6]<<16)+(tmpDat[7]<<24)),(tmpDat[0]+(tmpDat[1]<<8)),sPos,sSpd);
                    }
                }	
            }		
	    }
	    else if(fCode==PCCmd_WheelSpeedMove)
	    {
	        /*if(modeDat[0] != 0)
            {
                memcpy(tmpDat,modeDat,3);              
                
                for(i=0;i<slvNums;i++)
                {
		            if(slvID[i]!=(Emb_StartID_Wheel+i))        //!< �����ָ�������id�ţ����·�
                    {
                        continue;
                    }

                    sSpd = dat[i][8] + (dat[i][9]<<8);              //!< ���Ӽ��ٱ�Ϊ1 �ٶ�Ϊ����100����RPM

                    sSpd = sSpd/MOTOR_SPEED_COEFFICIENT;      

                    EmbRecordCmdSendToMotor(i,Ctrl_Mode_Speed,sSpd,0,EWAYBOT_WHEEL);    //!< ��¼�������·���ָ��
                    
                    tmpDat[3] = EwayMotorReg_TargSpd_L;
                    tmpDat[4] = 2; 
                    tmpDat[5] = (u8)sSpd;
                    tmpDat[6] = (u8)(sSpd>>8);                    

                    res = g_GeneralMotorSingleWrite((Emb_StartID_Wheel+i),7,tmpDat,EWAYBOT_WHEEL);
                    
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            SysLogWrite(LogMajor,"AvoidRs:2,Wheel(%d) change mode to Spd and Send SpdCmd(0x56-2)to Mov failed.rt:0x%x.",i,res);
                            
                            res = Bsp_printf("AvoidRs:2,Wheel(%d) change mode to Spd and Send SpdCmd(0x56-2)to Mov failed.rt:0x%x.",i,res);
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
                        &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                        {                                                        
                            res = Bsp_printf("AvoidRs:2,Wheel(%d) change mode to Spd and Send SpdCmd(0x56-2)to Mov,Spd:0x%x.",i,sSpd);
                        }
                    }
                }

            }
            else*/
            {
                for(i=0;i<slvNums;i++)
                {
		            if(slvID[i]!=(Emb_StartID_Wheel+i))        //!< �����ָ�������id�ţ����·�
                    {
                        continue;
                    }

                    sSpd = dat[i][8] + (dat[i][9]<<8);      //!< ���Ӽ��ٱ�Ϊ1�ٶ�Ϊ����100����RPM
                    sSpd = sSpd/MOTOR_SPEED_COEFFICIENT;

                    EmbRecordCmdSendToMotor(i,Ctrl_Mode_Speed,sSpd,0,EWAYBOT_WHEEL);    //!< ��¼�������·���ָ��
                            
                    tmpDat[i*2] = (u8)sSpd;
                    tmpDat[(i*2)+1] = (u8)(sSpd>>8);
                }
                        
                res = sysGeneralMotorBroadcastWrite(slvID[0],slvNums,EwayMotorReg_TargSpd_L,2,tmpDat,EWAYBOT_WHEEL);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        SysLogWrite(LogMajor,"AvoidRs:2,Wheels Send Spd Cmd(0x56-2) failed.rt:0x%x.",res);
                            
                        res = Bsp_printf("AvoidRs:2,Wheels Send Spd Cmd(0x56-2) failed.rt:0x%x.",res);
                    }
                }
                else
                {                
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
                    &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                    {
                        if(slvNums==1)
                        {
                            res = Bsp_printf("AvoidRs:2,Wheels Send Spd Cmd(0x56-2).Spd:0x%x(0x%x)",sSpd,(dat[0][8]+(dat[0][9]<<8)));
                        }
                        else
                        {
                            res = Bsp_printf("AvoidRs:2,Wheels Send Spd Cmd(0x56-2).Spd0:0x%x(Pc-0x%x),Spd1:0x%x(Pc-0x%x)",(tmpDat[0]+(tmpDat[1]<<8)),(dat[0][8]+(dat[0][9]<<8)),(tmpDat[2]+(tmpDat[3]<<8)),(dat[1][8]+(dat[1][9]<<8)));
                        }
                    }
                }
            }		
	    }

        return ERR_NONE;

    }
    else if(rslt==3)        //!< ת��һ������Σ���������ʱ�����鴦��
    {
        //!< �ݲ�����
        return ERR_NONE;
    }	
    
    return ERR_NONE;
}


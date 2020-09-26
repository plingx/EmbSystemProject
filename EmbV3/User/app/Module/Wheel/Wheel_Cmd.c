/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Wheel_Cmd.c
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
当无PC指令给轮子时，当前避障的处理

(1) 检查当前是否有避障标志or避障功能是否打开
        无，则返回
        有，则继续(2)

(2) 根据速度记录判断当前轮子运动情况:
        向前 or 向后 or 静止 or 原地低速转小圈 

(3) 根据当前避障标志位及当前轮子运动情况，进行处理

        若当前轮子是向前运动的，且有前方4个避障标识被置起，则发送停止指令给轮子
        若当前轮子时向后运动的，且有后方2个避障标识被置起，则发送停止指令给轮子
        若当前轮子是近似静止的，则不发送任何指令

(4) 退出


*/
/* --------------------------------------------------------------------------*/
/**
* @name g_WheelMovementObstacleAvoidProcess
* @brief 
* @details 当目前无PC下发的指令给轮子时，当前避障的处理
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
    

    if((pAvoidModule->ObstAvoid.unAvoidFlag==0)||(pAvoidModule->ObstAvoid.Swi == 0))      //!< 当前避障功能未打开or当前无避障标识被置起，无需避障，返回
    {
        return ERR_NONE;
    }

    //!< 更新两轮运动趋势分析
    res = g_WheelMovementStatusAnalysis(&Result,&Sum,&(pAvoidModule->WheelMov.LeftSpd[0]),&(pAvoidModule->WheelMov.RightSpd[0]));

    if(res == ERR_NONE)
    {
        pAvoidModule->ObstAvoid.WheelMove.Result = Result;
        pAvoidModule->ObstAvoid.WheelMove.Sum = Sum;
        pAvoidModule->ObstAvoid.WheelMove.ResUpdat = 0x01;

        if(Result==WHEELS_MOVEMENTSTATUS_MOVE)
        {
            if(((Sum >= 0)&&((pAvoidModule->ObstAvoid.unAvoidFlag&0x0F) != 0))||((Sum < 0)&&((pAvoidModule->ObstAvoid.unAvoidFlag&0x30) != 0)))           //!< 前进，检查前面4个雷达
            {                
                action = 1;             //!< action stop
                
                //!< 更新避障使能标识
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

                //!< 更新避障使能标识
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

                //!< 更新避障使能标识                                                    
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
默认轮子应用模式一直为车轮模式:
      绝对位置相对位置可根据需要设置；
      控制模式的速度位置模式可设置；

轮子下发指令处理流程:

(1) 检查与上位机通信状态，
		若为通信断开状态，则下发刹车指令，并返回ERR_NONE
		若为通信连接状态，则继续(2)

(2) 检查指令缓存是否有指令
		若无指令，则返回
		若有指令，则继续(3)

(3) 取指令，并判断指令类型
		若只取出1条轮子指令，则继续(4)
		若两个轮子均有指令，则判断2个轮子的指令类型是否一致
			若两个轮子指令不同，则扔掉这两条指令，并返回处理错误
			若两个轮子指令相同，则继续(4)

(4) 判断当前轮子指令模式是否与刚取出的指令模式是否相同
		若指令模式与当前轮子运行模式不同，则下发更改轮子运行模式指令(位置or速度，相对or绝对)，并更改当前系统存储的轮子运行模式
		若指令模式与当前轮子运行模式相同，则继续(5)

(5) 若指令为速度指令，下发速度指令
    若指令为位置指令，则下发位置指令

(6) 返回ERR_NONE    

*/


/*
左轮，速度模式下，speed>0 前进的方向
右轮，速度模式下，speed<0 前进的方向

左轮，位置模式下，PosX > 0 ,且Pos1      > Pos0      前进的方向
右轮，位置模式下，PosX < 0 ,且abs(Pos1) > abs(Pos0) 前进的方向

*/


/* --------------------------------------------------------------------------*/
/**
* @name g_WheelMovementCmdAnalysis
* @brief 
* @details 根据上位机的轮子指令类型及轮子当前左右轮的状态，分析出下发何种指令给轮子

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
轮子运动指令分析流程

(1) 判断当前避障功能是否打开并且在6个雷达中有检查到在规定范围内的障碍物
        若避障功能未打开or在雷达监测范围内无障碍物，则*pRslt = 2(继续照常下发任何下位机的指令)返回
        否，继续(2)

(2) 根据指令类型和左右轮的速度位置，判断当前是在前进or后退or静止
        若前方雷达检测到在规定范围内有障碍物，且待发送指令与当前轮子的位置or速度比较判断为前进，则*pRslt = 1(发送停止指令)返回
        若后方雷达检测到在规定范围内有障碍物，且待发送指令与当前轮子的位置or速度比较判断为后退，则*pRslt = 1(发送停止指令)返回
        其他，则*pRslt = 2(继续照常下发任何下位机的指令)


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
    
    //!< 判断当前避障状态标志
    if((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag == 0)||(EwayRadarObstAvoid.ObstAvoid.Swi==0))       //若无需考虑避障，指令照常下发
    {
        *pRslt = 2;       //!< 返回2表示继续照常下发指令

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-1,res2,return:AvoidSwi:%x,AvoidFlag:0x%x.",EwayRadarObstAvoid.ObstAvoid.Swi,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
                
        return ERR_NONE;
    }

    //!< 判断当前的指令是前进还是后退
    if(fCode == PCCmd_WheelPositionMove)
    {    
        pos[0] = pLd[8] + (pLd[9]<<8) + (pLd[10]<<16) + (pLd[11]<<24);      //!< 待发送指令的Pos

        pos[1] = pRd[8] + (pRd[9]<<8) + (pRd[10]<<16) + (pRd[11]<<24);

        pd0 = &EwayWheels.mState.pFromMotor->mRegs[0] ;                      //!< 当前轮子的Pos

        deltPos[0] = pos[0] - (pd0[15] + (pd0[16]<<8) + (pd0[17]<<16) + (pd0[18]<<24));
        
        pd1 = &((EwayWheels.mState.pFromMotor+1)->mRegs[0]);

        deltPos[1] = pos[1] - (pd1[15] + (pd1[16]<<8) + (pd1[17]<<16) + (pd1[18]<<24));

        if(((deltPos[0] > 0) && (deltPos[1] < 0))||((deltPos[0] == 0)&&(deltPos[1] < 0))||((deltPos[0] > 0)&&(deltPos[1] == 0)))
        {
            rslt = 2;               //!< 前进
        }
        else if(((deltPos[0] < 0) && (deltPos[1] > 0))||((deltPos[0] == 0) && (deltPos[1] > 0))||((deltPos[0] < 0) && (deltPos[1] == 0)))
        {
            rslt = 1;               //!< 后退
        }
        else
        {
            rslt = 3;               //!< 其他情况
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

        if(((spd[0] < 0)&&(spd[1] > 0))||((spd[0] == 0)&&(spd[1] > 0))||((spd[0] < 0)&&(spd[1] == 0)))                      //指令为后退
        {
            rslt = 1;               //!< 后退
        }
        else if(((spd[0] > 0)&&(spd[1] < 0))||((spd[0] == 0)&&(spd[1] < 0))||((spd[0] > 0)&&(spd[1] == 0)))                 //指令为前进
        {
            rslt = 2;               //!< 前进
        }
        else 
        {
            rslt = 3;               //!< 其他情况
        }
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMovAnaly-2.1 CmdSpeed:0x%x-0x%x",spd[0],spd[1]);//!< rslt是当前指令会导致的运动趋势
        }
    }

    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
    {
        Bsp_printf("WhlMovAnaly-2.2,0x%04x,rslt:%d(1-goback,2-goforward,3-others).",fCode,rslt);//!< rslt是当前指令会导致的运动趋势
    }
    
    if(((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x0F) != 0)&&(rslt==2))       //前方4个避障标识有打开，且为前进指令
    {
        *pRslt = 1;             //!< 发送停止指令

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-1(Stop),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }
    else if(((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x30) != 0)&&(rslt==1))//!< 后方标识有打开，且为后退指令
    {
        *pRslt = 1;             //!< 发送停止指令
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-1(Stop),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }
    else 
    {
        *pRslt = 2;             //!< 照常发送
        
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
    
    //!< 判断当前避障状态标志
    if((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag == 0)||(EwayRadarObstAvoid.ObstAvoid.Swi==0))       //若无需考虑避障，指令照常下发
    {
        *pRslt = 2;       //!< 返回2表示继续照常下发指令

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-1,res2(SendCmd),return:AvoidSwi:%x,AvoidFlag:0x%x.",EwayRadarObstAvoid.ObstAvoid.Swi,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
                
        return ERR_NONE;
    }

    //!< 判断当前的指令是前进还是后退
    if(fCode == PCCmd_WheelPositionMove)
    {    
        pos[0] = pLd[8] + (pLd[9]<<8) + (pLd[10]<<16) + (pLd[11]<<24);      //!< 待发送指令的Pos

        pos[1] = pRd[8] + (pRd[9]<<8) + (pRd[10]<<16) + (pRd[11]<<24);

        pd0 = &EwayWheels.mState.pFromMotor->mRegs[0] ;                      //!< 当前轮子的Pos

        deltPos[0] = pos[0] - (pd0[15] + (pd0[16]<<8) + (pd0[17]<<16) + (pd0[18]<<24));
        
        pd1 = &((EwayWheels.mState.pFromMotor+1)->mRegs[0]);

        deltPos[1] = pos[1] - (pd1[15] + (pd1[16]<<8) + (pd1[17]<<16) + (pd1[18]<<24));

        if(((deltPos[0] > 0) && (deltPos[1] < 0))||((deltPos[0] == 0)&&(deltPos[1] < 0))||((deltPos[0] > 0)&&(deltPos[1] == 0)))
        {
            rslt = 2;               //!< 前进
        }
        else if(((deltPos[0] < 0) && (deltPos[1] > 0))||((deltPos[0] == 0) && (deltPos[1] > 0))||((deltPos[0] < 0) && (deltPos[1] == 0)))
        {
            rslt = 1;               //!< 后退
        }
        else if((deltPos[0] > 0) && (deltPos[1] > 0))
        {
            if(deltPos[0]>deltPos[1])
            {
                rslt = 2;    //!< 向右前
            }
            else if(deltPos[0]<deltPos[1])
            {
                rslt = 1;    //!< 向左后
            }
            else
            {
                rslt = 3;    //!< 原地转，顺时针
            }
        }
        else if((deltPos[0] < 0) && (deltPos[1] < 0))
        {
            if(abs(deltPos[0]) > abs(deltPos[1]))
            {
                rslt = 1;    //!< 向右后
            }
            else if(abs(deltPos[0]) < abs(deltPos[1]))
            {
                rslt = 2;    //!< 向左前
            }
            else
            {
                rslt = 3;    //!< 原地转，逆时针
            }

        }
        else if((deltPos[0] == 0) && (deltPos[1] == 0))
        {
            rslt = 4;       //!< 保持当前位置
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

        if(((spd[0] < 0)&&(spd[1] > 0))||((spd[0] == 0)&&(spd[1] > 0))||((spd[0] < 0)&&(spd[1] == 0)))                      //指令为后退
        {
            rslt = 1;               //!< 后退
        }
        else if(((spd[0] > 0)&&(spd[1] < 0))||((spd[0] == 0)&&(spd[1] < 0))||((spd[0] > 0)&&(spd[1] == 0)))                 //指令为前进
        {
            rslt = 2;               //!< 前进
        }
        else if((spd[0] > 0)&&(spd[1] > 0))
        {
            if(spd[0] > spd[1])
            {
                rslt = 2;    //!< 向右前
            }
            else if(spd[0] < spd[1])
            {
                rslt = 1;    //!< 向左后
            }
            else
            {
                rslt = 3;    //!< 原地转，顺时针
            }
        }
        else if((spd[0] < 0)&&(spd[1] < 0))
        {
            if(abs(spd[0]) > abs(spd[1]))
            {
                rslt = 1;    //!< 向右后
            }
            else if(abs(spd[0]) < abs(spd[1]))
            {
                rslt = 2;    //!< 向左前
            }
            else
            {
                rslt = 3;    //!< 原地转，逆时针
            }
        }
        else if((spd[0] == 0)&&(spd[1] == 0))       //!< 保持当前位置
        {
            rslt = 4;
        }
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMovAnaly-2.1 CmdSpeed:0x%x-0x%x",spd[0],spd[1]);//!< rslt是当前指令会导致的运动趋势
        }
    }

    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
    {
        Bsp_printf("WhlMovAnaly-2.2,0x%04x,CmdRslt:%d(1-goback,2-goforward,3-turn,4-stop,0-NoInit).",fCode,rslt);//!< rslt是当前指令会导致的运动趋势
    }
    
    if(((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x0F) != 0)&&(rslt==2))       //前方4个避障标识有打开，且为前进指令
    {
        *pRslt = 1;             //!< 发送停止指令

        //!< 更新避障使能标识                   
        EwayRadarObstAvoid.ObstAvoid.AvoidEnableFlag = EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x0F;

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-1(Stop),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }
    else if(((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x30) != 0)&&(rslt==1))//!< 后方标识有打开，且为后退指令
    {
        *pRslt = 1;             //!< 发送停止指令

        //!< 更新避障使能标识                   
        EwayRadarObstAvoid.ObstAvoid.AvoidEnableFlag = EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x30;
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-1(Stop),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }
    else if(((EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x3F) != 0)&&(rslt==3))      //!< 原地转圈，检查6个Radar
    {
        *pRslt = 1;             //!< 发送停止指令
        
        //!< 更新避障使能标识                   
        EwayRadarObstAvoid.ObstAvoid.AvoidEnableFlag = EwayRadarObstAvoid.ObstAvoid.unAvoidFlag&0x3F;
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-1(Stop),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }
    else
    {
        *pRslt = 2;             //!< 照常发送
        
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
        (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
            Bsp_printf("WhlMoveAnalysis-3,fCode:0x%04x,Return-2(Move),AvoidF:0x%04x.",fCode,EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
        }
    }

    return ERR_NONE;    

}


/*      20180509 添加避障功能，修改代码逻辑      ling
默认轮子应用模式一直为车轮模式:
      绝对位置相对位置可根据需要设置；
      控制模式的速度位置模式可设置；

轮子下发指令处理流程:

(1) 检查与上位机通信状态，
		若为通信断开状态，则下发刹车指令，并返回ERR_NONE
		若为通信连接状态，则继续(2)

(2) 检查指令缓存是否有指令
		若无指令，则返回
		若有指令，则继续(3)

(3) 取指令，并判断指令类型
		若只取出1条轮子指令，则继续(4)
		若两个轮子均有指令，则判断2个轮子的指令类型是否一致
			若两个轮子指令码不同，则扔掉这两条指令，并返回处理错误
			若两个轮子指令码相同，则继续(4)

(4) 判断当前轮子指令模式是否与刚取出的指令模式是否相同
		若指令模式与当前轮子运行模式不同，则下发更改轮子运行模式指令(位置or速度，相对or绝对)，并更改当前系统存储的轮子运行模式
		若指令模式与当前轮子运行模式相同，则继续(5)

(5) 根据指令类型、指令内容、避障状态，分析出如下2种:
        直接下发当前指令，继续(7)
        下发停止指令，继续(6)

(6) 若为下发停止指令
        位置模式，组停止指令，下发，返回退出
        速度模式，组停止指令，下发，返回退出

(7) 若直接下发当前指令
        位置模式，组包，下发，返回退出
        速度模式，组包，下发，返回退出
        
(8) 返回ERR_NONE    
*/

/*      20180525 因添加Clear指令后是否位置保持，及将模式切换指令整合到速度位置指令中，修改      ling
默认轮子应用模式一直为车轮模式:
      绝对位置相对位置可根据需要设置；
      控制模式的速度位置模式可设置；

轮子下发指令处理流程:

(1) 检查与上位机通信状态，
		若为通信断开状态，则下发刹车指令，并返回ERR_NONE
		若为通信连接状态，则继续(2)

(2) 检查指令缓存是否有指令
		若无指令，则返回
		若有指令，则继续(3)

(3) 取指令，并判断指令类型
		若只取出1条轮子指令，则继续(4)
		若两个轮子均有指令，则判断2个轮子的指令类型是否一致
			若两个轮子指令码不同，则扔掉这两条指令，并返回处理错误
			若两个轮子指令码相同，则继续(4)

(4) 判断当前轮子指令模式是否与刚取出的指令模式是否相同
		若指令模式与当前轮子运行模式不同，则下发更改轮子运行模式指令(位置or速度，相对or绝对)，并更改当前系统存储的轮子运行模式
		若指令模式与当前轮子运行模式相同，则继续(5)

(5) 根据指令类型、指令内容、避障状态，分析出如下2种:
        直接下发当前指令，继续(7)
        下发停止指令，继续(6)

(6) 若为下发停止指令
        位置模式，组停止指令，下发，返回退出
        速度模式，组停止指令，下发，返回退出

(7) 若直接下发当前指令
        位置模式，组包，下发，返回退出
        速度模式，组包，下发，返回退出
        
(8) 返回ERR_NONE    
*/
/*
20180710添加对轮子电机0xC0寄存器的查询，修改处理流程


(1) 检查与上位机通信状态，
		若为通信断开状态，则下发自由停止(0x51-2)指令，并返回ERR_NONE
		若为通信连接状态，则继续(2)

(2) 检查指令缓存是否有指令
		若无指令，则继续(3)
		若有指令，则继续(4)

(3) 检查当前是否有Clear指令待处理
        若没有，则返回ERR_NONE;
        若有，则检查当前车轮2个电机的ReadyReg是否已经准备好；
                若还没有准备好，则清除Clear处理标志，并返回。
                若已经准备好，则检查Clear指令是否需要位置保持
        
                                    需要位置保持，当前为位置模式则发送当前位置及默认速度给各个轮子电机，并清除Clear标志，返回ERR_None
                                                当前为速度模式则发送模式转换为位置的指令，并发送当前位置给各个轮子电机，并清除Clear标志，返回ERR_None
                        
                                    不需要位置保持，什么都不用做，清除Clear标志，返回ERR_None

(4) 取指令，并判断指令类型
		若只取出1条轮子指令，则继续(5)
		若两个轮子均有指令，则判断2个轮子的指令类型是否一致
			若两个轮子指令不同，则扔掉这两条指令，并返回处理错误
			若两个轮子指令相同，则继续(5)

(5) 判断当前轮子指令模式是否与刚取出的指令模式是否相同
		若指令模式与当前轮子运行模式不同，则组更改模式数据包，并记录，继续(6)
		若指令模式与当前轮子运行模式相同，则继续(6)

(6) 调用避障开关的函数，并分析当前是否要停止、继续照常下发指令
            若为发送停止指令，则发送停止指令
            若照常发送指令，则发送
                    若指令为速度指令，并检查是否有模式转换指令，后下发速度指令(保证模式转换指令与速度指令在同一包中)
                    若指令为位置指令，并检查是否有模式转换指令，后下发位置指令(保证模式转换指令与速度指令在同一包中)

(7) 返回ERR_NONE    


*/

s8 sysModuleWheelsMovement(EwayWheelModule* pModule)
{ 	
	u8 tmpDat[20]={0};
    u8 modeDat[4]={0};
	u16 fCode;
	u8 dat[Motor_Num_Wheel][CMD_PC_2_EMB_BUFF_LEN]={0};//!< 用于存放从缓存读出的18byte数据.
	u8 i,slvNums=0;
	s8 res;
    u8 rslt;
	u8 slvID[2]={0};
	s16 sSpd;
	s32 sPos;
    u8* pd;
    u8 tmp,tmp1;
	MotorCMDBuff* pmotorCmd = pModule->mControl.pToMotorCmd;
	
	//!< 检查与上位机通信状态，若为断开，则向电机下发速度为0的指令or刹车的指令or位置指令，总之就是要停下来。
	if((EwayEmbSys.Comm.wPc & 0x01) == EMB_COMM_STATUS_DISCONNECTED)
	{	
		//!< 网络未连接，检查轮子是否为自由停止状态
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

	if(i >= pModule->mCount)                    //!< 指令缓存中并无待发指令.
	{
	    //!< 检查当前是否有Clear指令待处理
        if((pModule->mControl.posKeep&0xFF00)!=0)     //!< 收到过Clear指令
        {
//   20180711
#if CheckWheelMotorReadyBit_Def
            if(pModule->mRdyStatus!=0x03)        //!< 检查轮子电机是否已经准备好，若没有准备好，则清Clear处理标志后，直接返回，不发送任何数据给轮子
            {
                pModule->mControl.posKeep = 0x0000; 
                
                return ERR_NONE;
            }
#endif
            tmp = (u8)pModule->mControl.posKeep;
                    
            if(tmp == 'K')       //!< 需要位置保持
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
                        memcpy(&tmpDat[(i<<3)+4],(u8*)(&sysEwServo[Emb_StartID_Wheel+14+i].uPosWheel),4);   //!< 取Position的4个字节下发下去
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
                    //!< 切换模式为位置                    
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

                    //!< 更改系统暂存中的模式
                    EwayWheels.mControl.mCtrlMode = Ctrl_Mode_Posit;                    

                    //!< 下发当前位置
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
            else if(tmp == 'N')  //!< 不需要位置保持
            {/*     20180525 讨论的若不需要位置保持，则无需任何操作         ling
            
                //!< 无论位置还是速度模式，均向0x51寄存器下发自由停止指令
                tmpDat[0] = 2;                  //!< reg0x51:0-减速停止,1-紧急制动,2-自由停止
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

        //!< 在无指令时，检查当前是否需要避障
        if(EwayRadarObstAvoid.ObstAvoid.Swi != 0)
        {
            g_WheelMovementObstacleAvoidProcess(pModule,&EwayRadarObstAvoid);
        }

		return ERR_NONE;    
	}
	
	slvNums = 0;
	for(i=0;i<(pModule->mCount);i++)
	{
		//!< 根据id号查询相应指令缓存是否有待发指令
		if(((pmotorCmd+i)->dCmdCnt) > 0 )
		{
			//res = sysGeneralGetEmbToMotorCmd((pmotorCmd+i),&dat[slvNums][0]);	  //!< 取出指令
			res = g_GetPCCmdEmbToMotorCmd(pmotorCmd,i,&dat[slvNums][0]);

            //14Bytes//  2By    4By  2By    4By    2By            位置模式
            //10Bytes//  2By    4By  2By    0By    2By            速度模式
            //!< 轮子指令缓存16个14bytes:Code0-1 T0-3 iD0-1 Pos0-3 Speed0-1
            
			if(res==ERR_NONE)
			{
				slvID[slvNums] = dat[slvNums][6];						      //!< dat[6]&dat[7] 存放的是ID号，取低字节
					
				slvNums += 1;
			}				
		}
	}

	if(slvNums>=2)
	{
		if(dat[0][0]!=dat[1][0])     //!< 比较两条指令的fCode是否相同，若不同，则直接返回
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

    if(pModule->mRdyStatus!=0x03)        //!< 检查轮子电机是否已经准备好，若没有准备好，则清Clear处理标志后，直接返回，不发送任何数据给轮子
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

	//!< 若当前轮子控制模式与指令的模式不相同，则需要发送控制模式转换指令
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
            //!< 更改系统暂存中的模式
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

    res = g_WheelMovementCmdAnalysis(&rslt,fCode,&dat[0][0],&dat[1][0]);        //!< 根据指令类型、指令内容、避障状态，分析

    if(res != ERR_NONE)
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            SysLogWrite(LogNormal,"g_WheelMovementCmdAnalysis() err,rt:%d.",res);

            Bsp_printf("g_WheelMovementCmdAnalysis() err,rt:%d.",res);
        }

        return res;
    }

	//!< 下发指令
	memset(tmpDat,0,20);

    if(rslt==1)             //!< 发送停止指令 ，位置模式下发位置为当前位置速度为默认值，速度模式下发速度为0
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
    else if(rslt==2)        //!< 照常发送指令
    {
        if(fCode==PCCmd_WheelPositionMove)
	    {
	        /*if(modeDat[0] != 0)
            {
                memcpy(tmpDat,modeDat,3);

                for(i=0;i<slvNums;i++)
		        {
		            if(slvID[i]!=(Emb_StartID_Wheel+i))        //!< 检查有指令的轮子id号，并下发
                    {
                        continue;
                    }
			        sPos = dat[i][8] + (dat[i][9]<<8) + (dat[i][10]<<16) + (dat[i][11]<<24);
			        sSpd = dat[i][12] + (dat[i][13]<<8);                //!< 轮子减速比为1，速度为扩大100倍的RPM

                    sSpd = sSpd/MOTOR_SPEED_COEFFICIENT;

                    EmbRecordCmdSendToMotor(i,Ctrl_Mode_Posit,sSpd,sPos,EWAYBOT_WHEEL);    //!< 记录本周期下发的指令

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
                //!< 指令,时间戳,
		        //!< fCode	fTimeStamp ID  Pos	Speed
		        //!<   2		4		2	4	  2 
		        //!< 并未检查数值的有效性，待续
												//!< 若为右轮则位置和速度值取相反数
		        for(i=0;i<slvNums;i++)
		        {
		            if(slvID[i]!=(Emb_StartID_Wheel+i))        //!< 检查有指令的轮子id号，并下发
                    {
                        continue;
                    }
		            
			        sPos = dat[i][8] + (dat[i][9]<<8) + (dat[i][10]<<16) + (dat[i][11]<<24);
			        sSpd = dat[i][12] + (dat[i][13]<<8);                //!< 轮子减速比为1 速度为扩大100倍的RPM

                    sSpd = sSpd/MOTOR_SPEED_COEFFICIENT;

                    EmbRecordCmdSendToMotor(i,Ctrl_Mode_Posit,sSpd,sPos,EWAYBOT_WHEEL);    //!< 记录本周期下发的指令
		
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
		            if(slvID[i]!=(Emb_StartID_Wheel+i))        //!< 检查有指令的轮子id号，并下发
                    {
                        continue;
                    }

                    sSpd = dat[i][8] + (dat[i][9]<<8);              //!< 轮子减速比为1 速度为扩大100倍的RPM

                    sSpd = sSpd/MOTOR_SPEED_COEFFICIENT;      

                    EmbRecordCmdSendToMotor(i,Ctrl_Mode_Speed,sSpd,0,EWAYBOT_WHEEL);    //!< 记录本周期下发的指令
                    
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
		            if(slvID[i]!=(Emb_StartID_Wheel+i))        //!< 检查有指令的轮子id号，并下发
                    {
                        continue;
                    }

                    sSpd = dat[i][8] + (dat[i][9]<<8);      //!< 轮子减速比为1速度为扩大100倍的RPM
                    sSpd = sSpd/MOTOR_SPEED_COEFFICIENT;

                    EmbRecordCmdSendToMotor(i,Ctrl_Mode_Speed,sSpd,0,EWAYBOT_WHEEL);    //!< 记录本周期下发的指令
                            
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
    else if(rslt==3)        //!< 转弯一类的情形，留到测试时再酌情处理
    {
        //!< 暂不处理
        return ERR_NONE;
    }	
    
    return ERR_NONE;
}


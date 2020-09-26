/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Arms_Cmd.c
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author 
* @version 0.0.1
* @date 2018-01-30
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/01/30 | 0.0.1 | Ling     | Create file
*
*/

#include "includes.h"

extern EwayArmModule EwayArms;
extern EwayEmbSysModule EwayEmbSys;
extern EwayEmbSysDebugModule* pDebug;
extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];




/*
手臂电机下发指令流程        (目前主要先实现位置指令的下发)
(1) 判断当前下位机与PC通信状态是否正常，
		若通信不正常则下发刹车指令，下发指令前要考虑当前电机的控制模式是位置or速度再下发(默认为位置模式)
		若通信正常则继续(2)

(2) 获取当前指令缓存个数，
		若Cmd Cnt==0，则返回ERR_NONE.
		若Cmd Cnt>0,则继续(3)

(3) 取指令与当前电机控制模式比较(速度or位置)
		若不同，则组包发送电机控制模式切换指令，并存入待发送队列。继续(4)
		若相同，则继续(4)

(4) 组包发送给各个电机的指令，并存入待发送队列
	轮询6个指令缓存:
	a.若有指令则取出相应数据(id,position,speed),并根据电机特性计算Posi&Speed放入相应待写入指令缓存
	b.无指令则继续		

(5) 返回。


手臂运动处理流程(20180524因添加执行Clear指令后后是否位置保持的功能，又再次修改):
(1) 检查与上位机通信是否正常
        正常，则继续(2)
        不正常，则向2个头部电机下发紧急制动的指令(0x51-1)

(2) 检查指令buff中是否有指令
        若有指令，则继续(4)
        若无指令，则继续(3)

(3) 检查当前是否有Clear指令待处理
        若有，则判断是否需要位置保持
                    若需要位置保持，向电机发送位置为当前位置，速度为0的位置指令，清除Clear指令标志，退出，返回ERR_NONE;
                    若不需要位置保持，向电机0x51寄存器写自由停止的指令，清除Clear指令标志，退出，返回ERR_NONE;
                    
        若没有，则退出返回ERR_NONE;

(4) 当前有指令，则检查指令类型
        若为位置指令，则下发，退出，返回ERR_NONE;
        其他，忽略，退出，返回ERR_NONE;


*/
/*
检查手臂下发指令数据是否正确
1. PC下发给左手臂的指令，debug下查看发送前，数据是否正确。
	(1) 下发6条指令给左臂----ok
	(2) 下发4条指令给左臂----ok

2. PC下发给左臂的指令，EMB下发给电机是否正确，指令是否有被电机正确执?
	(1)  下发电机数据正确----ok
	(2)  电机执行正确

3.剩余待测试项目:
	(1)  明确电机速度位置的极值

*/

s8 sysModuleArmsMovement(EwayArmModule* pModule,EwayMotor_Device_Type devType)
{
	u8 tmpDat[30]={0};						//!< 初始化时为0
	u8 dat[CMD_PC_2_EMB_BUFF_LEN]={0};
	u8 sndDat[30]={0};						//!< motorsNums*regNums 6*4  24byte
	u8 i,slvNums;
	u8 slvID[6]={0};
	s8 res;
	u8 startID;
	s16 mSpeed;
    u16 mPos;
	u16 mCnt=0;
    u16 ClrmaskBit;
    u8 tmp=0;
    s32 tPos;
	MotorCMDBuff* pArmCmd;
    Motor_LastPosCmd* pArmLstPos;
	MotorReducRatio* pArmRa = pModule->mControl.pMotorRatio;

	if(devType == EWAYBOT_ARMS_LEFT)
	{
		startID = Emb_StartID_ArmL;
		pArmCmd = pModule->mControl.pToMotorCmd;
		pArmRa = pModule->mControl.pMotorRatio;
		mCnt = EwayArms.mCntLeft;
        ClrmaskBit = 0x0F00;
        pArmLstPos = EwayArms.mControl.pmlstPos;
	}
	else if(devType == EWAYBOT_ARMS_RIGHT)
	{
		startID = Emb_StartID_ArmR;
		pArmCmd = (pModule->mControl.pToMotorCmd) + Motor_Num_ArmLeft;
		pArmRa = (pModule->mControl.pMotorRatio) + Motor_Num_ArmLeft;
		mCnt = EwayArms.mCntRight;
        ClrmaskBit = 0xF000;
        pArmLstPos = EwayArms.mControl.pmlstPos + Motor_Num_ArmLeft;
	}
	else
	{
		return ERR_INPUT_PARAMETERS;
	}
		
	//!< 检查与上位机通信状态，若为断开，则向电机下发速度为0的指令or刹车的指令or位置指令，总之就是要停下来。
	if((EwayEmbSys.Comm.wPc & 0x01) == EMB_COMM_STATUS_DISCONNECTED)
	{/*
	    for(i=0;i<mCnt;i++)
        {
            tmpDat[i] = 1;                  //!< reg0x51:0-减速停止,1-紧急制动,2-自由停止
        }   
        
        res = sysGeneralMotorBroadcastWrite(startID,mCnt,EwayMotor_Stop,1,tmpDat,devType);
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("2.1 ArmMov-0 PC Comm disconnected,send(0x51-1) failed.");
            }
        }
        else
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
            {
                //Bsp_printf("2.1 ArmMov-0 PC Comm disconnected,send(0x51-1) Succ!");
            }
        }*/
        //!< 检查当前是否有avail参数为使能的电机并记录
        slvNums = 0;
	
		for(i=0;i<mCnt;i++)
		{
			//!< 根据id号检查相应缓存是否有指令
			if(((pArmLstPos+i)->avail) ==  LastPosAvailablePosOn)
			{						
			    slvID[slvNums] = dat[6];						//!< dat[6]&dat[7] 存放的是ID号，取低字节
					
				sndDat[(slvNums<<2)] = (u8)MOTOR_DEFAULT_SPEED_MIN;	 //!< slvNums标识当前要下发指令的舵机个数 * 4个寄存器作为偏移量
				sndDat[(slvNums<<2)+1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);

                if(EmbGetLastPosCmdSendtoMotor(pArmLstPos,i,&tPos)==ERR_NONE)
                {
                    mPos = (u16)tPos;
                    sndDat[(slvNums<<2)+2] = (u8)mPos;
                    sndDat[(slvNums<<2)+3] = (u8)(mPos>>8);         
                    
                    slvNums += 1;
                }
			}			
		}

        if((pModule->mControl.mCtrlMode == Ctrl_Mode_Posit)&&(slvNums>0))
        {
            res = sysGeneralMotorDiscontinuousBroadcastWrite(slvID,slvNums,EwayMotorReg_TargSpd_L,4,sndDat,devType);
            
            if(ERR_NONE != res)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                   res = Bsp_printf("2.1 Arm(%d)Mov-0 PC Comm disconnected,send CurPos failed.",(u8)devType);
                }
            }
            else
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                {
                    res = Bsp_printf("2.1 Arm(%d)Mov-0 PC Comm disconnected,send CurPos(0x56-4) Suc.",(u8)devType);
                }                               
            } 
        }
      
		return ERR_NONE;
	}
	
	//!< 获取当前缓存中的指令个数，若全是为0，则表明没有待下发的指令，返回ERR_NONE。	
	for(i=0;i<mCnt;i++)
	{		
		if(((pArmCmd+i)->dCmdCnt) > 0 ) break;
	}
	
	if(i >= mCnt)
	{
        //!< 检查当前是否有Clear指令待处理
        if((EwayArms.mControl.posKeep&ClrmaskBit)!=0)     //!< 收到过Clear指令
        {
            tmp = (u8)EwayArms.mControl.posKeep;
                    
            if(tmp == 'K')       //!< 需要位置保持
            {
                //!< pos mode,send Current Pos and speed= minimal value
                for(i=0;i<mCnt;i++)
                {
                    tmpDat[(i<<2)] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                    tmpDat[(i<<2)+1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
                    tmpDat[(i<<2)+2] = sysEwServo[startID-1+i].uPosJointL;//!< 取当前的Position下发下去    
                    tmpDat[(i<<2)+3] = sysEwServo[startID-1+i].uPosJointH;                    
                }

                res = sysGeneralMotorBroadcastWrite(startID,mCnt,EwayMotorReg_TargSpd_L,4,tmpDat,devType);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                       res = Bsp_printf("2.1 Arm(%d) Keep Pos Cmd, Posi mode,send CurrPos(0x56-4) failed.",(u8)devType);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                    {
                        res = Bsp_printf("2.1 Arm(%d) Keep Pos Cmd, Posi mode,send CurrPos(0x56-4) Suc.",(u8)devType);
                    }                               
                }                          
            }
            else if(tmp == 'N')  //!< 不需要位置保持   do nothing
            {                /*         
                //!< 无论位置还是速度模式，均向0x51寄存器下发自由停止指令                
                for(i=0;i<mCnt;i++)
                {
                    tmpDat[i] = 2;              //!< reg0x51:0-减速停止,1-紧急制动,2-自由停止
                }
                
                res = sysGeneralMotorBroadcastWrite(startID,mCnt,EwayMotor_Stop,1,tmpDat,devType);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("2.1 Arm(%d) Not Keep Pos Cmd,BrodcstWr(0x51-2) failed.",(u8)devType);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                    {
                        res = Bsp_printf("2.1 Arm(%d) Not Keep Pos Cmd,BrodcstWr(0x51-2) Suc!",(u8)devType);
                    }
                }                */
            }
        
            EwayArms.mControl.posKeep &= (~ClrmaskBit); 
        
            return ERR_NONE;
        }
        else
        {
		    return ERR_NONE;
        }    
	}
	
	//!< 取指令 检查指令的范围:速度限制，位置限制
	if(EwayArms.mControl.mCtrlMode==Ctrl_Mode_Posit)	  //!< 默认现在指令缓存中的指令均为位置指令 
	{
		slvNums = 0;
	
		for(i=0;i<mCnt;i++)
		{
			//!< 根据id号检查相应缓存是否有指令
			if(((pArmCmd+i)->dCmdCnt) > 0 )
			{
				res = sysGeneralGetEmbToMotorCmd((pArmCmd+i),dat);	  //!< 取出指令
	
				if(res==ERR_NONE)
				{				
					//!< 指令,时间戳,
					//!< fCode	fTimeStamp ID  Pos	Speed
					//!<   2		4		2	4	  2 
					//!<   2		4		2	2	  2         modified 20180508
						
					slvID[slvNums] = dat[6];						//!< dat[6]&dat[7] 存放的是ID号，取低字节
						
					mSpeed = dat[10] + (dat[11]<<8);
					mPos = dat[8] + (dat[9]<<8);
						
					//!< 检查速度是否超限[(+/-)15,(+/-)MAX],检查					
					sysEmbCheckSendToJointMotorParameters(&mPos,&mSpeed);

					//!< 根据id号取出减速比并计算下发给motor的速度值
					mSpeed = mSpeed*(*(pArmRa+i))/MOTOR_SPEED_COEFFICIENT;
	
					sndDat[(slvNums<<2)] = (u8)mSpeed;	 //!< slvNums标识当前要下发指令的舵机个数 * 4个寄存器作为偏移量
					sndDat[(slvNums<<2)+1] = (u8)(mSpeed>>8);
	
					//!< 位置直接填写码盘值
					sndDat[(slvNums<<2)+2] = (u8)mPos;
					sndDat[(slvNums<<2)+3] = (u8)(mPos>>8);			
	
					slvNums += 1;

                    EmbRecordingLastPosCmdSendtoMotor(pArmLstPos,i,(s32)mPos);

                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&
                    (pDebug->secondFun&SecFunDebugCtrl_EmbIn_MotorLstCmd_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK))
                    {
                        Bsp_printf("Record armMotor(%d) lastPosCmd,Pos:0x%x",(i+1),mPos);
                    }
				}				
			}			
		}
	
		//!< 组包发送到相应发送队列 	
		if(slvNums>0)
		{
			res = sysGeneralMotorDiscontinuousBroadcastWrite(slvID,slvNums,EwayMotorReg_TargSpd_L,4,sndDat,devType);
			if(ERR_NONE != res)
			{
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
				    res = Bsp_printf("2.1 Arm(%d) Send PosCmd to Motor BrodcstWr() failed.",(u8)devType);
                }
			}
			else
			{
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                {
                    if(((devType==EWAYBOT_ARMS_LEFT)&&(pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK))||((devType==EWAYBOT_ARMS_RIGHT)&&(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                    {
                        res = Bsp_printf("Arm(%d)SndPosCmdToM,Num:%d.P1:0x%x,S1:0x%x.P2:0x%x,S2:0x%x.P3:0x%x,S3:0x%x.P4:0x%x,S4:0x%x.P5:0x%x,S5:0x%x.P6:0x%x,S6:0x%x.",\
                              (u8)devType,slvNums,(sndDat[2]+(sndDat[3]<<8)),(sndDat[0]+(sndDat[1]<<8)),(sndDat[6]+(sndDat[7]<<8)),(sndDat[4]+(sndDat[5]<<8)),\
                              (sndDat[10]+(sndDat[11]<<8)),(sndDat[8]+(sndDat[9]<<8)),(sndDat[14]+(sndDat[15]<<8)),(sndDat[12]+(sndDat[13]<<8)),\
                              (sndDat[18]+(sndDat[19]<<8)),(sndDat[16]+(sndDat[17]<<8)),(sndDat[22]+(sndDat[23]<<8)),(sndDat[20]+(sndDat[21]<<8)));
                    }
                    
                }
		    }
        }
	
		return ERR_NONE;
	}
	
	return ERR_NONE;
}




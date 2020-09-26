/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Head_Cmd.c
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

extern EwayEmbSysModule EwayEmbSys;
extern EwayHeadModule EwayHeads;
extern EwayEmbSysDebugModule* pDebug;
extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];




s8 sysModuleHeadsMovement(EwayHeadModule* pModule)
{
	u8 tmpDat[12]={0};						//!< 初始化时为0
	u8 dat[CMD_PC_2_EMB_BUFF_LEN]={0};
	u8 sndDat[30]={0};						//!< motorsNums*regNums 6*4  24byte
	u8 i,slvNums;
	u8 slvID[6];
	s8 res;
    u8 tmp;
	s16 mSpeed;
    s32 tPos;
    u16 mPos;
	MotorCMDBuff* pHdCmd = pModule->mControl.pToMotorCmd;
	MotorReducRatio* pHdRa = pModule->mControl.pMotorRatio;
		
	//!< 检查与上位机通信状态，若为断开，则向电机下发速度为0的指令or刹车的指令or位置指令，总之就是要停下来。
	if((EwayEmbSys.Comm.wPc & 0x01) == EMB_COMM_STATUS_DISCONNECTED)
	{
	    //!< 检查当前是否有avail参数为使能的电机并记录
        slvNums = 0;
	
		for(i=0;i<Motor_Num_Head;i++)
		{
			//!< 根据id号检查相应缓存是否有指令
			if(((EwayHeads.mControl.pmlstPos+i)->avail) ==  LastPosAvailablePosOn)
			{						
			    slvID[slvNums] = Emb_StartID_Head + i;
					
				sndDat[(slvNums<<2)] = (u8)MOTOR_DEFAULT_SPEED_MIN;	 //!< slvNums标识当前要下发指令的舵机个数 * 4个寄存器作为偏移量
				sndDat[(slvNums<<2)+1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);

                if(EmbGetLastPosCmdSendtoMotor(EwayHeads.mControl.pmlstPos,i,&tPos)==ERR_NONE)
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
            res = sysGeneralMotorDiscontinuousBroadcastWrite(slvID,slvNums,EwayMotorReg_TargSpd_L,4,sndDat,EWAYBOT_HEAD);
            
            if(ERR_NONE != res)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    res = Bsp_printf("2.1 hdMov-0 PC Comm disconnected,send CurrPos(0x56-4) failed.");
                }
            }
            else
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
                {
                    res = Bsp_printf("2.1 hdMov-0 PC Comm disconnected,send CurrPos(0x56-4) Suc.");
                }                               
            }  
        }

		return ERR_NONE;
	}
	
	//!< 获取当前缓存中的指令个数，若全是为0，则表明没有待下发的指令，返回ERR_NONE。
	for(i=0;i<pModule->mCount;i++)
	{		
		if(((pHdCmd+i)->dCmdCnt) > 0 ) break;
	}
	
	if(i >= pModule->mCount)       //!< 当前指令buffer中已经无指令
	{
        //!< 检查当前是否有Clear指令待处理
        if((pModule->mControl.posKeep&0xFF00)!=0)     //!< 收到过Clear指令
        {
            tmp = (u8)pModule->mControl.posKeep;
                    
            if(tmp == 'K')       //!< 需要位置保持
            {
                //!< pos mode,send Current Pos and speed=0 
                tmpDat[0] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                tmpDat[1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
                tmpDat[2] = sysEwServo[Emb_StartID_Head-1].uPosJointL;//!< 取当前的Position下发下去    
                tmpDat[3] = sysEwServo[Emb_StartID_Head-1].uPosJointH;

                tmpDat[4] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                tmpDat[5] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
                tmpDat[6] = sysEwServo[Emb_StartID_Head].uPosJointL;//!< 取当前的Position下发下去    
                tmpDat[7] = sysEwServo[Emb_StartID_Head].uPosJointH;

                res = sysGeneralMotorBroadcastWrite(Emb_StartID_Head,Motor_Num_Head,EwayMotorReg_TargSpd_L,4,tmpDat,EWAYBOT_HEAD);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                       res = Bsp_printf("2.1 Hd Keep Pos Cmd, Posi mode,send CurrPos(0x56-4) spd=0 failed.");
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
                    {
                        res = Bsp_printf("2.1 Hd Keep Pos Cmd, Posi mode,send CurrPos(0x56-4) spd=0 Suc.");
                    }                               
                }                          
            }
            else if(tmp == 'N')  //!< 不需要位置保持        do nothing
            {                /*
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
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
                    {
                        res = Bsp_printf("2.1 Hd Not Keep Pos Cmd,BrodcstWr(0x51-2) Suc!");
                    }
                }    */            
            }
        
            pModule->mControl.posKeep = 0x0000; 
        
            return ERR_NONE;
        }
        else
        {
		    return ERR_NONE;
        }    
	}

	//!< 取指令 检查指令的范围:速度不能小于15,不能大于???，位置为磁编码器的值
	if(pModule->mControl.mCtrlMode==Ctrl_Mode_Posit)	  //!< 默认现在指令缓存中的指令均为位置指令 
	{
		slvNums = 0;
	
		for(i=0;i<pModule->mCount;i++)
		{
			//!< 根据id号检查相应缓存是否有指令
			if(((pHdCmd+i)->dCmdCnt) > 0 )
			{
				//res = sysGeneralGetEmbToMotorCmd((pHdCmd+i),dat);	  //!< 取出指令
				res = g_GetPCCmdEmbToMotorCmd(pHdCmd,i,dat);
	
				if(res==ERR_NONE)
				{				
					//!< 指令,时间戳,
					//!< fCode	fTimeStamp ID  Pos	Speed
					//!<   2		4		2	4	  2 
					//!<   2		4		2	2	  2             //!< modify 20180508
						
					slvID[slvNums] = dat[6];						//!< dat[6]&dat[7] 存放的是ID号，取低字节
						
					mSpeed = dat[10] + (dat[11]<<8);                //!< mSpeed = dat[12] + (dat[13]<<8);

                    mPos = dat[8] + (dat[9]<<8);
						
					//!< 检查速度是否超限[(+/-)15,(+/-)MAX],检查
					sysEmbCheckSendToJointMotorParameters(&mPos,&mSpeed);

					//!< 根据id号取出减速比并计算下发给motor的速度值
					mSpeed = mSpeed*(*(pHdRa+i))/MOTOR_SPEED_COEFFICIENT;
	
					sndDat[(slvNums<<2)] = (u8)mSpeed;	 //!< slvNums标识当前要下发指令的舵机个数 * 4个寄存器作为偏移量
					sndDat[(slvNums<<2)+1] = (u8)(mSpeed>>8);
	
					//!< 位置填写码盘值2字节
					sndDat[(slvNums<<2)+2] = (u8)mPos;
					sndDat[(slvNums<<2)+3] = (u8)(mPos>>8);			
	
					slvNums += 1;
                    
                    EmbRecordingLastPosCmdSendtoMotor(EwayHeads.mControl.pmlstPos,i,(s32)mPos);

                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&
                    (pDebug->secondFun&SecFunDebugCtrl_EmbIn_MotorLstCmd_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
                    {
                        Bsp_printf("Record HdMotor(%d) lastPosCmd,Pos:0x%x",(i+1),mPos);
                    }
				}				
			}			
		}
	
		//!< 组包发送到相应发送队列 	
		if(slvNums>0)
		{
			res = sysGeneralMotorDiscontinuousBroadcastWrite(slvID,slvNums,EwayMotorReg_TargSpd_L,4,sndDat,EWAYBOT_HEAD);
			if(ERR_NONE != res)
			{
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    SysLogWrite(LogMajor,"2.1 Head Send to Motor BrodcstWrCmd(0x56-4) failed.rt:%d.",res);

                    res = Bsp_printf("2.1 Head Send to Motor BrodcstWrCmd(0x56-4) failed.rt:%d.",res);
                }
			}
			else
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
                {
                    res = Bsp_printf("Head SndToMCmd(0x56-4).Id:%d,Num:%d.P1:0x%x,S1:0x%x.P2:0x%x,S2:0x%x.",\
                        slvID[0],slvNums,(sndDat[2]+(sndDat[3]<<8)),(sndDat[0]+(sndDat[1]<<8)),(sndDat[6]+(sndDat[7]<<8)),(sndDat[4]+(sndDat[5]<<8)));
                }
			}
		}
	
		return ERR_NONE;
	}
	
	return ERR_NONE;
}


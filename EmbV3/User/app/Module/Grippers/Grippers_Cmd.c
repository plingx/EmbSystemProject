/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Grippers_Cmd.c
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
#include "Emb_Gripper.h"


extern EwayEmbSysModule EwayEmbSys;
extern EwayEmbSysDebugModule* pDebug;
extern DYNAMIXEL_SERVO_STATUS sysDyServo[SYS_MAX_DYNAMIXEL_SERVO_NUMS];



/*
(1) 检查与上位机通信:
		若通信不正常则，下发停止运动指令
		若通信正常，则继续(2)

(2) 检查每个电机的指令缓存是否有待发送指令:
		若均无指令,则返回ERR_NONE;
		若有指令，则继续(3)

(3) 判断当前爪子的控制模式是否为位置速度模式:
		若不是,则不处理
		若是，则继续(4)

(4) 组包下发指令
		检查数据是否超限(待下发的位置及速度数值是否超出限定范围)
	


返回ERR_NONE;

*/
s8 sysModuleGrippersMovement(EwayGripperModule* pModule)
{
	u8 dat[CMD_PC_2_EMB_BUFF_LEN]={0};
	u8 i;
	s8 res;
	u8 slvID[2]={0};
	u8 sndDat[10]={0};
	u8 slvNums;
	u16 mSpeed,mPos;
	//float mTmp;
	MotorCMDBuff* pGpCmd = pModule->mControl.pToMotorCmd;
	
	if((EwayEmbSys.Comm.wPc & 0x01) == EMB_COMM_STATUS_DISCONNECTED)
	{
		//!< 当网络断开时，向爪子发送停止指令，即向爪子位置写为当前位置，速度为0
		/*
		for(i=0;i<Motor_Num_Gripper;i++)
        {
            sndDat[(i<<2)] = sysDyServo[i].unCurPosL;
            sndDat[(i<<2)+1] = sysDyServo[i].unCurPosH;
            sndDat[(i<<2)+2] = 0;
            sndDat[(i<<2)+3] = 0;
            
            slvID[i] = Emb_StartID_Gripper + i;
        }
        
        slvNums = Motor_Num_Gripper;
        
		res = sysSendDynamixelBroadcastSyncWritePacket(EWAYBOT_GRIPPERS,DYNAMIXEL_REG_GoalPosL,4,slvID[0],slvNums,sndDat);
	
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("2.1 GripMov-0 PC Comm disconnected,send CurPos,spd=0 failed.");
            }
        }
        else
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
            {
                //Bsp_printf("2.1 GripMov-0 PC Comm disconnected,sendCurPos,spd=0 Succ!");
            }
        }*/
        
		return ERR_NONE;
	}
	
	for(i=0;i<(pModule->mCount);i++)
	{		
		if(((pGpCmd+i)->dCmdCnt) > 0 ) break;
	}
	
	if(i >= pModule->mCount)
	{
		return ERR_NONE;
	}

	if(pModule->mControl.mCtrlMode != Ctrl_Mode_Posit)
	{
		//!< 目前只处理位置模式的指令，后续再添加爪子的转矩开关和转矩限制功能
		return ERR_NONE;
	}

	slvNums = 0;
	for(i=0;i<(pModule->mCount);i++)
	{
		//!< 根据id号查询相应指令缓存是否有待发指令
		if(((pGpCmd+i)->dCmdCnt) > 0 )
		{
			res = sysGeneralGetEmbToMotorCmd((pGpCmd+i),dat);	  //!< 取出指令
	
			if(res==ERR_NONE)
			{				
				//!< 指令,时间戳,
				//!< fCode	fTimeStamp ID  Pos	Speed
				//!<   2		4		2	2	  2 
						
				slvID[slvNums] = dat[6];						      //!< dat[6]&dat[7] 存放的是ID号，取低字节

				mPos = dat[8] + (dat[9]<<8);
				//!< 检查位置是否超限   0-4096
				if(mPos>DYNAMIXEL_POSITION_MAX) 
				{
					continue;                                        //!< 位置超限不下发此指令
				}

				//!< 写位置				
				sndDat[slvNums<<2] = (u8)mPos;                      //!< slvNums标识当前要下发指令的舵机个数 * 4个寄存器作为偏移量
				sndDat[(slvNums<<2)+1] = (u8)(mPos>>8);
				
				mSpeed = dat[10] + (dat[11]<<8);					//!< RPM转换为爪子的速度寄存器数值  RPM = 0.114*RegisterValue
				//mSpeed = (u16)((float)mSpeed/0.114);
				//Tmp = ((float)(mSpeed))/0.114;
				//mSpeed = (u16)mTmp;
				mSpeed = mSpeed/11.4;            //!< 上位机下发的速度是扩大100倍的rpm,因此还要除以100,即(0.114*100)   20180508 modify ling
				
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Gripper_MASK))
                {
                    Bsp_printf("Snd to Gripper(%d) Pos:0x%x,Spd:0x%x.",dat[6],(dat[8] + (dat[9]<<8)),(dat[10] + (dat[11]<<8)));
                }

				if((mSpeed==0)||(mSpeed>DYNAMIXEL_SPEED_JOINT_MAX))	//!< 检查速度是否超限
				{
					mSpeed = DYNAMIXEL_SPEED_DEFAULT;
				}
				
				//!< 写速度
				sndDat[(slvNums<<2)+2] = (u8)mSpeed;
				sndDat[(slvNums<<2)+3] = (u8)(mSpeed>>8);	
					
				slvNums += 1;
			}				
		}
	}

	if(slvNums>0)
	{
		res = sysSendDynamixelBroadcastSyncWritePacket(EWAYBOT_GRIPPERS,DYNAMIXEL_REG_GoalPosL,4,slvID[0],slvNums,sndDat);
		if(ERR_NONE != res)
		{
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
			    SysLogWrite(LogMajor,"GripperSndSyncWrite() failed.rt:0x%x.",res);

			    res = Bsp_printf("GripperSndSyncWrite() failed.rt:0x%x.",res);
            }
		}
		else
		{
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Gripper_MASK))
            {
                if(slvNums==1)
                {                
                    res = Bsp_printf("GripperSndSyncWrite,id0:0x%x,Pos:0x%x,Spd:0x%x.",slvID[0],(sndDat[0]+(sndDat[1]<<8)),(sndDat[2]+(sndDat[3]<<8)));
                }
                else if(slvNums==2)
                {                
                    res = Bsp_printf("GripperSndSyncWrite,id0:0x%x,Pos:0x%x,Spd:0x%x,id1:0x%x,Pos:0x%x,Spd:0x%x.",slvID[0],(sndDat[0]+(sndDat[1]<<8)),(sndDat[2]+(sndDat[3]<<8)),slvID[1],(sndDat[4]+(sndDat[5]<<8)),(sndDat[6]+(sndDat[7]<<8)));
                }             
            }
		}
	}

	return ERR_NONE;
}


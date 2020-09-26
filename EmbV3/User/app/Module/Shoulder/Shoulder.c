/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Shoulder.c
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author 
* @version 0.0.1
* @date 2017-12-20
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/01/18 | 0.0.1 | xx       | Create file
*
*/

#include "includes.h"

extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];
extern QueueHandle_t PCToShoulderQueHdl;
extern QueueHandle_t ShoulderToMotorQueHdl;

extern EwayEmbSysModule EwayEmbSys;
extern EwayArmModule EwayArms;
extern EwayHeadModule EwayHeads;
extern EwayGripperModule EwayGrippers; 
extern EwayWheelModule EwayWheels;
extern u8 MotorCfgReg[SYS_MAX_GENERAL_SERVO_NUMS][GENERAL_SERVO_CFG_REGS];
extern osMutexId PCCmdBuffOpHandleShd;
extern EwayEmbSysDebugModule* pDebug;





MotorSTATEBuff ShoulderReg={0};
MotorRecordBuff ShoulderRecord={0};
MotorCMDBuff ShoulderCmd={{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleShd};
MotorReducRatio ShoulderReduRat;
Motor_LastPosCmd ShoulderLstPos={0};
u8 ShoulderPID[EMB_READ_WHEELMODE_POS_PID_REG_NUM]={0};

EwayShoulderModule EwayShoulder={
	{&ShoulderCmd,&ShoulderReduRat,Ctrl_Mode_Speed,App_Mode_Wheel,Pos_ClosedLoop_Absolute,&ShoulderLstPos,0},
	{0,{0xFF,0xFF},{0x00000000,0x00000000},{0x0000,0x0000},{0,0,0}},
	{&ShoulderReg,&ShoulderRecord,&MotorCfgReg[(Emb_StartID_Shoulder-1)][0]},
    {{0},{0}},   
	Motor_Num_Shoulder,
	0x0000,
	ShoulderPID
};


s8 BspGetLimitSwitchStatus(SYSLIMITSWITCH sw,u8* pSta);
s8 sysGetMotorSendQueue(EwayMotor_Device_Type devType,QueueHandle_t* pQue);
s8 sysGeneralGetEmbToMotorCmd(MotorCMDBuff* pmotorCmd,u8* pDat);





s8 sysGetLimitSwitchStatus(EwayShoulderModule* pModule)
{
	u8 tmp;
	s8 res;
	
	//< check up and down limit status
	res = BspGetLimitSwitchStatus(DownLim,&tmp);
	if(res==ERR_NONE)
	{
		pModule->shLimit.pinState[0] = (EwayShoulder.shLimit.pinState[0]<<1)+tmp;
	}

	res = BspGetLimitSwitchStatus(UpLim,&tmp);
	if(res==ERR_NONE)
	{
		pModule->shLimit.pinState[1] = (EwayShoulder.shLimit.pinState[1]<<1)+tmp;
	}	
	
	return ERR_NONE;
}



s8 sysEmbSetShoulderGeneralMotorPosPID(u8* pdat)
{
    s8 res = ERR_NONE;
    u8 dat[10]={0};    

    if(pdat == NULL)
    {
        return ERR_INPUT_PARAMETERS;
    }

    memcpy(dat,pdat,6);
    
    res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotor_PosCtrl_KP_L,EMB_READ_WHEELMODE_POS_PID_REG_NUM,dat,EWAYBOT_SHOULDER);

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
s8 sysPcToShoulderCmdProcess(u16 Code,u32 timStamp,u16 dlen,u8* pdat)
{
	u8 i;
	s8 res=ERR_NONE;
	u8 Unit=0;    //!< 根据指令类型确定单元命令长度:位置指令12byte,速度指令12byte
	u16 iD;
	u8 dat[20]={0};
	
	//!< 检查指令码	
	switch(Code)
	{
		case PCCmd_ShoulderPosMoveMode:
						Unit = PCCmd_ShoulderPosMoveMode_Unit;             //!< id(2bytes) + position(4bytes) + speed(2bytes)
						
						if(dlen<Unit) 
                        {                  
							res = ERR_INPUT_PARAMETERS;
                        }			
			break;
		case PCCmd_ShoulderSpdMoveMode:
						Unit = PCCmd_ShoulderSpdMoveMode_Unit;
						
						if(dlen<Unit)
                        {
							res = ERR_INPUT_PARAMETERS;
                        }			
			break;
		case PCCmd_ShoulderPIDMode:
                        Unit = PCCmd_ShoulderPosPIDSet_Unit;
						
						if(dlen<Unit)
                        {
							res = ERR_INPUT_PARAMETERS;
                        }
			break;
        /*
		case PCCmd_ShoulderTorqueSwitch:
			break;
		case PCCmd_ShoulderMotorLimit:
			break;
		*/
		default:
			res = ERR_INPUT_PARAMETERS;
			break;
	}	

	if(res != ERR_NONE)
	{
		return res;
	}

    if(Code == PCCmd_ShoulderPIDMode)
    {
        iD = pdat[0] + (pdat[1]<<8);
        
        if(iD != Emb_StartID_Shoulder)
        {
            return ERR_INPUT_PARAMETERS;
        }
        
#if Emb_Set_MotorPID
        res = sysEmbSetShoulderGeneralMotorPosPID(pdat+2);
        
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("EmbSetShoulderMotorPosPID() failed.");
            }

            return res;
        }
        else
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
            {
                res = Bsp_printf("EmbSetShoulderMotorPosPID succ.P:0x%04x,I:%04x,D:%04x.",(pdat[2]+(pdat[3]<<8)),(pdat[4]+(pdat[5]<<8)),(pdat[6]+(pdat[7]<<8)));
            }
        }
        
#endif
        return ERR_NONE;
	}
	
	//!< 检查id号
	dat[0] = (u8)Code;
	dat[1] = (u8)(Code>>8);
	dat[2] = (u8)timStamp;
	dat[3] = (u8)(timStamp>>8);
	dat[4] = (u8)(timStamp>>16);
	dat[5] = (u8)(timStamp>>24);

	i=0;
	
	while(i<dlen)
	{
		iD = pdat[i] + (pdat[i+1]<<8);
		
		if(iD != Emb_StartID_Shoulder)          //!< 检查肩膀ID
		{
			i += Unit;
		}
		else
		{
			memcpy(&dat[6],(pdat+i),Unit);   
			                           //  2By    4By  2By    4By    2By
			//!< 肩部指令缓存16个14bytes:Code0-1 T0-3 iD0-1 Pos0-3 Speed0-1
			//!< 肩部指令缓存16个16bytes:Code0-1 T0-3 iD0-1 Pos0-3 Speed0-3      (20170717)     
			res = g_StorePcCmdtoCmdBuff(EwayShoulder.mControl.pToMotorCmd,0,dat,(Unit+6));

			if(res != ERR_NONE)
			{
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
				    SysLogWrite(LogMajor,"sh g_StorePcCmdtoCmdBuff() failed,rt:%d",res);

				    //Bsp_printf("sh g_StorePcCmdtoCmdBuff() failed,rt:%d",res);
                }
			}
		
			i += Unit;
		}
	}
	
	return ERR_NONE;
}



/* --------------------------------------------------------------------------*/
/**
* @name sysGeneralGetEmbToMotorCmd
* @brief 
* @details Get one valid command from command buffer
* 
*                Code(2)+TimeStamp(4)+ID(2)+Posi(8)+Speed(2)
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
/*
s8 sysGeneralGetEmbToMotorCmd(NodeCtrlModule* pNode,u8* pDat)
{
	MotorCMDBuff* pCmd;

	if(pNode->pToMotorCmd->dCmdCnt<=0)
	{
		return ERR_INPUT_PARAMETERS;
	}

	pCmd = pNode->pToMotorCmd;

	memcpy(pDat,&(pNode->pToMotorCmd->dCmd[pNode->pToMotorCmd->dCmdRe][0]),CMD_PC_2_EMB_BUFF_LEN);

	pNode->pToMotorCmd->dCmdCnt--;//!< dCmdCnt会在相同的任务中处理，因此无需做临界段代码防护，无需关闭打开中断
								 //!< 若后续需要修改代码将此功能移到其他任务中，则需注意要做临界段代码防护.

	pNode->pToMotorCmd->dCmdRe++;

	if((pNode->pToMotorCmd->dCmdRe)>=CMD_PC_2_EMB_BUFF_NUMS) pNode->pToMotorCmd->dCmdRe = 0;

	return ERR_NONE;
}
*/


s8 sysGeneralGetEmbToMotorCmd(MotorCMDBuff* pmotorCmd,u8* pDat)
{
	if((pmotorCmd->dCmdCnt)<=0)
	{
		return ERR_INPUT_PARAMETERS;
	}

	memcpy(pDat,&(pmotorCmd->dCmd[pmotorCmd->dCmdRe][0]),CMD_PC_2_EMB_BUFF_LEN);

	pmotorCmd->dCmdCnt--;//!< dCmdCnt会在相同的任务中处理，因此无需做临界段代码防护，无需关闭打开中断
								 //!< 若后续需要修改代码将此功能移到其他任务中，则需注意要做临界段代码防护.

	pmotorCmd->dCmdRe++;

	if((pmotorCmd->dCmdRe)>=CMD_PC_2_EMB_BUFF_NUMS) pmotorCmd->dCmdRe = 0;

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
void sysEmbToMotorCmdProcess(EwayMotor_Device_Type devType)
{
	s8 res = ERR_NONE;
	
	switch(devType)
	{
		case EWAYBOT_SHOULDER:				
				res = sysModuleShoulderMovement(&EwayShoulder);
				if(ERR_NONE != res)
				{
					//SysLogWrite(LogMajor,"3-3 sysModuleShoulderMovement() failed.rt:0x%x.",res);

					//Bsp_printf("3-2-3-1 sysModuleShoulderMovement() failed.rt:0x%x",res);
				}

			break;

		case EWAYBOT_ARMS_LEFT:				
				res = sysModuleArmsMovement(&EwayArms,EWAYBOT_ARMS_LEFT);
				if(ERR_NONE != res)
				{
					//SysLogWrite(LogMajor,"3-3 sysModuleArmsMovement(EWAYBOT_ARMS_LEFT) failed.rt:0x%x.",res);

					//res = Bsp_printf("3-3 sysModuleArmsMovement(EWAYBOT_ARMS_LEFT) failed.\n");
				}
			break;

		case EWAYBOT_ARMS_RIGHT:				
				res = sysModuleArmsMovement(&EwayArms,EWAYBOT_ARMS_RIGHT);
				if(ERR_NONE != res)
				{
					//SysLogWrite(LogMajor,"3-3 sysModuleArmsMovement(EWAYBOT_ARMS_RIGHT) failed.rt:0x%x.",res);

					//res = Bsp_printf("3-3 sysModuleArmsMovement(EWAYBOT_ARMS_RIGHT) failed.\n");
				}				
			break;

		case EWAYBOT_HEAD:				
				res = sysModuleHeadsMovement(&EwayHeads);
				if(ERR_NONE != res)
				{
					//SysLogWrite(LogMajor,"3-3 sysModuleHeadsMovement() failed.rt:0x%x.",res);

					//res = Bsp_printf("3-3 sysModuleHeadsMovement() failed.\n");
				}				
			break;

		case EWAYBOT_GRIPPERS:				
				res = sysModuleGrippersMovement(&EwayGrippers);
				if(ERR_NONE != res)
				{
					//SysLogWrite(LogMajor,"3-3 sysModuleGrippersMovement() failed.rt:0x%x.",res);

					//res = Bsp_printf("3-3 sysModuleGrippersMovement() failed.\n");
				}				
			break;

		case EWAYBOT_WHEEL:				
				res = sysModuleWheelsMovement(&EwayWheels);
				if(ERR_NONE != res)
				{
					//SysLogWrite(LogMajor,"3-3 sysModuleWheelsMovement() failed.rt:0x%x.",res);

					//res = Bsp_printf("3-3 sysModuleWheelsMovement() failed.\n");
				}
			break;
			
		default:
			//res = ERR_INPUT_PARAMETERS;
			break;
	}
	
	//return  res;
}


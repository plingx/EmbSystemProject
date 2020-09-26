/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp.c
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2018-01-26
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/01/26 | 0.0.1 |          | Create file
*
*/
#include "includes.h"



extern u8 MotorCfgReg[SYS_MAX_GENERAL_SERVO_NUMS][GENERAL_SERVO_CFG_REGS];
extern osMutexId PCCmdBuffOpHandleAml[Motor_Num_ArmLeft];
extern osMutexId PCCmdBuffOpHandleAmr[Motor_Num_ArmRight];
extern EwayEmbSysDebugModule* pDebug;



MotorSTATEBuff ArmsReg[Motor_Num_Arm]={0};
MotorRecordBuff ArmsRecord[Motor_Num_Arm]={0};
MotorCMDBuff ArmsCmd[Motor_Num_Arm]={
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAml[0]},
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAml[1]},
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAml[2]},
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAml[3]},
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAml[4]},
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAml[5]},
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAmr[0]},
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAmr[1]},
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAmr[2]},
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAmr[3]},
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAmr[4]},
                                     {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleAmr[5]}


};
/*
MotorReducRatio ArmsReduRat[Motor_Num_Arm]={
	MOTOR_REDUCTION_RATIO_1,
	MOTOR_REDUCTION_RATIO_2,
	MOTOR_REDUCTION_RATIO_3,
	MOTOR_REDUCTION_RATIO_4,
	MOTOR_REDUCTION_RATIO_5,
	MOTOR_REDUCTION_RATIO_6,
	MOTOR_REDUCTION_RATIO_7,
	MOTOR_REDUCTION_RATIO_8,
	MOTOR_REDUCTION_RATIO_9,
	MOTOR_REDUCTION_RATIO_10,
	MOTOR_REDUCTION_RATIO_11,
	MOTOR_REDUCTION_RATIO_12
};
*/
MotorReducRatio ArmsReduRat[Motor_Num_Arm]={0};
Motor_LastPosCmd ArmsLstPos[Motor_Num_Arm]={0};


EwayArmModule EwayArms={
	{&ArmsCmd[0],&ArmsReduRat[0],Ctrl_Mode_Posit,App_Mode_Joint,Pos_ClosedLoop_Absolute,&ArmsLstPos[0],0},
	{&ArmsReg[0],&ArmsRecord[0],&MotorCfgReg[(Emb_StartID_ArmL-1)][0]},
	6,
	6,
	0x0000
};


/* --------------------------------------------------------------------------*/
/**
* @name l_StorePcCmdtoArmsCmdBuff
* @brief 
* @details Code(2)+TimeStamp(4)+ID(2)+Posi(4)+Speed(2)
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 l_StorePcCmdtoArmsCmdBuff(u16 id,u8* pdat,u8 len)    
{	
	MotorCMDBuff* pCmd;
	u8* pDest; 
	u8 piD;     //<
	//s8 res = ERR_NONE;

	piD = (u8)(id-Emb_ID_Offset_ARM);              //!< 找到正确的buffer编号

	pCmd = EwayArms.mControl.pToMotorCmd + piD;    //!< 根据舵机ID取其指令缓存地址
	
	if(len>CMD_PC_2_EMB_BUFF_LEN)                         //!< code(2) + timeStamp(4) + dataUnit 
		return ERR_INPUT_PARAMETERS;

	if(pCmd->dCmdCnt>=CMD_PC_2_EMB_BUFF_NUMS)
		return ERR_DATA_OVERFLOW;
	
	pDest = &(pCmd->dCmd[(pCmd->dCmdWr)][0]);

	memcpy(pDest,pdat,len);

	pCmd->dCmdWr++;
	pCmd->dCmdCnt++;

	if(pCmd->dCmdWr>=CMD_PC_2_EMB_BUFF_NUMS)
	{
		pCmd->dCmdWr = 0;
	}

    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
        (pDebug->secondFun&SecFunDebugCtrl_XQueue_2_XCmdBuf_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
    {
	    Bsp_printf("Store to Arm Cmd,fCode:0x%x,id:%d,Pos:0x%x,Spd:0x%x",(pdat[0]+(pdat[1]<<8)),id,(pdat[8]+(pdat[9]<<8)),(pdat[10]+(pdat[11]<<8)));
    }
	
	return ERR_NONE;
	
}


/* --------------------------------------------------------------------------*/
/**
* @name sysPcToArmsCmdProcess
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
s8 sysPcToArmsCmdProcess(u16 Code,u32 timStamp,u16 dlen,u8* pdat)
{
	u8 i;
	s8 res=ERR_NONE;
	u8 Unit=0;
	u16 iD;
	u8 dat[20]={0};
		
	//!< 检查指令码 
	switch(Code)
	{
		case PCCmd_ArmPosMoveMode:
					Unit = PCCmd_ArmPosMoveMode_Unit;					  //!< id(2bytes) + position(2bytes) + speed(2bytes)
							
					if(dlen<Unit) 
						res = ERR_INPUT_PARAMETERS;
				
			break;
            /*
		case PCCmd_ArmSpdMoveMode:     //!< 20180508 modify ling //!< 头部与手臂均为关节模式位置模式，无速度模式。若发送速度模式，则不执行此指令
					Unit = PCCmd_ArmSpdMoveMode_Unit;
							
					if(dlen<Unit) 
						res = ERR_INPUT_PARAMETERS;
				
				break;*/
	/*
		case PCCmd_ArmPIDMode:
			break;
		case PCCmd_ArmTorqueSwitch:
			break;
		case PCCmd_ArmMotorLimit:
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
			
		if((iD < Emb_StartID_ArmL)||(iD >= (Emb_StartID_ArmL + Motor_Num_Arm)))
		{
			i += Unit;
		}
		else
		{
			memcpy(&dat[6],(pdat+i),Unit);
										//12Bytes//  2By    4By  2By    2By    2By
						//!< 手臂指令缓存16个12bytes:Code0-1 T0-3 iD0-1 Pos0-1 Speed0-1		
			//res = l_StorePcCmdtoArmsCmdBuff(iD,dat,(Unit+6));
			res = g_StorePcCmdtoCmdBuff(EwayArms.mControl.pToMotorCmd,(iD-Emb_StartID_ArmL),dat,(Unit+6));

			if(res != ERR_NONE)
			{
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
				    SysLogWrite(LogMajor,"Arms g_StorePcCmdtoCmdBuff() failed,rt:%d",res);

				    Bsp_printf("Arms g_StorePcCmdtoCmdBuff() failed,rt:%d",res);
                }
			}
            else
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_XQueue_2_XCmdBuf_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                {
                    Bsp_printf("Arms g_StorePcCmdtoCmdBuff() suc,fCode:0x%x,id:%d,Pos:0x%x,Spd:0x%x",(dat[0]+(dat[1]<<8)),iD,(dat[8]+(dat[9]<<8)),(dat[10]+(dat[11]<<8)));
                }
            }
			
			i += Unit;
		}
	}
		
	return ERR_NONE;
}



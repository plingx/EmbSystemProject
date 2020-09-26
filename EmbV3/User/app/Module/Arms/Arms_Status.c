#include "Arms_Status.h"
#include "includes.h"

extern u8 ArmSvdelayTm[Motor_Num_ArmLeft];

//从寄存器0x78开始，查询15个。
s8 SendQueryArmsStatus(void)
{
	s8 sRet1=0,sRet2=0;
	sRet1 = sysGeneralMotorBroadcastRead(Emb_StartID_ArmL,Motor_Num_ArmLeft,SERVO_REG_READ_START_ADDR,SERVO_REG_READ_NUM_ARM,ArmSvdelayTm,EWAYBOT_ARMS_LEFT);
	sRet2 = sysGeneralMotorBroadcastRead(Emb_StartID_ArmR,Motor_Num_ArmRight,SERVO_REG_READ_START_ADDR,SERVO_REG_READ_NUM_ARM,ArmSvdelayTm,EWAYBOT_ARMS_RIGHT);

	if(ERR_NONE!=sRet1)
	{
		SysLogWrite(LogMajor,"SendQuery_ArmStatus ,Left Arm send query is error, uRet=%d",sRet1);
		return sRet1;
	}

	if(ERR_NONE!=sRet2)
	{
		SysLogWrite(LogMajor,"SendQuery_ArmStatus ,Right Arm send query is error, uRet=%d",sRet2);
		return sRet2;
	}

	return ERR_NONE;
}


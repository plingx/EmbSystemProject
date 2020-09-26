#ifndef _G_R_I_P_P_E_R_H
#define _G_R_I_P_P_E_R_H

#include "bsp.h"

#define  GRIPPER_NUM 2
#define  GRIPPER_PAYLOAD_BYTES 15

typedef struct{
	u16 uPosition;
	u16 uSpeed;
	u32 uStatus;
	u16 uVoltage;
	u16 uCurrent;
	u16 uLoad;
	u8 uTemperature;
}SingleGripperStatus;

typedef struct{
	SingleGripperStatus sSingleGripperStatus[GRIPPER_NUM];
}GripperStatus;


s8 SendQueryGrippersStatus(void);
s8 ReportGripperStatusToPC(u8 *uData, u16 *uLen);

#endif


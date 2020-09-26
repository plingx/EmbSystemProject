#ifndef _WHEEL_STATUS_H
#define _WHEEL_STATUS_H

#include "bsp.h"

#define  WHEEL_NUM 2
#define  WHEEL_PAYLOAD_BYTES 17

typedef struct{
	u32 uPosition;
	u16 uSpeed;
	u32 uStatus;
	u16 uVoltage;
	u16 uCurrent;
	u8 uTemperature;
}SingleWheelStatus;

typedef struct{
	SingleWheelStatus sSingleWheelStatus[WHEEL_NUM];
}WheelsStatus;


s8 SendQueryWheelStatus(void);
s8 GetWheelsStatus(WheelsStatus *pWheelsStatus);
s8 ReportWheelsStatusToPC(u8 *uData, u16 *uLen);


#endif


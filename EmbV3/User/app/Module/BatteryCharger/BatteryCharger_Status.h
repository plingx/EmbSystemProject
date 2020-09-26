#ifndef BATTERY_CHARGER_STATUS_H
#define BATTERY_CHARGER_STATUS_H

//#include "stm32f4xx.h"
#include "BatteryCharger.h"

#define BatteryCharger_SlaveAddr             0x01
#define BTCharger_QUERY_PKT_RESPONSE_LEN     0x15




s8 SendQueryBatteryChargerStatus(void);
s8 ReportBatteryStatusToPC(ModuleBatteryCharger *psBatteryStatus,u8 *uData, u8 *uLen);
s8 g_GetBatteryStatus(EwayBatteryChargerModule *psBattery,u8* pd,u8 lenth);


#endif

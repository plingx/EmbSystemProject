#ifndef _P_S_M_C_M_D_H
#define _P_S_M_C_M_D_H

#include "stm32f4xx.h"
#include "PSM.h"




s8 SetPSMCmdTcpErr(void);

s8 SetPSMCmdTcpReConnect(void);

s8 SetPSMCmdLedCtrl(LedCtrlType eLedCtrl);

s8 SetPSMCmdSwitchCtrl(u8 uSwitchCtrl);

s8 SetPSMCmdPowerOff(void);

//µ¥λΪs
s8 SetPSMCmdPowerOffDelay(u8 uDelayTime);

#endif

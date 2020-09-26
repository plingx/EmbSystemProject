#ifndef _P_S_M_S_T_A_T_U_S_H_
#define _P_S_M_S_T_A_T_U_S_H_

#include "stm32f4xx.h"
#include "bsp.h"
#include "PSM.h"



#define PSM_RESPONSE_INFO_LEN_MIN	    8    //!< DD DD ID LEN CODE STATUS CRCL CRCH
#define PSM_QUERY_PKT_RESPONSE_LEN     0x2A





s8 SendQueryPSMStatus(void);
//s8 GetPSMStatus(PSM *pPSMStatus);
s8 ReportPSMStatusToPC(PSM *pPSMStatus,u8 *uData, u8 *uLen);
//s8 ReadPSMData(PSM *pPSMStatus,u8 *uData,u8 uLen);
//s8 CopyUSARTRecieve(UART_T *psUart,u8 *punBuf,u16 unNum);

s8 g_GetPsmStatus(EwayPSMModule *pPSM,u8* pd,u8 lenth);



#endif

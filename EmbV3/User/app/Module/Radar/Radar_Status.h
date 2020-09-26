#ifndef R_A_D_A_R_STAT_H
#define R_A_D_A_R_STAT_H

#include "bsp.h"

#define  RADAR_FRONT_NUM     4
#define  RADAR_BEHIND_NUM   2
#define RADAR_NUM       (RADAR_FRONT_NUM+RADAR_BEHIND_NUM)     //总共6个测距超声波雷达，前面4个，后面2个

typedef struct{
    u16 unDistans[RADAR_NUM];
    u8   unAvoidFlag[RADAR_NUM];
}RadarStatus;

s8 GetRadarStatus(RadarStatus *psRadarStat);
s8 SetRadarStatusToPC(RadarStatus *psRadarStat,u8 *puDat,u8 *puLen);


#endif

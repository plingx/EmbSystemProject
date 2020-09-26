
#include "Radar_Status.h"

extern u8 g_unAvoidSwitch;

s8 GetRadarStatus(RadarStatus *psRadarStat)
{
    u8 uDistance[(RADAR_NUM<<1)];
    
    if(NULL==psRadarStat)
    {
        return ERR_POINTER_NULL;
    }
    
    GetTim9RadarInfo(RADAR_NUM,uDistance);

    for(u8 i=0;i<RADAR_NUM;i++)
    {
        psRadarStat->unDistans[i]=(u16)uDistance[i];
    }

    return ERR_NONE;
}

/*
                                 �汾��    ָ��    ʱ���            ���ݳ���        1����cm    ��    n����cm           ״̬     ע����NUM�ݶ�9��    
TOF����״̬�ϴ�           FE E1    03 00    00 00 00 00    0x0C+NUM*2        00 00                00 00           00 00        
                                                                                                                                               00 80    �����Լ쿪    bit 16
                                                                                                                                                        1����0����
*/

s8 SetRadarStatusToPC(RadarStatus *psRadarStat,u8 *puDat,u8 *puLen)
{
    u16 uRadarStat;
    
    if(NULL==psRadarStat||NULL==puDat||NULL==puLen)
    {
        return ERR_POINTER_NULL;
    }

    for(u8 i=0;i<RADAR_NUM;i++)
    {
        puDat[10+2*i]=psRadarStat->unDistans[i]&0xFF;
        puDat[10+2*i+1]=(psRadarStat->unDistans[i]>>8)&0xFF;
        uRadarStat|=(psRadarStat->unAvoidFlag[i]<<i);//���ϱ�ʶ
    }

    if(1==g_unAvoidSwitch)
    {
        uRadarStat |= 0x8000;
    }
    
    puDat[10+2*6]=uRadarStat&0xFF;
    puDat[10+2*6+1]=(uRadarStat>>8)&0xFF;

    return ERR_NONE;
}



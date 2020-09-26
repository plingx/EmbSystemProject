#ifndef R_A_D_A_R_H
#define R_A_D_A_R_H

#include "bsp.h"
#include "Radar_Status.h"
#include "Emb_MyServo.h"

#define ObstDistance_Brake              35  //!< Ĭ�ϵ�����ﵽ35����ʱ��ɲ��
#define Avoidance_WHEEL_SPEED_NUMS      7
#define WHEELS_STATIC_STOP_VALUE         5   //!< ��ĳ�����ӵ��ٶȾ���ֵС�� 5 rpm���������Ϊ�Ǿ�ֹ��
#define WHEELS_STATIC_STOP_TIMES         4   //!< ˫�ֱ��ж���ֹ����С����
#define WHEELS_STATIC_TURN_TIMES         4   //!< ˫�ֱ��ж�תСȦ����С����


#define WHEELS_MOVEMENTSTATUS_NONE          0x00    //!< ˫���˶�״̬δ����
#define WHEELS_MOVEMENTSTATUS_STOP          0x10    //!< ˫��Ϊ��ֹ״̬
#define WHEELS_MOVEMENTSTATUS_TURN_STOP     0x20    //!< ˫��Ϊԭ��תСȦ����������ԳƵķ�ʽ�˶���
#define WHEELS_MOVEMENTSTATUS_TURN_AHEAD    0x21    //!< ˫��Ϊԭ��תСȦ�����������ǰ�˶���
#define WHEELS_MOVEMENTSTATUS_TURN_BACK     0x22    //!< ˫��Ϊԭ��תСȦ�������������˶���
#define WHEELS_MOVEMENTSTATUS_MOVE          0x30    //!< ˫��Ϊ��ǰor�������״̬

typedef struct{
    u8 ResUpdat;        //!< ��ŵĽ���Ƿ�Ϊˢ�¹��ģ���ʼ��Ϊ0,��Ϊ���¹��ģ���Ϊ��0��ʹ�����Ҫ���
    u8 Result;          //!< ���ÿ���������˶���������Ľ�� 1
    s8 Sum;             //!< ���ÿ���������˶���������Ľ�� 2
}ObstAvoidMoveMentStatusModule;


typedef struct{
    u8 Swi;                 //!< ���Ϲ����� 0: off   1:on
    u8 distance;            //!< ���Ͼ��� cm  ���255cm ������
    u16 unAvoidFlag;        //!< ���ϱ�ʶ״̬λ bit0-5��ʾChannel x �ڹ涨�������Ƿ����ϰ���(0:��,1:û��) bit15���ڱ�ʾ��ǰ���Ͽ����Ƿ�� ���ϴ�״̬��PCʱ��bit15��Ҫ����ˢ��ΪSwi��ֵ
    u16 AvoidEnableFlag;    //!< ���ϱ�ʶ״̬λ bit0-5��ʾChannel x �Ƿ��ڶ���ϰ���״̬��
    u8 WheelSpd[Motor_Num_Wheel][8];        //!< ���ڼ�¼2�����ӵ��˶����ƣ�ͨ�����ӵ�ǰ���ٶ�ֵ���ж�Ŀǰ����ǰ��������˶���
    ObstAvoidMoveMentStatusModule WheelMove;
}EwayObstAvoidModule;

typedef struct{
    u8 UltraSoundDist[RADAR_NUMS_MAX];          //!< ����������̽ͷ̽�⵽�ľ���
    u8 RecvFlag;                                //!< ����������̽ͷ��Ч�ľ�����ձ�־����ϵͳ���ڲ�ѯʹ��
}EwayUltraSoundModule;

typedef struct{
    s16 LeftSpd[Avoidance_WHEEL_SPEED_NUMS];              //!< ��������ٶȵĻ���
    s16 RightSpd[Avoidance_WHEEL_SPEED_NUMS];             //!< ��������ٶȵĻ���
}EwayWheelMoveModule;

typedef struct{
    EwayObstAvoidModule ObstAvoid;
    EwayUltraSoundModule RadarCollect;
    EwayWheelMoveModule WheelMov;
}EwayRadarObstAvoidModule;




s8  EmergencyObstAvoid(s16 sLeftWheelSpeed,s16 sRightWheelSpeed,RadarStatus *psRadarStat);
void sysEmbCheckRadarAndObstacleAvoid(void);
void sysEmbMotorSingleCycleCmdStatusClear(void);
s8 g_WheelMovementStatusAnalysis(u8* pRslt,s8* pSum,s16* pLd,s16* pRd);



#endif

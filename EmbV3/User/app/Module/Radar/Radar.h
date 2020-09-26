#ifndef R_A_D_A_R_H
#define R_A_D_A_R_H

#include "bsp.h"
#include "Radar_Status.h"
#include "Emb_MyServo.h"

#define ObstDistance_Brake              35  //!< 默认当距离达到35厘米时就刹车
#define Avoidance_WHEEL_SPEED_NUMS      7
#define WHEELS_STATIC_STOP_VALUE         5   //!< 当某个轮子的速度绝对值小于 5 rpm，则可以认为是静止的
#define WHEELS_STATIC_STOP_TIMES         4   //!< 双轮被判定静止的最小次数
#define WHEELS_STATIC_TURN_TIMES         4   //!< 双轮被判定转小圈的最小次数


#define WHEELS_MOVEMENTSTATUS_NONE          0x00    //!< 双轮运动状态未分析
#define WHEELS_MOVEMENTSTATUS_STOP          0x10    //!< 双轮为静止状态
#define WHEELS_MOVEMENTSTATUS_TURN_STOP     0x20    //!< 双轮为原地转小圈，且是以轴对称的方式运动的
#define WHEELS_MOVEMENTSTATUS_TURN_AHEAD    0x21    //!< 双轮为原地转小圈，且是相对向前运动的
#define WHEELS_MOVEMENTSTATUS_TURN_BACK     0x22    //!< 双轮为原地转小圈，且是相对向后运动的
#define WHEELS_MOVEMENTSTATUS_MOVE          0x30    //!< 双轮为向前or向后行走状态

typedef struct{
    u8 ResUpdat;        //!< 存放的结果是否为刷新过的，初始化为0,若为更新过的，则为非0，使用完后要清除
    u8 Result;          //!< 存放每周期轮子运动情况分析的结果 1
    s8 Sum;             //!< 存放每周期轮子运动情况分析的结果 2
}ObstAvoidMoveMentStatusModule;


typedef struct{
    u8 Swi;                 //!< 避障功能是 0: off   1:on
    u8 distance;            //!< 避障距离 cm  最大255cm 够用了
    u16 unAvoidFlag;        //!< 避障标识状态位 bit0-5表示Channel x 在规定距离内是否有障碍物(0:有,1:没有) bit15用于表示当前避障开关是否打开 在上传状态给PC时，bit15都要重新刷新为Swi的值
    u16 AvoidEnableFlag;    //!< 避障标识状态位 bit0-5表示Channel x 是否处于躲避障碍的状态下
    u8 WheelSpd[Motor_Num_Wheel][8];        //!< 用于记录2个轮子的运动趋势，通过轮子当前的速度值，判断目前是向前还是向后运动的
    ObstAvoidMoveMentStatusModule WheelMove;
}EwayObstAvoidModule;

typedef struct{
    u8 UltraSoundDist[RADAR_NUMS_MAX];          //!< 各个超声波探头探测到的距离
    u8 RecvFlag;                                //!< 各个超声波探头有效的距离接收标志，供系统周期查询使用
}EwayUltraSoundModule;

typedef struct{
    s16 LeftSpd[Avoidance_WHEEL_SPEED_NUMS];              //!< 存放左轮速度的缓存
    s16 RightSpd[Avoidance_WHEEL_SPEED_NUMS];             //!< 存放右轮速度的缓存
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

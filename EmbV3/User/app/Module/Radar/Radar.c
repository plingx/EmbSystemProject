#include "includes.h"

extern EwayEmbSysModule EwayEmbSys;
extern EwayWheelModule EwayWheels;
extern EwayEmbSysDebugModule* pDebug;





EwayRadarObstAvoidModule EwayRadarObstAvoid={
                                                {0,ObstDistance_Brake,0x0000,0x0000,{{0},{0}},{0,0,0}},
                                                {{0,0,0,0,0,0,0,0},0},
                                                {{0},{0}}
                                            };



//紧急避障功能   其实应该是紧急刹车功能，当超声波雷达检测到障碍物到达一定距离时，就刹车。


s8 l_WheelMovementStatusUpdate(s16 spd0,s16 spd1)
{
    u8 i;
    s16 *pSpd0 = &EwayRadarObstAvoid.WheelMov.LeftSpd[0];
    s16 *pSpd1 = &EwayRadarObstAvoid.WheelMov.RightSpd[0];

    for(i=1;i<Avoidance_WHEEL_SPEED_NUMS;i++)
    {
        pSpd0[i-1] = pSpd0[i];
        pSpd1[i-1] = pSpd1[i];
    }

    pSpd0[Avoidance_WHEEL_SPEED_NUMS-1] = spd0;
    pSpd1[Avoidance_WHEEL_SPEED_NUMS-1] = spd1;

    return ERR_NONE;
}
/*

        spd0 > 0, spd1 < 0          5       相对的正前方

        spd0 = 0, spd1 < 0          3       向左前转弯

        spd0 > 0, spd1 = 0          3       向右前转弯

        spd0 < 0, spd1 > 0         -5       相对的正后方

        spd0 = 0, spd1 > 0         -3       向左后转弯

        spd0 < 0, spd1 = 0         -3       向右后转弯
        
        spd0 > 0, spd1 > 0          
                |spd0| >= |spd1|    1       相对向右前转弯
                |spd0| <  |spd1|   -1       相对向左后转弯

        spd0 < 0, spd1 < 0  
                |spd0| >= |spd1|   -1       相对向右后转弯
                |spd0| <  |spd1|    1       相对向左前转弯

        spd0 = 0, spd1 = 0          0       相对静止


根据最近5次的左右轮速度，计算得分，判断运动趋势

*/
/*
s8 g_WheelMovementStatusAnalysis(s8* pRslt,s16* pLd,s16* pRd)
{
    s8 sum=0;
    u8 i;

    for(i=0;i<Avoidance_WHEEL_SPEED_NUMS;i++)
    {
        if((pLd[i] > 0)&&(pRd[i] < 0))                    //!< 相对的正前方
        {
            sum += 5;
        }
        else if(((pLd[i] == 0)&&(pRd[i] < 0))||((pLd[i] > 0)&&(pRd[i] == 0)))       //!< 向左前转弯 or 向右前转弯
        {
            sum += 3;
        }
        else if((pLd[i] < 0)&&(pRd[i] > 0))     //!< 相对的正后方
        {
            sum += (-5);
        }
        else if(((pLd[i] == 0)&&(pRd[i] > 0))||((pLd[i] < 0)&&(pRd[i] == 0)))       //!< 向左后转弯 or 向右后转弯
        {
            sum += (-3);
        }
        else if((pLd[i] > 0)&&(pRd[i] > 0))
        {
            if(abs(pLd[i]) >= abs(pRd[i]))      //!< 相对向右前转弯
            {
                sum += 1;
            }
            else                                //!< 相对向左后转弯
            {
                sum += (-1);
            }
        }
        else if((pLd[i] < 0)&&(pRd[i] < 0))
        {
            if(abs(pLd[i]) >= abs(pRd[i]))      //!< 相对向右后转弯
            {
                sum += (-1);
            }
            else                                //!< 相对向左前转弯
            {
                sum += 1;
            }
        }
        else if((pLd[i] == 0)&&(pRd[i] == 0))       //!< 静止
        {
            sum += 0;
        }           

    }

    *pRslt = sum;
    
    return ERR_NONE;
}*/

/*

        spd0 > 0, spd1 < 0
                |spd0| == |spd1|    5       相对的正前方
                |spd0| >  |spd1|    3       向右前转弯
                |spd0| <  |spd1|    3       向左前转弯
        spd0 = 0, spd1 < 0          3       向左前转弯
        spd0 > 0, spd1 = 0          3       向右前转弯

        spd0 < 0, spd1 > 0
                |spd0| == |spd1|   -5       相对的正后方
                |spd0| >  |spd1|   -3       向右后转弯
                |spd0| <  |spd1|   -3       向左后转弯
        spd0 = 0, spd1 > 0         -3       向左后转弯
        spd0 < 0, spd1 = 0         -3       向右后转弯
        
        spd0 > 0, spd1 > 0   
                |spd0| == |spd1|    0       原地转圈相对静止
                |spd0| >  |spd1|    1       相对向右前转弯
                |spd0| <  |spd1|   -1       相对向左后转弯

        spd0 < 0, spd1 < 0
                |spd0| == |spd1|    0       原地转圈相对静止
                |spd0| >= |spd1|   -1       相对向右后转弯
                |spd0| <  |spd1|    1       相对向左前转弯

        spd0 = 0, spd1 = 0          0       静止


根据最近7次的左右轮速度，计算得分，判断运动趋势，并保存在全局变量中待sysModuleWheelsMovement()中读取分析使用

*/
s8 g_WheelMovementStatusAnalysis(u8* pRslt,s8* pSum,s16* pLd,s16* pRd)
{
    s8 sum=0;
    u8 i;
    u8 stopCnt=0;
    u8 circleCnt=0,circltCNT0=0,circltCNT1=0,circltCNT2=0;

    if((pRslt==NULL)||(pSum==NULL)||(pLd==NULL)||(pRd==NULL))
    {
        return ERR_INPUT_PARAMETERS;
    }

    for(i=0;i<Avoidance_WHEEL_SPEED_NUMS;i++)
    {
        //!< 对静止的统计 |spd0|<5 |spd1|<5
        if(((abs(pLd[i])>0)&&(abs(pLd[i])<WHEELS_STATIC_STOP_VALUE))&&((abs(pRd[i])>0)&&(abs(pLd[i])<WHEELS_STATIC_STOP_VALUE)))
        {
            stopCnt++;
        }

        //!< 对原地转小圈的判断
        if(((pLd[i]>0)&&(pRd[i]>0))||((pLd[i]<0)&&(pRd[i]<0)))
        {
            circleCnt++;                    //!< 总的次数大于等于4次

            if((pLd[i] > 0)&&(pRd[i] > 0))
            {
                if(pLd[i] > pRd[i])
                {
                    circltCNT1++;               //!< 原地转小圈，相对向右前方向

                    sum += 1;
                }
                else if(pLd[i] < pRd[i])
                {
                    circltCNT2++;               //!< 原地转小圈，相对向左后方向

                    sum +=(-1);
                }
                else if(pLd[i] == pRd[i])
                {
                    circltCNT0++;               //!< 原地转小圈，圆心在两轮中心，相对静止
                }
            }
            else if((pLd[i] < 0)&&(pRd[i] < 0))
            {
                if(abs(pLd[i]) > abs(pRd[i]))
                {
                    circltCNT2++;               //!< 原地转小圈，相对向右后方向

                    sum += (-1);
                }
                else if(abs(pLd[i]) < abs(pRd[i]))
                {
                    circltCNT1++;               //!< 原地转小圈，相对向左前方向

                    sum += 1;
                }
                else if(abs(pLd[i]) == (pRd[i]))
                {
                    circltCNT0++;               //!< 原地转小圈，圆心在两轮中心，相对静止
                }                
            }            
        }
    
        if((pLd[i] > 0)&&(pRd[i] < 0))                    //!< 相对的正前方
        {
            if(abs(pLd[i])==abs(pRd[i]))
            {
                sum += 5;
            }
            else
            {
                sum += 3;
            }
        }
        else if(((pLd[i] == 0)&&(pRd[i] < 0))||((pLd[i] > 0)&&(pRd[i] == 0)))       //!< 向左前转弯 or 向右前转弯
        {
            sum += 3;
        }
        else if((pLd[i] < 0)&&(pRd[i] > 0))     //!< 相对的正后方
        {
            if(abs(pLd[i])==abs(pRd[i]))
            {
                sum += (-5);
            }
            else
            {
                sum += (-3);
            }
        }
        else if(((pLd[i] == 0)&&(pRd[i] > 0))||((pLd[i] < 0)&&(pRd[i] == 0)))       //!< 向左后转弯 or 向右后转弯
        {
            sum += (-3);
        }         

    }

    //!< 判断是否为静止
    if(stopCnt >= WHEELS_STATIC_STOP_TIMES)                     //!< 判断是否为静止不动的状态
    {
        *pRslt = WHEELS_MOVEMENTSTATUS_STOP;
    }
    else if(circleCnt >= WHEELS_STATIC_TURN_TIMES)              //!< 判断是否为原地转小圈
    {
        if(circltCNT0 > (WHEELS_STATIC_TURN_TIMES-1))               //!< 若大于3次，则认为是原地转圈的状态
        {
            *pRslt = WHEELS_MOVEMENTSTATUS_TURN_STOP;
        }
        else if(circltCNT1 > (WHEELS_STATIC_TURN_TIMES-1))          //!< 若大于3次，则认为是相对向前运动的状态
        {
            *pRslt = WHEELS_MOVEMENTSTATUS_TURN_AHEAD;
        }
        else if(circltCNT2 > (WHEELS_STATIC_TURN_TIMES-1))          //!< 若大于3次，则认为是相对向后运动的状态
        {
            *pRslt = WHEELS_MOVEMENTSTATUS_TURN_BACK;
        }
        else                                                    //!< 若无大于3次的确定运动状态，则认为是正常的向前向后的运动状态(spd0>0,spd1<0) or(spd0<0 spd1>0)
        {
            *pRslt = WHEELS_MOVEMENTSTATUS_MOVE;
        
            *pSum = sum;
        }
    }
    else            //!< 否则为双轮均向前(spd0>0,spd1<0) or向后(spd0<0 spd1>0)的运动状态
    {
        *pRslt = WHEELS_MOVEMENTSTATUS_MOVE;
        
        *pSum = sum;
    }
    
    //Bsp_printf("NoWheelCmdWhelMovAnly-1,Rslt:0x%x,StpCnt:%d,cirCnt:%d-%d-%d,Sum:%d.AvoidF:0x%x.",(*pRslt),stopCnt,circltCNT0,circltCNT1,circltCNT2,(*pSum),EwayRadarObstAvoid.ObstAvoid.unAvoidFlag);
    
    return ERR_NONE;
}




void sysEmbCheckRadarAndObstacleAvoid(void)
{
    MotorSTATEBuff* pWheel;
    EwayObstAvoidModule* pAvoid = &EwayRadarObstAvoid.ObstAvoid;
    u8 uDistance[RADAR_NUM<<1]={0};
    s16 spd[2];//spdX[2];
    s8 res;
    u8 i;

    //!< 更新两轮速度
    
    if(((EwayEmbSys.Comm.wMotors[15]&0x07)!=0)&&((EwayEmbSys.Comm.wMotors[16])&0x07)!=0)
    {
        for(i=0;i<2;i++)            //!<< 取出左右轮的速度
        {
            pWheel = EwayWheels.mState.pFromMotor + i;

            spd[i] = pWheel->mRegs[11] + (pWheel->mRegs[12]<<8);
        }
    }
    else        //!< 若通信不正常，则默认速度为0
    {        
        for(i=0;i<2;i++)            //!<< 取出左右轮的速度
        {
            spd[i] = 0x0000;
        }
    }
    
    l_WheelMovementStatusUpdate(spd[0],spd[1]);      //!< 更新缓存中的左右轮速度

    res = GetTim9RadarInfo(RADAR_NUM,uDistance);       
    
    //!< 根据各路声呐的反馈，和避障距离，更新避障标识和避障使能标识
    pAvoid->unAvoidFlag = 0;
    pAvoid->AvoidEnableFlag = 0;

    if(res == ERR_NONE)
    {
        for (i=0;i<RADAR_NUM;i++)
        {
            if(uDistance[i] <= (pAvoid->distance))    
            {
                pAvoid->unAvoidFlag |= (0x0001 << i);
            }
            else 
            {
                pAvoid->unAvoidFlag &= ~(0x0001 << i);    //!< 若后退中且后2个radar并未探测到障碍物在规定区域内，则清除标志
            }
        }

        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
           (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ObstacleAvoid_MASK))
        {
           Bsp_printf("RadarObstAvoid-3,AvoidFlag:0x%x,GetRadarInfo(cm):1-%d,2-%d,3-%d,4-%d,5-%d,6-%d",pAvoid->unAvoidFlag,uDistance[0],uDistance[1],\
               uDistance[2],uDistance[3],uDistance[4],uDistance[5]);
        }
    }
    else
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
           Bsp_printf("RadarObstAvoid-3,GetTim9RadarInfo() failed,rt:%d.",res);
        }
    }
}



#include "includes.h"

extern EwayEmbSysModule EwayEmbSys;
extern EwayWheelModule EwayWheels;
extern EwayEmbSysDebugModule* pDebug;





EwayRadarObstAvoidModule EwayRadarObstAvoid={
                                                {0,ObstDistance_Brake,0x0000,0x0000,{{0},{0}},{0,0,0}},
                                                {{0,0,0,0,0,0,0,0},0},
                                                {{0},{0}}
                                            };



//�������Ϲ���   ��ʵӦ���ǽ���ɲ�����ܣ����������״��⵽�ϰ��ﵽ��һ������ʱ����ɲ����


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

        spd0 > 0, spd1 < 0          5       ��Ե���ǰ��

        spd0 = 0, spd1 < 0          3       ����ǰת��

        spd0 > 0, spd1 = 0          3       ����ǰת��

        spd0 < 0, spd1 > 0         -5       ��Ե�����

        spd0 = 0, spd1 > 0         -3       �����ת��

        spd0 < 0, spd1 = 0         -3       ���Һ�ת��
        
        spd0 > 0, spd1 > 0          
                |spd0| >= |spd1|    1       �������ǰת��
                |spd0| <  |spd1|   -1       ��������ת��

        spd0 < 0, spd1 < 0  
                |spd0| >= |spd1|   -1       ������Һ�ת��
                |spd0| <  |spd1|    1       �������ǰת��

        spd0 = 0, spd1 = 0          0       ��Ծ�ֹ


�������5�ε��������ٶȣ�����÷֣��ж��˶�����

*/
/*
s8 g_WheelMovementStatusAnalysis(s8* pRslt,s16* pLd,s16* pRd)
{
    s8 sum=0;
    u8 i;

    for(i=0;i<Avoidance_WHEEL_SPEED_NUMS;i++)
    {
        if((pLd[i] > 0)&&(pRd[i] < 0))                    //!< ��Ե���ǰ��
        {
            sum += 5;
        }
        else if(((pLd[i] == 0)&&(pRd[i] < 0))||((pLd[i] > 0)&&(pRd[i] == 0)))       //!< ����ǰת�� or ����ǰת��
        {
            sum += 3;
        }
        else if((pLd[i] < 0)&&(pRd[i] > 0))     //!< ��Ե�����
        {
            sum += (-5);
        }
        else if(((pLd[i] == 0)&&(pRd[i] > 0))||((pLd[i] < 0)&&(pRd[i] == 0)))       //!< �����ת�� or ���Һ�ת��
        {
            sum += (-3);
        }
        else if((pLd[i] > 0)&&(pRd[i] > 0))
        {
            if(abs(pLd[i]) >= abs(pRd[i]))      //!< �������ǰת��
            {
                sum += 1;
            }
            else                                //!< ��������ת��
            {
                sum += (-1);
            }
        }
        else if((pLd[i] < 0)&&(pRd[i] < 0))
        {
            if(abs(pLd[i]) >= abs(pRd[i]))      //!< ������Һ�ת��
            {
                sum += (-1);
            }
            else                                //!< �������ǰת��
            {
                sum += 1;
            }
        }
        else if((pLd[i] == 0)&&(pRd[i] == 0))       //!< ��ֹ
        {
            sum += 0;
        }           

    }

    *pRslt = sum;
    
    return ERR_NONE;
}*/

/*

        spd0 > 0, spd1 < 0
                |spd0| == |spd1|    5       ��Ե���ǰ��
                |spd0| >  |spd1|    3       ����ǰת��
                |spd0| <  |spd1|    3       ����ǰת��
        spd0 = 0, spd1 < 0          3       ����ǰת��
        spd0 > 0, spd1 = 0          3       ����ǰת��

        spd0 < 0, spd1 > 0
                |spd0| == |spd1|   -5       ��Ե�����
                |spd0| >  |spd1|   -3       ���Һ�ת��
                |spd0| <  |spd1|   -3       �����ת��
        spd0 = 0, spd1 > 0         -3       �����ת��
        spd0 < 0, spd1 = 0         -3       ���Һ�ת��
        
        spd0 > 0, spd1 > 0   
                |spd0| == |spd1|    0       ԭ��תȦ��Ծ�ֹ
                |spd0| >  |spd1|    1       �������ǰת��
                |spd0| <  |spd1|   -1       ��������ת��

        spd0 < 0, spd1 < 0
                |spd0| == |spd1|    0       ԭ��תȦ��Ծ�ֹ
                |spd0| >= |spd1|   -1       ������Һ�ת��
                |spd0| <  |spd1|    1       �������ǰת��

        spd0 = 0, spd1 = 0          0       ��ֹ


�������7�ε��������ٶȣ�����÷֣��ж��˶����ƣ���������ȫ�ֱ����д�sysModuleWheelsMovement()�ж�ȡ����ʹ��

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
        //!< �Ծ�ֹ��ͳ�� |spd0|<5 |spd1|<5
        if(((abs(pLd[i])>0)&&(abs(pLd[i])<WHEELS_STATIC_STOP_VALUE))&&((abs(pRd[i])>0)&&(abs(pLd[i])<WHEELS_STATIC_STOP_VALUE)))
        {
            stopCnt++;
        }

        //!< ��ԭ��תСȦ���ж�
        if(((pLd[i]>0)&&(pRd[i]>0))||((pLd[i]<0)&&(pRd[i]<0)))
        {
            circleCnt++;                    //!< �ܵĴ������ڵ���4��

            if((pLd[i] > 0)&&(pRd[i] > 0))
            {
                if(pLd[i] > pRd[i])
                {
                    circltCNT1++;               //!< ԭ��תСȦ���������ǰ����

                    sum += 1;
                }
                else if(pLd[i] < pRd[i])
                {
                    circltCNT2++;               //!< ԭ��תСȦ������������

                    sum +=(-1);
                }
                else if(pLd[i] == pRd[i])
                {
                    circltCNT0++;               //!< ԭ��תСȦ��Բ�����������ģ���Ծ�ֹ
                }
            }
            else if((pLd[i] < 0)&&(pRd[i] < 0))
            {
                if(abs(pLd[i]) > abs(pRd[i]))
                {
                    circltCNT2++;               //!< ԭ��תСȦ��������Һ���

                    sum += (-1);
                }
                else if(abs(pLd[i]) < abs(pRd[i]))
                {
                    circltCNT1++;               //!< ԭ��תСȦ���������ǰ����

                    sum += 1;
                }
                else if(abs(pLd[i]) == (pRd[i]))
                {
                    circltCNT0++;               //!< ԭ��תСȦ��Բ�����������ģ���Ծ�ֹ
                }                
            }            
        }
    
        if((pLd[i] > 0)&&(pRd[i] < 0))                    //!< ��Ե���ǰ��
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
        else if(((pLd[i] == 0)&&(pRd[i] < 0))||((pLd[i] > 0)&&(pRd[i] == 0)))       //!< ����ǰת�� or ����ǰת��
        {
            sum += 3;
        }
        else if((pLd[i] < 0)&&(pRd[i] > 0))     //!< ��Ե�����
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
        else if(((pLd[i] == 0)&&(pRd[i] > 0))||((pLd[i] < 0)&&(pRd[i] == 0)))       //!< �����ת�� or ���Һ�ת��
        {
            sum += (-3);
        }         

    }

    //!< �ж��Ƿ�Ϊ��ֹ
    if(stopCnt >= WHEELS_STATIC_STOP_TIMES)                     //!< �ж��Ƿ�Ϊ��ֹ������״̬
    {
        *pRslt = WHEELS_MOVEMENTSTATUS_STOP;
    }
    else if(circleCnt >= WHEELS_STATIC_TURN_TIMES)              //!< �ж��Ƿ�Ϊԭ��תСȦ
    {
        if(circltCNT0 > (WHEELS_STATIC_TURN_TIMES-1))               //!< ������3�Σ�����Ϊ��ԭ��תȦ��״̬
        {
            *pRslt = WHEELS_MOVEMENTSTATUS_TURN_STOP;
        }
        else if(circltCNT1 > (WHEELS_STATIC_TURN_TIMES-1))          //!< ������3�Σ�����Ϊ�������ǰ�˶���״̬
        {
            *pRslt = WHEELS_MOVEMENTSTATUS_TURN_AHEAD;
        }
        else if(circltCNT2 > (WHEELS_STATIC_TURN_TIMES-1))          //!< ������3�Σ�����Ϊ���������˶���״̬
        {
            *pRslt = WHEELS_MOVEMENTSTATUS_TURN_BACK;
        }
        else                                                    //!< ���޴���3�ε�ȷ���˶�״̬������Ϊ����������ǰ�����˶�״̬(spd0>0,spd1<0) or(spd0<0 spd1>0)
        {
            *pRslt = WHEELS_MOVEMENTSTATUS_MOVE;
        
            *pSum = sum;
        }
    }
    else            //!< ����Ϊ˫�־���ǰ(spd0>0,spd1<0) or���(spd0<0 spd1>0)���˶�״̬
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

    //!< ���������ٶ�
    
    if(((EwayEmbSys.Comm.wMotors[15]&0x07)!=0)&&((EwayEmbSys.Comm.wMotors[16])&0x07)!=0)
    {
        for(i=0;i<2;i++)            //!<< ȡ�������ֵ��ٶ�
        {
            pWheel = EwayWheels.mState.pFromMotor + i;

            spd[i] = pWheel->mRegs[11] + (pWheel->mRegs[12]<<8);
        }
    }
    else        //!< ��ͨ�Ų���������Ĭ���ٶ�Ϊ0
    {        
        for(i=0;i<2;i++)            //!<< ȡ�������ֵ��ٶ�
        {
            spd[i] = 0x0000;
        }
    }
    
    l_WheelMovementStatusUpdate(spd[0],spd[1]);      //!< ���»����е��������ٶ�

    res = GetTim9RadarInfo(RADAR_NUM,uDistance);       
    
    //!< ���ݸ�·���ŵķ������ͱ��Ͼ��룬���±��ϱ�ʶ�ͱ���ʹ�ܱ�ʶ
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
                pAvoid->unAvoidFlag &= ~(0x0001 << i);    //!< ���������Һ�2��radar��δ̽�⵽�ϰ����ڹ涨�����ڣ��������־
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




#include "includes.h"


extern EwayEmbSysModule EwayEmbSys;
extern EwayPSMModule EwayPSM;


EwayBatteryChargerModule EwayBatteryCharger={{0},0};







void sysBatteryChargerMaintainWork(void)
{
    s8 res;
	u8 tmp;
    LedCtrlType led;
    
    if(((EwayEmbSys.sysTimCnt%EMB_CHECK_PSU_Capacity_Interval)==0)&&((EwayEmbSys.Comm.wPsu&0x02) != 0))//!< 10s并且上一次与PSU的通信是正常的，则检查电量，并执行相应操作
    {
        tmp = (u8)(EwayBatteryCharger.sBattCharger.unStatus>>8);

        if((tmp&0x01) != 0)	//!< 当使用电池供电时，电量低则点亮告警，当使用外部电源时，即使电量低也不显示告警
        {
			//!< 当电量低，且PSM指示灯状态并非是红色告警闪烁状态，设置PSM指示灯为告警，标记电池进入低电量状态
            if((EwayBatteryCharger.sBattCharger.unBatteryCapacity <= PSU_Capacity_Low_Limit)&&(EwayPSM.sPSM.unRobotLedStatus!=2)&&((EwayEmbSys.Comm.wPsm&0x01) != 0))
            {            
                res = SetPSMCmdLedCtrl(LED_READ_2HZ);                     //电池电量不足时，通过PSM更改灯带的点亮方式
                if(res == ERR_NONE)
                {
                    EwayBatteryCharger.sStatus |= PSU_LowEnergy_MaskBit;
                }
            }//!< 当电池电量恢复，且当前PSM指示灯仍为告警闪烁，则关闭
            else if((EwayBatteryCharger.sBattCharger.unBatteryCapacity > PSU_Capacity_Low_Limit)&&(EwayPSM.sPSM.unRobotLedStatus==2)&&((EwayEmbSys.Comm.wPsm&0x01) != 0))
            {
                switch(EwayPSM.sPSM.unRobotCurrentStatus)
                {
                    case 0:led = LED_BLUE_ON;           //!< 关机状态
                        break;
                    case 1:
                    case 2:led = LED_GREEN_0_5HZ;       //!< 唤醒/启动状态
                        break;
                    case 3:led = LED_YELLOW_2HZ;        //!< 休眠状态
                        break;
                    case 4:led = LED_BLUE_1HZ;          //!< 急停状态
                        break;
                    case 6:led = LED_OFF;               //!< 休眠状态
                        break;
                    default:led = LED_BLUE_ON;
                        break;                    
                }

                res = SetPSMCmdLedCtrl(led);//电池电量恢复后，通过PSM更改灯带的点亮方式，目前需求不明确，先做成点亮蓝色待后续明确后再修改
                if(res == ERR_NONE)
                {
                    EwayBatteryCharger.sStatus &= ~(PSU_LowEnergy_MaskBit);
                }
            }
		}
        else
        {
             if((EwayPSM.sPSM.unRobotLedStatus==2)&&((EwayEmbSys.Comm.wPsm&0x01) != 0))
             {
                switch(EwayPSM.sPSM.unRobotCurrentStatus)
                {
                    case 0:led = LED_BLUE_ON;           //!< 关机状态
                        break;
                    case 1:
                    case 2:led = LED_GREEN_0_5HZ;       //!< 唤醒/启动状态
                        break;
                    case 3:led = LED_YELLOW_2HZ;        //!< 休眠状态
                        break;
                    case 4:led = LED_BLUE_1HZ;          //!< 急停状态
                        break;
                    case 6:led = LED_OFF;               //!< 休眠状态
                        break;
                    default:led = LED_BLUE_ON;
                        break;                    
                }

                res = SetPSMCmdLedCtrl(led);        //!< 根据当前robot的状态，更改PSM的灯带颜色
                if(res == ERR_NONE)
                {
                    EwayBatteryCharger.sStatus &= ~(PSU_LowEnergy_MaskBit);
                }
             }
        }
    }
    
}





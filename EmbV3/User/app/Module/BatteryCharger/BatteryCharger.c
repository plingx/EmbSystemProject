
#include "includes.h"


extern EwayEmbSysModule EwayEmbSys;
extern EwayPSMModule EwayPSM;


EwayBatteryChargerModule EwayBatteryCharger={{0},0};







void sysBatteryChargerMaintainWork(void)
{
    s8 res;
	u8 tmp;
    LedCtrlType led;
    
    if(((EwayEmbSys.sysTimCnt%EMB_CHECK_PSU_Capacity_Interval)==0)&&((EwayEmbSys.Comm.wPsu&0x02) != 0))//!< 10s������һ����PSU��ͨ���������ģ������������ִ����Ӧ����
    {
        tmp = (u8)(EwayBatteryCharger.sBattCharger.unStatus>>8);

        if((tmp&0x01) != 0)	//!< ��ʹ�õ�ع���ʱ��������������澯����ʹ���ⲿ��Դʱ����ʹ������Ҳ����ʾ�澯
        {
			//!< �������ͣ���PSMָʾ��״̬�����Ǻ�ɫ�澯��˸״̬������PSMָʾ��Ϊ�澯����ǵ�ؽ���͵���״̬
            if((EwayBatteryCharger.sBattCharger.unBatteryCapacity <= PSU_Capacity_Low_Limit)&&(EwayPSM.sPSM.unRobotLedStatus!=2)&&((EwayEmbSys.Comm.wPsm&0x01) != 0))
            {            
                res = SetPSMCmdLedCtrl(LED_READ_2HZ);                     //��ص�������ʱ��ͨ��PSM���ĵƴ��ĵ�����ʽ
                if(res == ERR_NONE)
                {
                    EwayBatteryCharger.sStatus |= PSU_LowEnergy_MaskBit;
                }
            }//!< ����ص����ָ����ҵ�ǰPSMָʾ����Ϊ�澯��˸����ر�
            else if((EwayBatteryCharger.sBattCharger.unBatteryCapacity > PSU_Capacity_Low_Limit)&&(EwayPSM.sPSM.unRobotLedStatus==2)&&((EwayEmbSys.Comm.wPsm&0x01) != 0))
            {
                switch(EwayPSM.sPSM.unRobotCurrentStatus)
                {
                    case 0:led = LED_BLUE_ON;           //!< �ػ�״̬
                        break;
                    case 1:
                    case 2:led = LED_GREEN_0_5HZ;       //!< ����/����״̬
                        break;
                    case 3:led = LED_YELLOW_2HZ;        //!< ����״̬
                        break;
                    case 4:led = LED_BLUE_1HZ;          //!< ��ͣ״̬
                        break;
                    case 6:led = LED_OFF;               //!< ����״̬
                        break;
                    default:led = LED_BLUE_ON;
                        break;                    
                }

                res = SetPSMCmdLedCtrl(led);//��ص����ָ���ͨ��PSM���ĵƴ��ĵ�����ʽ��Ŀǰ������ȷ�������ɵ�����ɫ��������ȷ�����޸�
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
                    case 0:led = LED_BLUE_ON;           //!< �ػ�״̬
                        break;
                    case 1:
                    case 2:led = LED_GREEN_0_5HZ;       //!< ����/����״̬
                        break;
                    case 3:led = LED_YELLOW_2HZ;        //!< ����״̬
                        break;
                    case 4:led = LED_BLUE_1HZ;          //!< ��ͣ״̬
                        break;
                    case 6:led = LED_OFF;               //!< ����״̬
                        break;
                    default:led = LED_BLUE_ON;
                        break;                    
                }

                res = SetPSMCmdLedCtrl(led);        //!< ���ݵ�ǰrobot��״̬������PSM�ĵƴ���ɫ
                if(res == ERR_NONE)
                {
                    EwayBatteryCharger.sStatus &= ~(PSU_LowEnergy_MaskBit);
                }
             }
        }
    }
    
}





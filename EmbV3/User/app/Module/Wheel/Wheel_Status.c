#include "Wheel_Status.h"
#include "includes.h"

extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];
extern EwayEmbSysModule EwayEmbSys;
extern EwayWheelModule EwayWheels;



u8 WheelSvdelayTm[Motor_Num_Wheel]={0x01,0x11};

//从0x78开始，查询21个寄存器
s8 SendQueryWheelStatus(void)
{
	s8 sRet=0;
	sRet = sysGeneralMotorBroadcastRead(Emb_StartID_Wheel,Motor_Num_Wheel,SERVO_REG_READ_START_ADDR,SERVO_REG_READ_NUM_WHEEL,WheelSvdelayTm,EWAYBOT_WHEEL);

	if(ERR_NONE != sRet)
	{
		return sRet;
	}

	return ERR_NONE;
}

#if Emb_Wheel_Offline_Debug
extern u32 EwayWheelCommState[2];
#endif

s8 ReportWheelsStatusToPC(u8 *uData, u16 *uLen)
{
    u8* pd;
    u8 uStartWheelId=Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head;
    
	if(NULL == uData||NULL == uLen)
	{
		return ERR_POINTER_NULL;
	}

	for(u8 i=0;i<WHEEL_NUM;i++)
	{    
		uData[i*WHEEL_PAYLOAD_BYTES+10] = (u8)(i + Emb_StartID_Wheel);                                       //(u8)(0xFF&(i+uStartWheelId));   ling-20180427 modify
		uData[i*WHEEL_PAYLOAD_BYTES+11] = (u8)((i + Emb_StartID_Wheel)>>8); 

        if((EwayEmbSys.Comm.wMotors[i+EMB_COMM_WHEEL_START_BIT]&0x01)==0)       //!< 最近一次通信不成功，则上报最近一次通信成功的数据
        {
            pd = &((EwayWheels.mState.pFromMotor+i)->mRegs[0]);
            
    		uData[i*WHEEL_PAYLOAD_BYTES+12] = pd[15]; 
    		uData[i*WHEEL_PAYLOAD_BYTES+13] = pd[16]; 
    		uData[i*WHEEL_PAYLOAD_BYTES+14] = pd[17];  
    		uData[i*WHEEL_PAYLOAD_BYTES+15] = pd[18]; 
    		uData[i*WHEEL_PAYLOAD_BYTES+16] = pd[11]; 
    		uData[i*WHEEL_PAYLOAD_BYTES+17] = pd[12];
            uData[i*WHEEL_PAYLOAD_BYTES+18] = pd[0];                           //轮毂控制器返回的状态。
		    uData[i*WHEEL_PAYLOAD_BYTES+19] = pd[1];
        }
        else
        {
    		uData[i*WHEEL_PAYLOAD_BYTES+12] = (u8)(sysEwServo[i+uStartWheelId].uPosWheel); 
    		uData[i*WHEEL_PAYLOAD_BYTES+13] = (u8)((sysEwServo[i+uStartWheelId].uPosWheel)>>8); 
    		uData[i*WHEEL_PAYLOAD_BYTES+14] = (u8)((sysEwServo[i+uStartWheelId].uPosWheel)>>16);  
    		uData[i*WHEEL_PAYLOAD_BYTES+15] = (u8)((sysEwServo[i+uStartWheelId].uPosWheel)>>24); 
    		uData[i*WHEEL_PAYLOAD_BYTES+16] = sysEwServo[i+uStartWheelId].uSpdRPML; 
    		uData[i*WHEEL_PAYLOAD_BYTES+17] = sysEwServo[i+uStartWheelId].uSpdRPMH;

            uData[i*WHEEL_PAYLOAD_BYTES+18] = sysEwServo[i+uStartWheelId].uStaL;                           //轮毂控制器返回的状态。
		    uData[i*WHEEL_PAYLOAD_BYTES+19] = sysEwServo[i+uStartWheelId].uStaH;
        }	
#if Emb_Wheel_Offline_Debug
        if((EwayWheelCommState[i]&0x7FFFFFFF)==0)
#else
        if((EwayEmbSys.Comm.wMotors[i+uStartWheelId]&EMB_COMM_ERR_BITS_3_TIMES) == 0)          //!< 连续3次通信有问题        ling-20180427 modify       
#endif
        {
            uData[i*WHEEL_PAYLOAD_BYTES+20] |= 0x01;
        }
        else
        {
            uData[i*WHEEL_PAYLOAD_BYTES+20] = 0x00;
        }
        uData[i*WHEEL_PAYLOAD_BYTES+21] = 0x00;
        
		uData[i*WHEEL_PAYLOAD_BYTES+22] = sysEwServo[i+uStartWheelId].uinVolL; 
		uData[i*WHEEL_PAYLOAD_BYTES+23]= sysEwServo[i+uStartWheelId].uinVolH;
		uData[i*WHEEL_PAYLOAD_BYTES+24] = sysEwServo[i+uStartWheelId].uinCurtL; 
		uData[i*WHEEL_PAYLOAD_BYTES+25] = sysEwServo[i+uStartWheelId].uinCurtH;
		uData[i*WHEEL_PAYLOAD_BYTES+26] = sysEwServo[i+uStartWheelId].udevTmpt;
	}

	return ERR_NONE;
}





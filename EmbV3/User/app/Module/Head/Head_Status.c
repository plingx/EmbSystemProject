/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Head_Status.c
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author 
* @version 0.0.1
* @date 2018-01-30
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/01/30 | 0.0.1 | Ling     | Create file
*
*/
#include "Head_Status.h"
#include "includes.h"

extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];
extern EwayEmbSysModule EwayEmbSys;
extern EwayEmbSysInfoModule EwayEmbSysInfo;;


u8 ShoulderSvdelayTm[Motor_Num_Shoulder+Motor_Num_Head]={0x01,0x11,0x21};

//头部和肩部使用同一个总线，所以一起查询。查询同样多的数目？
//头部和肩部一样，从0x78开始，查询19个寄存器的值。
s8 SendQueryHeadStatus(void)
{
	s8 sRet=0;

	sRet = sysGeneralMotorBroadcastRead(Emb_StartID_Shoulder,(Motor_Num_Shoulder+Motor_Num_Head),SERVO_REG_READ_START_ADDR,SERVO_REG_READ_NUM_HEAD_SHOULDER,ShoulderSvdelayTm,EWAYBOT_SHOULDER);

	if(ERR_NONE!=sRet)
	{
		return sRet;
	}

	return ERR_NONE;
}
s8 ReportHeadsStatusToPC(u8 *uData, u16 *uLen)
{
    u8* pd = (uData+10);
    u16 len=0;
    u8 unj;

	if(NULL == uData||NULL == uLen)
	{
		return ERR_POINTER_NULL;
	}
	
    //
    //	关于肩部 1)位置和速度都是直接是舵机输出轴的值。舵机与升降平台的系数转换在上位机完成
    //			 2)肩部是否撞击了上限位进行标定，相关的status会存在状态字节中。所以status字节中，头手肩会有差异性的bit
    //
    for(unj=0;unj<(Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head);unj++)
    {
        pd[0] = unj+1;              //!< Arm 1-6,7-12 Shoulder:13,Head:14-15     ling-20170427 modified
        pd[1] = 0;

        if((Motor_Num_Arm+Motor_Num_Shoulder-1)!=unj)
        {
            pd[2] =  sysEwServo[unj].uPosJointL; 
			pd[3] =  sysEwServo[unj].uPosJointH;	
            pd[4] =  sysEwServo[unj].uSpdRPML; 
		    pd[5] =  sysEwServo[unj].uSpdRPMH;
		    pd[6] =  sysEwServo[unj].uStaL;     //电机返回的状态，2个字节
		    pd[7] =  sysEwServo[unj].uStaH;
            
            if((EwayEmbSys.Comm.wMotors[unj]&EMB_COMM_ERR_BITS_3_TIMES)!=0)         //!< ling-20180427 modify
            {
                pd[8] = 0;          // status1
                pd[9] = 0;          // status2
            }
            else
            {
                pd[8] |= 0x01;      // status1
                pd[9] = 0;          // status2
            }        
        
		    pd[10] = sysEwServo[unj].uinVolL; 
		    pd[11] = sysEwServo[unj].uinVolH;
		    pd[12] = sysEwServo[unj].uinCurtL; 
		    pd[13] = sysEwServo[unj].uinCurtH;
		    pd[14] = sysEwServo[unj].udevTmpt;

            pd += EWAYSERVO_PAYLOAD_BYTES;

            len += EWAYSERVO_PAYLOAD_BYTES;
            
        }
        else        //!< id=13  肩部升降电机，车轮模式
        {
            pd[2] = (u8)(0xFF&sysEwServo[unj].uPosWheel); 
            pd[3] = (u8)(0xFF&(sysEwServo[unj].uPosWheel>>8)); 
            pd[4] = (u8)(0xFF&(sysEwServo[unj].uPosWheel>>16)); 
            pd[5] = (u8)(0xFF&(sysEwServo[unj].uPosWheel>>24));
            
            pd[6] =  sysEwServo[unj].uSpdRPML; 
            pd[7] =  sysEwServo[unj].uSpdRPMH;
            pd[8] =  sysEwServo[unj].uStaL;   //电机返回的状态，2个字节
            pd[9] =  sysEwServo[unj].uStaH;
            
            if((EwayEmbSys.Comm.wMotors[unj]&EMB_COMM_ERR_BITS_3_TIMES)!=0)         //!< ling-20180427 modify
            {
                pd[10] = 0;          // status1
            }
            else
            {
                pd[10] |= 0x01;      // status1
            }        

            if((EwayEmbSysInfo.Rds.En!=0)&&(EwayEmbSysInfo.Rds.init != 0))
            {
                pd[11] |= 0x01;      // status2
            }
            else
            {
                pd[11] = 0;          // status2
            }
                    
            pd[12] = sysEwServo[unj].uinVolL; 
            pd[13] = sysEwServo[unj].uinVolH;
            pd[14] = sysEwServo[unj].uinCurtL; 
            pd[15] = sysEwServo[unj].uinCurtH;
            pd[16] = sysEwServo[unj].udevTmpt;
            
            pd += (EWAYSERVO_PAYLOAD_BYTES+2);

            len += (EWAYSERVO_PAYLOAD_BYTES+2);
        }    
	}

	return ERR_NONE;
}


/*s8 ReportHeadsStatusToPC(u8 *uData, u16 *uLen)
{
	if(NULL == uData||NULL == uLen)
	{
		return ERR_POINTER_NULL;
	}
	
  //
//	关于肩部 1)位置和速度都是直接是舵机输出轴的值。舵机与升降平台的系数转换在上位机完成
//				 2)肩部是否撞击了上限位进行标定，相关的status会存在状态字节中。所以status字节中，头手肩会有差异性的bit
  //
	for(u8 unj=0;unj<(Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head);unj++)
	{	
		uData[unj*EWAYSERVO_PAYLOAD_BYTES+10]=  unj+1;              //!< Arm 1-6,7-12 Shoulder:13,Head:14-15     ling-20170427 modified
		uData[unj*EWAYSERVO_PAYLOAD_BYTES+11]=  0; 
			
		if((Motor_Num_Arm+Motor_Num_Shoulder-1)!=unj)
		{
			uData[unj*EWAYSERVO_PAYLOAD_BYTES+12]=  sysEwServo[unj].uPosJointL; 
			uData[unj*EWAYSERVO_PAYLOAD_BYTES+13]=  sysEwServo[unj].uPosJointH;	
            uData[unj*EWAYSERVO_PAYLOAD_BYTES+14] =  sysEwServo[unj].uSpdRPML; 
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+15] =  sysEwServo[unj].uSpdRPMH;
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+18] =  sysEwServo[unj].uStaL;   //电机返回的状态，2个字节
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+19] =  sysEwServo[unj].uStaH;

            if((EwayEmbSys.Comm.wMotors[unj]&EMB_COMM_ERR_BITS_3_TIMES)!=0)         //!< ling-20180427 modify
            {
                uData[unj*EWAYSERVO_PAYLOAD_BYTES+20] = 0;          // status1
                uData[unj*EWAYSERVO_PAYLOAD_BYTES+21] = 0;          // status2
            }
            else
            {
                uData[unj*EWAYSERVO_PAYLOAD_BYTES+20] |= 0x01;      // status1
                uData[unj*EWAYSERVO_PAYLOAD_BYTES+21] = 0;          // status2
            }        
        
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+22] = sysEwServo[unj].uinVolL; 
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+23] = sysEwServo[unj].uinVolH;
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+24] = sysEwServo[unj].uinCurtL; 
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+25] = sysEwServo[unj].uinCurtH;
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+26] = sysEwServo[unj].udevTmpt;
            
		}
		else    //id=13  肩部升降电机，车轮模式
		{
			uData[unj*EWAYSERVO_PAYLOAD_BYTES+12] = (u8)(0xFF&sysEwServo[unj].uPosWheel); 
			uData[unj*EWAYSERVO_PAYLOAD_BYTES+13] = (u8)(0xFF&(sysEwServo[unj].uPosWheel>>8)); 
			uData[unj*EWAYSERVO_PAYLOAD_BYTES+14] = (u8)(0xFF&(sysEwServo[unj].uPosWheel>>16)); 
			uData[unj*EWAYSERVO_PAYLOAD_BYTES+15] = (u8)(0xFF&(sysEwServo[unj].uPosWheel>>24));

            uData[unj*EWAYSERVO_PAYLOAD_BYTES+16] =  sysEwServo[unj].uSpdRPML; 
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+17] =  sysEwServo[unj].uSpdRPMH;
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+18] =  sysEwServo[unj].uStaL;   //电机返回的状态，2个字节
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+19] =  sysEwServo[unj].uStaH;

            if((EwayEmbSys.Comm.wMotors[unj]&EMB_COMM_ERR_BITS_3_TIMES)!=0)         //!< ling-20180427 modify
            {
                uData[unj*EWAYSERVO_PAYLOAD_BYTES+20] = 0;          // status1
                uData[unj*EWAYSERVO_PAYLOAD_BYTES+21] = 0;          // status2
            }
            else
            {
                uData[unj*EWAYSERVO_PAYLOAD_BYTES+20] |= 0x01;      // status1
                uData[unj*EWAYSERVO_PAYLOAD_BYTES+21] = 0;          // status2
            }        
        
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+22] = sysEwServo[unj].uinVolL; 
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+23] = sysEwServo[unj].uinVolH;
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+24] = sysEwServo[unj].uinCurtL; 
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+25] = sysEwServo[unj].uinCurtH;
		    uData[unj*EWAYSERVO_PAYLOAD_BYTES+26] = sysEwServo[unj].udevTmpt;
            
		}		
	}

	return ERR_NONE;
}*/


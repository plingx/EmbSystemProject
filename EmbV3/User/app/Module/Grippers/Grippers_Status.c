#include "includes.h"



extern EwayEmbSysModule EwayEmbSys;
extern DYNAMIXEL_SERVO_STATUS sysDyServo[SYS_MAX_DYNAMIXEL_SERVO_NUMS];
extern EwayGripperModule EwayGrippers;


//Gripper 基本是独立的，与其他功能没有什么耦合关系。

#define LEFT_GRIPPER_ID  1
#define RIGHT_GRIPPER_ID  2

s8 SendQueryGrippersStatus()
{       
    s8 sRet1=0,sRet2=0;
    //后续优化成一条查询命令  连续两条的总线时隙是有问题的。
    sRet1=sysSendDynamixelReadDataPacket(EWAYBOT_GRIPPERS,LEFT_GRIPPER_ID,DYNAMIXEL_REG_CurPosL,8);
    //Delay....
    sRet2=sysSendDynamixelReadDataPacket(EWAYBOT_GRIPPERS,RIGHT_GRIPPER_ID,DYNAMIXEL_REG_CurPosL,8);      //!<COM5 Gripper

    if(ERR_NONE!=sRet1)
    {
        return sRet1;
    }

    if(ERR_NONE!=sRet2)
    {
        return sRet2;
    }

    return ERR_NONE;
}

s8 ReportGripperStatusToPC(u8 *uData, u16 *uLen)
{
    u8 i;
    s16 sSpd;
    u16 SrcSpd;
    
    if(NULL==uData||NULL==uLen)
    {
        return ERR_POINTER_NULL;
    }   

		
    for(i=0;i<GRIPPER_NUM;i++)
    {   
        SrcSpd = sysDyServo[i].unCurSpdL+(sysDyServo[i].unCurSpdH<<8);
        sSpd = (SrcSpd&(~Dynamixel_Direction_MaskBit))*(MOTOR_SPEED_COEFFICIENT*Dynamixel_MoveSpeed_Coeffcient);
    
        uData[i*GRIPPER_PAYLOAD_BYTES+10] = (u8)(LEFT_GRIPPER_ID+i);           //!< ling-20180427 modify
        uData[i*GRIPPER_PAYLOAD_BYTES+11] = (u8)((LEFT_GRIPPER_ID+i)>>8);
        uData[i*GRIPPER_PAYLOAD_BYTES+12] = sysDyServo[i].unCurPosL;            //!< Dynamixel Pos Range:0-4095(0x0FFF)
        uData[i*GRIPPER_PAYLOAD_BYTES+13] = sysDyServo[i].unCurPosH;
        uData[i*GRIPPER_PAYLOAD_BYTES+14] = (u8)sSpd;                             //!< bit10为方向bit 

#if DYNAMIXEL_SPEED_DIRECTION_FLAG
        if((SrcSpd&Dynamixel_Direction_MaskBit)!=0)
        {
            uData[i*GRIPPER_PAYLOAD_BYTES+15] = ((u8)(sSpd>>8))|0x8000;
        }
        else
#endif            
        {
            uData[i*GRIPPER_PAYLOAD_BYTES+15] = (u8)(sSpd>>8);
        }        
        
        uData[i*GRIPPER_PAYLOAD_BYTES+16] = (EwayGrippers.mState.pFromMotor + i)->mRegs[0];
        uData[i*GRIPPER_PAYLOAD_BYTES+17] = 0x00;

        if((EwayEmbSys.Comm.wMotors[i+EMB_COMM_GRIP_START_BIT]&EMB_COMM_ERR_BITS_3_TIMES)==0)
        {
            uData[i*GRIPPER_PAYLOAD_BYTES+18] |= 0x01;
        }
        else
        {
            uData[i*GRIPPER_PAYLOAD_BYTES+18] = (u8)(0x00);
        }        
        uData[i*GRIPPER_PAYLOAD_BYTES+19] = 0;
        
        uData[i*GRIPPER_PAYLOAD_BYTES+20] = sysDyServo[i].unCurVolt;
        uData[i*GRIPPER_PAYLOAD_BYTES+21] = 0;
        uData[i*GRIPPER_PAYLOAD_BYTES+22] = sysDyServo[i].unCurLoadL;
        uData[i*GRIPPER_PAYLOAD_BYTES+23] = sysDyServo[i].unCurLoadH;
        uData[i*GRIPPER_PAYLOAD_BYTES+24] = sysDyServo[i].unCurTempt;
    }
    
    return ERR_NONE;
}


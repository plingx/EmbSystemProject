/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Emb_Gripper.h
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2017-12-20
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2017/12/20 | 0.0.1 | LingPing | Create file
*
*/
#ifndef _EMB_GRIPPER_H_
#define _EMB_GRIPPER_H_

#define DYNAMIXEL_SPEED_DIRECTION_FLAG                   0          //!< 使能上报爪子电机的速度方向


#define DYNAMIXEL_SPEED_DEFAULT                          44        //!< Dynamixel电机默认的速度，当下发的电机指令超限时，赋给爪子电机此默认速度 约5rpm  44*0.114
#define DYNAMIXEL_SPEED_JOINT_MAX                        0x3FF
#define DYNAMIXEL_POSITION_MAX                           0x0FFF   //!< 0-4095

#define Dynamixel_MoveSpeed_Coeffcient                   (0.11)        //!<   RPM = RegValue * 0.11           
#define Dynamixel_Direction_MaskBit                      (0x0400)
#define SYS_MAX_DYNAMIXEL_SERVO_NUMS                     2



#define GRIPPER_FRAME_HEADER0                            0xFF
#define GRIPPER_FRAME_HEADER1                            0xFF

#define GRIPPER_CMD_BROADCAST_ID                         0xFE

#define DYNAMIXEL_CMD_PING                               0x01
#define DYNAMIXEL_CMD_RD_DATA                            0x02
#define DYNAMIXEL_CMD_WR_DATA                            0x03
#define DYNAMIXEL_CMD_RG_WRIT                            0x04
#define DYNAMIXEL_CMD_ACTION                             0x05
#define DYNAMIXEL_CMD_RESET                              0x06
#define DYNAMIXEL_CMD_SYNC_WR                            0x83


#define DYNAMIXEL_RW_REG_NUMS_MAX                        0x50

#define DYNAMIXEL_REG_GoalPosL                           0x1E
#define DYNAMIXEL_REG_GoalPosH                           0x1F




#define DYNAMIXEL_REG_CurPosL                        0x24
#define DYNAMIXEL_REG_CurPosH                        0x25
#define DYNAMIXEL_REG_CurSpdL                        0x26
#define DYNAMIXEL_REG_CurSpdH                        0x27
#define DYNAMIXEL_REG_CurLoadL                       0x28
#define DYNAMIXEL_REG_CurLoadH                       0x29
#define DYNAMIXEL_REG_CurVoltg                       0x2A
#define DYNAMIXEL_REG_CurTempt                       0x2B     //!< present temperature



#define GRIPPER_SYNC_WR_MAX_REG_NUM                    10    
#define GripperStartID                                 1
#define GripperNum                                     2

#define COMM_DYNAMIXEL_RECVD_MSG_LENGTH_MINI           2
#define COMM_DYNAMIXEL_RECVD_MSG_LENGTH_MAXI           50



#define COMM_DYNAMIXEL_MOTOR_MSG_HEADER               0xFFFF



typedef __packed struct{
    u8 unCurPosL;   //!< Dynami Pos Range:0-4095(0x0FFF)
    u8 unCurPosH;
    u8 unCurSpdL;   //!< Dynami Speed Range:0-2047(0x07FF) 0-1023(CCW direction) 1024-2047(CW direction)
    u8 unCurSpdH;
    u8 unCurLoadL;     
    u8 unCurLoadH;     
    u8 unCurVolt;     //!< 实时电压
    u8 unCurTempt;     //!< 实时温度
}DYNAMIXEL_SERVO_STATUS;






//!< 尝试是否可以改成bulk read的模式 20180129
s8 sysSendDynamixelReadDataPacket(EwayMotor_Device_Type devType,u8 devID,u8 regStartAdd,u8 regNum);
s8 sysSendDynamixelBroadcastSyncWritePacket(EwayMotor_Device_Type devType,u8 regStartAdd,u8 regNum,u8 IDStart,u8 IDNums,u8* pdat);
s8 sysParseDynamixelMotorRecvdData(u8* pbuf,u16 length,EwayMotor_Device_Type devType);
s8 sysProcessDynamixelRecvdDataPacket(EwayMotor_Device_Type devType);


#endif

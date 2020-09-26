/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Grippers.h
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
* 2018/01/29 | 0.0.1 | xx       | Create file
*
*/
#ifndef _APP_MODULE_GRIPPERS_H_
#define _APP_MODULE_GRIPPERS_H_


                                            //!< 单个爪子读指令为8 Bytes,同步写2个爪子的4个reg指令最长，为18个字节
#define CMD_GRIPPER2MOTOR_INFO_LEN        20   //!< define the Emb send to head motor cmd buffer length   2*(4+1)+8 = 18
#define CMD_GRIPPER2MOTOR_QUEU_NUM        8    //!< 每个周期可以向电机发送的最大指令数目，可以根据实际需要进行修改。

#define Dynamixel_Motor_Torque_Max        1023

typedef struct{
    NodeCtrlModule mControl;           //!< motor control parameters
    NodeStateModule mState;            //<  motor state: registers and others
    u16 mTorqueLmt[Motor_Num_Gripper];
    u16 mCount;                        //!< number of motors
    u16 mCommStatus;                   //!< every motor communication status  bitx:0-disconnected,1-connection
}EwayGripperModule;



#include "Grippers_Cmd.h"
#include "Grippers_Status.h"


s8 sysPcToGrippersCmdProcess(u16 Code,u32 timStamp,u16 dlen,u8* pdat);

#endif


/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Grippers.h
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
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


                                            //!< ����צ�Ӷ�ָ��Ϊ8 Bytes,ͬ��д2��צ�ӵ�4��regָ�����Ϊ18���ֽ�
#define CMD_GRIPPER2MOTOR_INFO_LEN        20   //!< define the Emb send to head motor cmd buffer length   2*(4+1)+8 = 18
#define CMD_GRIPPER2MOTOR_QUEU_NUM        8    //!< ÿ�����ڿ����������͵����ָ����Ŀ�����Ը���ʵ����Ҫ�����޸ġ�

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


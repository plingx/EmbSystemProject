/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Wheel.h
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
* 2018/01/15 | 0.0.1 | xx       | Create file
*
*/
#ifndef _APP_MODULE_WHEEL_H_
#define _APP_MODULE_WHEEL_H_
                                            //!< �㲥д4��reg��ָ�����Ϊ19���ֽ�
#define CMD_WHEEL2MOTOR_INFO_LEN        30   //!< define the Emb send to wheel motor cmd buffer length   2*(8+1)+9 = 30
#define CMD_WHEEL2MOTOR_QUEU_NUM        4    //!< ÿ�����ڿ����������͵����ָ����Ŀ�����Ը���ʵ����Ҫ�����޸ġ�




typedef struct{    
    NodeCtrlModule mControl;                //!< motor control parameters
    NodeStateModule mState;                 //<  motor state: registers and others
    MotorReRecord mResp[Motor_Num_Wheel];   //!< �����ӵ���ظ��ļ�¼    
    Motor_StopMode MotorStop;
    u16 mCount;                             //!< number of motors
    u16 mCommStatus;                        //!< every motor communication status  bitx:0-disconnected,1-connection
    u16 mRdyStatus;                         //!< ÿ����������ϵ���Ƿ��Ѿ�׼����
}EwayWheelModule;



typedef enum
{
    Stop_Decelerate=0,//0  ����ֹͣ
    Stop_Brake =1,        //1�����ƶ�
    Stop_Free = 2          //2����ֹͣ
}EwayMotor_Stop_type;


#include "Wheel_Cmd.h"
#include "Wheel_Status.h"


s8 WheelBrake(EwayMotor_Stop_type eStopType);
s8 sysPcToWheelsCmdProcess(u16 Code,u32 timStamp,u16 dlen,u8* pdat);

#endif


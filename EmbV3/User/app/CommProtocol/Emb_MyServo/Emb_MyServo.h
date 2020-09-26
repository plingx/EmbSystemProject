/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Emb_MyServo.h
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
* 2017/12/20 | 0.0.1 | LingPing | Create file
*
*/
#ifndef _EMB_MYSERVO_H_
#define _EMB_MYSERVO_H_

#include "cmsis_os.h"
/************************************************************************************************/
#define MOTOR_SPEED_MAX_LIMIT_EN                                            1
#define MOTOR_SPEED_MIN_LIMIT_EN                                            0
#define MOTOR_POSITION_MAX_LIMIT_EN                                         1
#define MOTOR_POSITION_MIN_LIMIT_EN                                         1
#define MOTOR_POSITION_MAX                                                 4080    //!< ������λ��   
#define MOTOR_POSITION_MIN                                                 10    //!< �����Сλ��   
#define MOTOR_SPEED_MAX                                                    300   //1023    //!< �������ٶ�        RPM
#define SHOULDER_MOTOR_SPEED_MAX                                           1000   //1000    //!< �������ٶ�        RPM
#define MOTOR_DEFAULT_SPEED_MIN                                                    0x0001        //!< �����С�ٶ�        RPM
#define MOTOR_SPEED_COEFFICIENT                                             100
/***************************************************************************************************/
#define MORO2_ARM_TYPE  //!< MORO1_ARM_TYPE    MORO2_ARM_TYPE

/*   ���ID                     �ڴ����
�ֱ� 1 - 2 - 3 - 4 - 5 - 6      (0-5)
     7 - 8 - 9 - 10- 11- 12     (6-11)
�粿 13                         (12)
ͷ�� 14-15                      (13-14)
*/

/*
�������   ���ٱ�
A4           200
A3           360
A2           240
A1           229.8

*/

#define MOTOR_tA1       229.80f
#define MOTOR_tA2       240
#define MOTOR_tA3       360
#define MOTOR_tA4       200
#define MOTOR_tALift    5
#define MOTOR_tWhel     1








#ifdef  MORO2_ARM_TYPE
/* 0-1-6-7           A4
   2-3-8-9           A3
   4-5-10-11-13     A2
   14                A1
*/
#define MOTOR_REDUCTION_RATIO_1                                            200
#define MOTOR_REDUCTION_RATIO_2                                            200
#define MOTOR_REDUCTION_RATIO_3                                            360
#define MOTOR_REDUCTION_RATIO_4                                            360
#define MOTOR_REDUCTION_RATIO_5                                            240
#define MOTOR_REDUCTION_RATIO_6                                            240
#define MOTOR_REDUCTION_RATIO_7                                            200
#define MOTOR_REDUCTION_RATIO_8                                            200
#define MOTOR_REDUCTION_RATIO_9                                            360
#define MOTOR_REDUCTION_RATIO_10                                           360
#define MOTOR_REDUCTION_RATIO_11                                           240
#define MOTOR_REDUCTION_RATIO_12                                           240
#define MOTOR_REDUCTION_RATIO_13                                           5
#define MOTOR_REDUCTION_RATIO_14                                           240
#define MOTOR_REDUCTION_RATIO_15                                           229.8f
#endif


#ifdef  MORO1_ARM_TYPE

/*                                         ���ٱ�
    0-1-2-3-6-7-8-9            A3            360
       4-5-10-11-13-14            A2            240
       12                        A2            5
*/
#define MOTOR_REDUCTION_RATIO_1                                            360
#define MOTOR_REDUCTION_RATIO_2                                            360
#define MOTOR_REDUCTION_RATIO_3                                            360
#define MOTOR_REDUCTION_RATIO_4                                            360
#define MOTOR_REDUCTION_RATIO_5                                            240
#define MOTOR_REDUCTION_RATIO_6                                            240
#define MOTOR_REDUCTION_RATIO_7                                            360
#define MOTOR_REDUCTION_RATIO_8                                            360
#define MOTOR_REDUCTION_RATIO_9                                            360
#define MOTOR_REDUCTION_RATIO_10                                           360
#define MOTOR_REDUCTION_RATIO_11                                           240
#define MOTOR_REDUCTION_RATIO_12                                           240
#define MOTOR_REDUCTION_RATIO_13                                           5
#define MOTOR_REDUCTION_RATIO_14                                           240
#define MOTOR_REDUCTION_RATIO_15                                           229.8f//240    ��ǰ���Ի���ʹ�õ���229.8�ĵ�����������޸ġ�
#endif

/**************************************************************************************************/





#define COMM_SERVO_MSG_HEADER                                              0xFFFF
#define COMM_SERVO_RECVD_MSG_LENGTH_MINI                                   0x08    //!< recvd servo msg minimum length

/****************************************************************************************************
    Instruction
 ****************************************************************************************************/
//!< PING,���߼��ʶ���Ƿ����ý��տ���ָ��׼��
#define  MY_SERVO_Ping_H                                                   0x01
//!< PING,֡��
#define  MY_SERVO_PingLength_H                                             0x07
//!< ���Ĵ���;������ָ��
#define  MY_SERVO_Read_H                                                   0x02
//!< д�Ĵ���,д����ָ��յ�����֤��������ִ��
#define  MY_SERVO_Write_H                                                  0x03
//!< ͬ���㲥��,ͬʱ������Ĵ������Ͷ�ָ��(���������ղ�ѯָ��ָ��ʱ���Ӧ)
#define  MY_SERVO_BROADCAST_READ                                           0x04
//!< ͬ���㲥д,ͬʱ��������������Ϳ���ָ��(����������Ӧ)
#define  MY_SERVO_BROADCAST_WRITE                                          0x05
//!< �趨��λ,�趨��ǰλ��Ϊ��λ
#define  MY_SERVO_SetZero_H                                                0xE0
//!< ����,�����������λ����
#define  MY_SERVO_Reborn_H                                                 0xE1
//!< �ָ���������
#define  MY_SERVO_SetInit_H                                                0xE2
/****************************************************************************************************
    ID
 ****************************************************************************************************/
//!< �㲥ָ��ID
#define  BROADCAST_ID                                                      0xFE

//!< �ֱ۶��ID���1-12�������13��,ͷ��������14-15,����16-17
#define  Emb_StartID_ArmL                                                  0x01
#define  Emb_StartID_ArmR                                                  0x07
#define  Emb_StartID_Shoulder                                              0x0D
#define  Emb_StartID_Head                                                  0x0E
#define  Emb_StartID_Gripper                                               0x01
#define  Emb_StartID_Wheel                                                 0x01

#define  Motor_Num_ArmLeft                                                 6
#define  Motor_Num_ArmRight                                                6
#define  Motor_Num_Arm                                                    (Motor_Num_ArmLeft+Motor_Num_ArmRight)
#define  Motor_Num_Shoulder                                                1
#define  Motor_Num_Head                                                    2
#define  Motor_Num_Gripper                                                 2
#define  Motor_Num_Wheel                                                   2

//!< Ϊ������ID��洢�ռ��ţ��ض���Offset
#define Emb_ID_Offset_ARM                                                  0x01
#define Emb_ID_Offset_SHOULDER                                             0x0D   //13
#define Emb_ID_Offset_HEAD                                                 0x0E   //14
#define Emb_ID_Offset_GRIPPER                                              0x01
#define Emb_ID_Offset_WHEEL                                                0x01   //!< 0x10   //16






#define  PIDCMD_Length                                                     0x03
#define  TorqueSwitchCMD_Length                                            0x01
#define  MotorLimitCMD_Length                                              0x02 

#define  MyServo_Start_Position                                            0x0800
#define  Shoulder_Start_Position                                           0
#define  Manipulation_Start_Position                                       0x0800


/*************Read******************/
#define SERVO_REG_READ_NUMS_MAX                                            22         //!< ������Ĵ���������󲻳���22��  reg6-reg
#define SERVO_REG_READ_START_ADDR                                          0x78
#define SERVO_REG_READ_NUM_HEAD_SHOULDER                                   19
#define SERVO_REG_READ_NUM_ARM                                             15
#define SERVO_REG_READ_NUM_WHEEL                                           21




#define  ManipulationReadStartAddr                                         0x21
#define  ManipulationReadNum                                               4

#define  ShoulderReadStartAddr                                             0x21
#define  ShoulderReadNum                                                   7

//!< ����ϵͳ������ͨ�õ����Ŀ
#define SYS_MAX_GENERAL_SERVO_NUMS           (Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head+Motor_Num_Wheel)
#define GENERAL_SERVO_CFG_REGS                                              30          //!< ��������������üĴ������� reg:6-26
/***********************************/
#define Write_Regs_Record_Max_Space                                         20          //!< Arms &Head(14)    tStamp(4) + ID(2) + DnPos(2) + DnSpd(2) + UpPos(2) + UpSpd(2)
                                                                                        //!< Shld&Wheel(20)  tStamp(4) + ID(2) + InsTyp(1)+ DnPos(4) + DnSpd(2) + UpPos(4) + UpSpd(2)
#define SingleCyDataUnit_ArmHead                                            10          //!< ��ȥʱ����ĳ��� ID(2) + DnPos(2) + DnSpd(2) + UpPos(2) + UpSpd(2)
#define SingleCyDataUnit_ShlWhel                                            15          //!< ��ȥʱ����ĳ��� ID(2) + InsTyp(1)+ DnPos(4) + DnSpd(2) + UpPos(4) + UpSpd(2)





//��˾���ƵĿ��������Ĵ����������¡�����ͷ/��/�ֱ�/���Ӷ��õ������¿�������ֻ��צ�ӻ����ó��̵Ŀ�������

#define EWAY_MOTOR_CONTROL_MODE_SPEED                                      0x00
#define EWAY_MOTOR_CONTROL_MODE_POSITION                                   0x01
#define EWAY_MOTOR_CONTROL_MODE_TORQUE                                     0x02
#define EWAY_MOTOR_CONTROL_MODE_DUTYCYCLE                                  0x03

#define StatusLen_LeftArm                                                  0x8A
#define StatusLen_RightArm                                                 0x8A
#define StatusLen_Head_Shoulder                                            0x51
#define StatusLen_Wheel                                                    0x3A
#define StatusLen_Gripper                                                  0x21
#define StatusLen_PSU                                                      0x15
#define StatusLen_PSM                                                      0x27



//�Ĵ�����ַ��HEX��    ����    ����    ��д    ȡֵ��Χ    ��ʼֵ    ��ע
//0            �豸�ͺ�������    �豸�ͺţ�������    R    0-255    1(0X01)    ��Ҫ��˾�ڲ�����Ӳ����������������λ���������������������
//1//    �豸�ͺŴη���    �豸�ͺţ��η���    R    0-255    1(0X01)    ��Ҫ��ͬһ�����£���С���ڲ�ϸ��
//2            �̼��汾���汾��    �̼��汾�����汾�ţ�    R    0-255    1(0X01)    �̼����屾������Ӳ���汾��Ӧ
////3    �̼��汾���屾��    �̼��汾�����屾�ţ�    R    0-255    0(0x00)    ͬ���屾�²�ͬ����汾
////4    ��ַ��    ��վID    RW    1-253    1(0x01)    
////5    ������    ͨ�Ų�����    RW    ���·������ʱ�    10��1152000��    Ĭ��У�鷽ʽ8N1
////6    Ӧ���ӳ�ʱ��    ���յ���Ч��Ϣ�������ӳ�ʱ��    RW    0-255    250(0XFA)    �ӳ�ʱ�����ȡֵ��2��΢�룬����ȡֵ10��������ӳ�20us
#define  EwayMotor_AppMode      0x7//7    ���Ӧ��ģʽ    �ؽ�ģʽ���߳���ģʽ    RW    0-1    0    0����ؽ�ģʽ��1������ģʽ
#define  EwayMotor_CtrlMode     0x8//8    �������ģʽ    �������ģʽ���ٶȣ�λ�ú�ת�ؿ���    RW    0-3    1(0X01)    0�����ٶȿ���ģʽ��1����λ�ÿ���ģʽ��2����ת�أ������������ģʽ��3����ռ�ձ�ģʽ
#define  EwayMotor_PosLoopMode  0x9//9    λ�ñջ���������    λ�ñջ����־���λ�ÿ��ƻ������λ��    RW    0-1    0    0�������λ�ã�1�������λ�ã�����ģʽ��Ч
//A    ���������    ��ˢ���������    RW    0-255    6    
//B    ���ٻ����ٱ�L    ���ٻ����ٱȵ��ֽ�    RW    0-65535    2400(0XF0)    ��ֵ�Ŵ�100��
//C    ���ٻ����ٱ�H    ���ٻ����ٱȸ��ֽ�                
//D    ��С�Ƕ�ֵL        RW    0-4095    0    �ؽ�ģʽ��Ч
//E    ��С�Ƕ�ֵH                    
//F    ���Ƕ�ֵL        RW    0-4095    4095    �ؽ�ģʽ��Ч
//10    ���Ƕ�ֵH                    
//11    ��������ѹ        RW    8-60    32    ��λV
//12    ��������ѹ        RW    8-60    19    ��λV
//13    ����¶�        RW    20-120    80    ��λ���϶�
//14    ��������        RW    0-50    15    ��ֵ�Ŵ���10������λ0.1A
//15    �������ص���        RW    0-80    25    ��ֵ�Ŵ���10������λ0.1A
//16    �������ת��        RW    0-50    45    ��ֵ�Ŵ���10������λ0.1N.m
//17    λ�ñջ������������        RW    0-255    0    
//18    �ƶ�����        RW    0-30    20    ��ֵ�Ŵ���10������λ0.1A
//19    �ػ�����        RW    0-255    127    ���ֹͣ�����bit0�����ȣ�bit1��������bit2�����أ�bit3����ѹ��bit4��Ƿѹ��bit5���Ƕȳ�����Χ��bit6��Ӳ������
//1A    LED����        RW    0-255    127    LED��2HZ��˸��bit0�����ȣ�bit1��������bit2�����أ�bit3����ѹ��bit4��Ƿѹ��bit5���Ƕȳ�����Χ��bit6��Ӳ������
//1B    �ٶȿ���KP-L    �ٶ�PID�ջ�����������ֽ�    RW    0-9999        ��ֵ�Ŵ���10000��
//1C    �ٶȿ���KP-H    �ٶ�PID�ջ�����������ֽ�    RW            
//1D    �ٶȿ���KI-L    �ٶ�PID�ջ�����������ֽ�    RW    0-9999        ��ֵ�Ŵ���10000��
//1E    �ٶȿ���KI-H    �ٶ�PID�ջ�����������ֽ�    RW            
//1F    �ٶȿ���KD-L    �ٶ�PID�ջ�΢��������ֽ�    RW    0-9999        ��ֵ�Ŵ���10000��
//20    �ٶȿ���KD-H    �ٶ�PID�ջ�΢��������ֽ�    RW            
#define EwayMotor_PosCtrl_KP_L   0x21    //21    λ�ÿ���KP-L    λ��PID�ջ�����������ֽ�    RW    0-9999        ��ֵ�Ŵ���10000��
//22    λ�ÿ���KP-H    λ��PID�ջ�����������ֽ�    RW            
//23    λ�ÿ���KI-L    λ��PID�ջ�����������ֽ�    RW    0-9999        ��ֵ�Ŵ���10000��
//24    λ�ÿ���KI-H    λ��PID�ջ�����������ֽ�    RW            
//25    λ�ÿ���KD-L    λ��PID�ջ�΢��������ֽ�    RW    0-9999        ��ֵ�Ŵ���10000��
//26    λ�ÿ���KD-H    λ��PID�ջ�΢��������ֽ�    RW            
//27    ��������KP-L    ����PID�ջ�����������ֽ�    RW    0-9999        ��ֵ�Ŵ���10000��
//28    ��������KP-H    ����PID�ջ�����������ֽ�    RW            
//29    ��������KI-L    ����PID�ջ�����������ֽ�    RW    0-9999        ��ֵ�Ŵ���10000��
//2A    ��������KI-H    ����PID�ջ�����������ֽ�    RW            
//2B    ��������KD-L    ����PID�ջ�΢��������ֽ�    RW    0-9999        ��ֵ�Ŵ���10000��
//2C    ��������KD-H    ����PID�ջ�΢��������ֽ�    RW            
                        
//3C    �������ѧϰ״̬        RW    0-1    0    0����δѧϰ��1������ѧϰ��ֻ��ͨ���������ѧϰ���ܽ�δѧϰ״̬��Ϊѧϰ״̬
//3D    �����������1        RW    1-6    0    0������δѧϰ
//3E    �����������2        RW    1-6    0    0������δѧϰ
//3F    �����������3        RW    1-6    0    0������δѧϰ
//40    �����������4        RW    1-6    0    0������δѧϰ
//41    �����������5        RW    1-6    0    0������δѧϰ
//42    �����������6        RW    1-6    0    0������δѧϰ
                        
//50    ת�ؿ���        RW    0-1    0    0����ָ�ת�ر��֣�1�������ת��
#define EwayMotor_Stop   0x51//51    ֹͣ        RW    0-2    0    0�������ֹͣ��1��������ƶ���2��������ֹͣ
//52    �趨ռ�ձ�L        RW    -10000~10000    0    �Ŵ���100������λ%
//53    �趨ռ�ձ�H                    
//54    �趨���ٶ�L    ���ת�ټ��ٶ�    RW    0-65535    800    ��ֵ�Ŵ���10������λΪתÿ�루��Ե���������ת�٣�
//55    �趨���ٶ�H                    
#define EwayMotorReg_TargSpd_L        0x56            //56    �趨Ŀ���ٶȣ����ת�٣�L    δ����������ٵĵ��ת��    RW    -32768~32767    0    ��λΪRPM
#define EwayMotorReg_TargSpd_H        0x57            //57    �趨Ŀ���ٶȣ����ת�٣�H                    
#define EwayMotorReg_JointTargPos_L   0x58            //58    �趨Ŀ��λ��L-�ؽ�ģʽ    �ű�����λ��    RW    0-4095    0    0-4095��Ӧ0-360��
#define EwayMotorReg_JointTargPos_H   0x59            //59    �趨Ŀ��λ��H-�ؽ�ģʽ                    
#define EwayMotorReg_WheelTargPos_LL  0x5A            //5A    �趨Ŀ��λ��LL-����ģʽ        RW    -2147483648-2147483647    0    ����һ��һ��λ��
#define EwayMotorReg_WheelTargPos_L   0x5B            //5B    �趨Ŀ��λ��L-����ģʽ                    
#define EwayMotorReg_WheelTargPos_H   0x5C            //5C    �趨Ŀ��λ��H-����ģʽ                    
#define EwayMotorReg_WheelTargPos_HH  0x5D            //5D    �趨Ŀ��λ��HH-����ģʽ                    
//5E    �趨Ŀ��ת��/����L    ����    RW            
//5F    �趨Ŀ��ת��/����H                    
                        
//78    ״̬1        R    0-255    0    bit0�����ȣ�bit1��������bit2�����أ�bit3����ѹ��bit4��Ƿѹ��bit5���Ƕȳ�����Χ��bit6����������bit7������������
//79    ״̬2        R    0-255    0    bit0���Ƕȼ�����bit1���޷�����Ŀ���ٶȣ�bit2����ת��bit7��δ�������
//7A    ʵʱռ�ձ�L        R    -10000-10000    0    �Ŵ���100������λ%
//7B    ʵʱռ�ձ�H                    
//7C    �����ѹL        R    0-6000    0    ��ѹV����ֵ�Ŵ���100������λ0.01V
//7D    �����ѹH                    
//7E    �������L        R    -2000 -  2000    0    ����A����ֵ�Ŵ���100������λ0.01A
//7F    �������H                    
//80    �������¶�        R    0-120    25    
//81    ʵʱ�ٶȣ����ת�٣�L        R    -32768~32767    0    ��λΪRPM������ǰת��
//82    ʵʱ�ٶȣ����ת�٣�H                    
//83    ʵʱת�٣������������ת�٣�L        R    -32768-32767    0    ����������ٺ������ת�٣������ת�ٷŴ���100������λ0.01RPM
//84    ʵʱת�٣������������ת�٣�H                    
//85    ��ǰλ��L-�ؽ�ģʽ        R    0-4095    0    0-4095��Ӧ0-360��
//86    ��ǰλ��H-�ؽ�ģʽ                    
//87    ��ǰλ��LL-����ģʽ        R    -2147483648-2147483647    0    ����һ��һ��λ��
//88    ��ǰλ��L-����ģʽ                    
//89    ��ǰλ��H-����ģʽ                    
//8A    ��ǰλ��HH-����ģʽ                    
//8B    ��ǰת��L    ����    R            
//8C    ��ǰת��H                    
                        
//B4    U���ѹL        R    0-6000    0    ��ֵ�Ŵ���100������λ0.01V
//B5    U���ѹH                    
//B6    V���ѹL        R    0-6000    0    ��ֵ�Ŵ���100������λ0.01V
//B7    V���ѹH                    
//B8    W���ѹL        R    0-6000    0    ��ֵ�Ŵ���100������λ0.01V
//B9    W���ѹH                    
//BA    U�����L        R    -2000 -  2000    0    ��ֵ�Ŵ���100������λ0.01A
//BB    U�����H                    
//BC    V�����L        R    -2000 -  2000    0    ��ֵ�Ŵ���100������λ0.01A
//BD    V�����H                    
//BE    W�����L        R    -2000 -  2000    0    ��ֵ�Ŵ���100������λ0.01A
//BF    W�����H                    
#define EwayMotorReg_WheelReady     0xC0            //0xC0    �����챵���Ƿ��Ѿ�׼����           


//!< reg0x51:
#define EwayMotor_Stop_Off          0x00     //!< ȡ�� ����/����/���� ֹͣ
#define EwayMotor_Stop_SlowDown     0x01     //!< ����ֹͣ
#define EwayMotor_Stop_UrgentStop   0x02     //!< ����ֹͣ
#define EwayMotor_Stop_FreeStop     0x03     //!< ����ֹͣ

#define EwayMotor_CtrlMode_Speed    0x01    //!< �ٶ�ָ���ʶ
#define EwayMotor_CtrlMode_Posit    0x02    //!< λ��ָ���ʶ


typedef enum{
    StopMode_SlowDnOn = 44,
    StopMode_UrgentOn = 55,
    StopMode_FreeOn  = 66,
    StopMode_Off = 77
}Motor_StopMode;


typedef enum{
    Ctrl_Mode_Posit  = 00,       //!< �������ģʽ������λ��ģʽ��
    Ctrl_Mode_Speed  = 11,       //!< �������ģʽ�������ٶ�ģʽ��
    Ctrl_Mode_Torque = 22,       //!< �������ģʽ������ת��(����)ģʽ��
    Ctrl_Mode_DutyCy = 33        //!< �������ģʽ������ռ�ձ�ģʽ��
}Motor_CtrlMode;

typedef enum{
    App_Mode_Joint  = 00,       //!< ���Ӧ��ģʽΪ�ؽ�ģʽ
    App_Mode_Wheel  = 11        //!< ���Ӧ��ģʽΪ����ģʽ
}Motor_ApplicationMode;


//reg:0x09 
typedef enum{
    Pos_ClosedLoop_Absolute = 00,   //!< λ�ñջ���������Ϊ����λ��
    Pos_ClosedLoop_Relative = 11    //!< λ�ñջ���������Ϊ���λ��
}Motor_PosClosedLoopCtrlMode;


//reg:0x0B+(0C<<8)          ���ٱ�  Reduction Ratio
typedef enum{
    tA1 = 0,            //!< 229.8
    tA2,                //!< 240
    tA3,                //!< 360
    tA4,                //!< 200
    tALift,             //!< 5          �粿��������ļ��ٱ�
    tWhel               //!< 1          ���Ӽ��ٱ� 1 
}Motor_ReductionRatioMode;



typedef enum{
    EWAYBOT_ARMS_LEFT = 0,              //!< ���
    EWAYBOT_ARMS_RIGHT,                 //!< �ұ�
    EWAYBOT_SHOULDER,                   //!< ���
    EWAYBOT_HEAD,                       //!< ͷ
    EWAYBOT_GRIPPERS,                   //!< צ��
    EWAYBOT_WHEEL,                      //!< ����
    EWAYBOT_PSM,                        //!< ��Դ��
    EWAYBOT_PSU,                        //!< ��Դ����
    EWAYBOT_EMB                         //!< Embϵͳ����
}EwayMotor_Device_Type;


typedef struct{
    u8 mQueryRecd[2];    //!< byte0: bit7-4 ����Query����      bit3-0 �յ� fCode=0x02 �Ļظ�����        byte1:�����ѯ������    Ŀǰ��δʹ��
	u8 mExeCmRecd[2];    //!< byte0: bit7-4 ����дָ�����     bit3-0 �յ� fCode=0x03 �Ļظ�����        byte1:�����·�������    0:�ٶ�ģʽ 1:λ��ģʽ    Ŀǰֻ���ٶ�λ��ģʽ�л�ʱʹ��
}MotorReRecord;        //!< �������ݰ��ļ�¼���յ��ظ��ļ�¼



/*
typedef struct{
    MotorCMDBuff* pToMotorCmd;         //!< pointer of shoulder cmd buffer    
    MotorReducRatio* pMotorRatio;      //!< 
    Motor_CtrlMode mCtrlMode;          //!< motor control mode
    Motor_ApplicationMode  mAppMode;   //!< motro application mode
    Motor_PosClosedLoopCtrlMode mPosClsedLoopCtrlMode;
    u16 posKeep;                         //!< ����λ��������Clearָ����Ƿ���Ҫλ�ñ��ֵı�־�������֣���Ϊ0
}NodeCtrlModule;
*/



typedef struct{
    u8 uStaL;        //!< ״̬1,2
    u8 uStaH;
    u8 uinVolL;      //!< �����ѹL,H
    u8 uinVolH;
    u8 uinCurtL;     //!< �������L,H
    u8 uinCurtH;     
    u8 udevTmpt;     //!< �������¶�
    u8 uSpdRPML;     //!< ʵʱת��
    u8 uSpdRPMH;
    u8 uPosJointL;   //!< ��ǰλ�ã��ؽ�ģʽ
    u8 uPosJointH;
    s32 uPosWheel;   //!< ��ǰλ�ã�����ģʽ
    u8 uTorqueL;     //!< ��ǰת��L,H
    u8 uTorqueH;
}EWAYBOT_SERVO_STATUS;


typedef float MotorReducRatio;



s8 sysGeneralMotorRecvDataProcess(EwayMotor_Device_Type devType);
s8 sysGeneralMotorBroadcastRead(u8 unSlaveStartID,u8 unSlaveNums,u8 unRegStartAddr,u8 unRegNums,u8 *delayTime,EwayMotor_Device_Type devType);
s8 sysGeneralMotorBroadcastWrite(u8 unSlaveStartID,u8 unSlaveNums,u8 unRegStartAddr,u8 unRegNums,u8 *pdat,EwayMotor_Device_Type devType);
s8 sysGetMotorCommPort(EwayMotor_Device_Type devType,COM_PORT_E* pCom);
s8 sysGetMotorSendQueue(EwayMotor_Device_Type devType,QueueHandle_t* pQue);
s8 sysGeneralMotorDiscontinuousBroadcastWrite(u8* pSlaveID,u8 unSlaveNums,u8 unRegStartAddr,u8 unRegNums,u8 *pdat,EwayMotor_Device_Type devType);
s8 CheckRecvDataLen(EwayMotor_Device_Type devType , u8 uLen, u8 *pUData);
s8 sysGeneralMotorRecvRegsProcess(EwayMotor_Device_Type devType);
s8 g_GeneralMotorSingleRead(u8 unSlaveAddr,u8 regSegNums,u8* pInfo,EwayMotor_Device_Type devType);
void sysEmbMotorStatusCollect(void);
void sysEmbMotorSingleCycleCmdStatusClear(void);
s8 g_GeneralMotorSingleWrite(u8 unSlaveAddr,u8 infoLen,u8* pInfo,EwayMotor_Device_Type devType);
void sysEmbMotorStatusCollect(void);
s8 sysEmbCheckSendToJointMotorParameters(u16* pPos,s16* pSpd);
s8 sysEmbCheckSendToWheelMotorParameters(s16* pSpd);
s8 sysEmbCheckSendToShoulderMotorParameters(s32* pSpd);


#endif

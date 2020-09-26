/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Emb_MyServo.h
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
#ifndef _EMB_MYSERVO_H_
#define _EMB_MYSERVO_H_

#include "cmsis_os.h"
/************************************************************************************************/
#define MOTOR_SPEED_MAX_LIMIT_EN                                            1
#define MOTOR_SPEED_MIN_LIMIT_EN                                            0
#define MOTOR_POSITION_MAX_LIMIT_EN                                         1
#define MOTOR_POSITION_MIN_LIMIT_EN                                         1
#define MOTOR_POSITION_MAX                                                 4080    //!< 舵机最大位置   
#define MOTOR_POSITION_MIN                                                 10    //!< 舵机最小位置   
#define MOTOR_SPEED_MAX                                                    300   //1023    //!< 舵机最大速度        RPM
#define SHOULDER_MOTOR_SPEED_MAX                                           1000   //1000    //!< 舵机最大速度        RPM
#define MOTOR_DEFAULT_SPEED_MIN                                                    0x0001        //!< 舵机最小速度        RPM
#define MOTOR_SPEED_COEFFICIENT                                             100
/***************************************************************************************************/
#define MORO2_ARM_TYPE  //!< MORO1_ARM_TYPE    MORO2_ARM_TYPE

/*   电机ID                     内存序号
手臂 1 - 2 - 3 - 4 - 5 - 6      (0-5)
     7 - 8 - 9 - 10- 11- 12     (6-11)
肩部 13                         (12)
头部 14-15                      (13-14)
*/

/*
电机类型   减速比
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

/*                                         减速比
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
#define MOTOR_REDUCTION_RATIO_15                                           229.8f//240    当前测试机器使用的是229.8的电机，后续再修改。
#endif

/**************************************************************************************************/





#define COMM_SERVO_MSG_HEADER                                              0xFFFF
#define COMM_SERVO_RECVD_MSG_LENGTH_MINI                                   0x08    //!< recvd servo msg minimum length

/****************************************************************************************************
    Instruction
 ****************************************************************************************************/
//!< PING,在线监测识别，是否做好接收控制指令准备
#define  MY_SERVO_Ping_H                                                   0x01
//!< PING,帧长
#define  MY_SERVO_PingLength_H                                             0x07
//!< 读寄存器;读数据指令
#define  MY_SERVO_Read_H                                                   0x02
//!< 写寄存器,写数据指令，收到后验证无误立即执行
#define  MY_SERVO_Write_H                                                  0x03
//!< 同步广播读,同时给多个寄存器发送读指令(驱动器按照查询指令指定时间回应)
#define  MY_SERVO_BROADCAST_READ                                           0x04
//!< 同步广播写,同时给多个驱动器发送控制指令(驱动器不回应)
#define  MY_SERVO_BROADCAST_WRITE                                          0x05
//!< 设定零位,设定当前位置为零位
#define  MY_SERVO_SetZero_H                                                0xE0
//!< 重启,驱动器软件复位重启
#define  MY_SERVO_Reborn_H                                                 0xE1
//!< 恢复出厂设置
#define  MY_SERVO_SetInit_H                                                0xE2
/****************************************************************************************************
    ID
 ****************************************************************************************************/
//!< 广播指令ID
#define  BROADCAST_ID                                                      0xFE

//!< 手臂舵机ID编号1-12，肩膀电机13号,头部舵机编号14-15,轮子16-17
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

//!< 为适配电机ID与存储空间编号，特定义Offset
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
#define SERVO_REG_READ_NUMS_MAX                                            22         //!< 读电机寄存器个数最大不超过22个  reg6-reg
#define SERVO_REG_READ_START_ADDR                                          0x78
#define SERVO_REG_READ_NUM_HEAD_SHOULDER                                   19
#define SERVO_REG_READ_NUM_ARM                                             15
#define SERVO_REG_READ_NUM_WHEEL                                           21




#define  ManipulationReadStartAddr                                         0x21
#define  ManipulationReadNum                                               4

#define  ShoulderReadStartAddr                                             0x21
#define  ShoulderReadNum                                                   7

//!< 定义系统中最大的通用电机数目
#define SYS_MAX_GENERAL_SERVO_NUMS           (Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head+Motor_Num_Wheel)
#define GENERAL_SERVO_CFG_REGS                                              30          //!< 读各个电机的配置寄存器个数 reg:6-26
/***********************************/
#define Write_Regs_Record_Max_Space                                         20          //!< Arms &Head(14)    tStamp(4) + ID(2) + DnPos(2) + DnSpd(2) + UpPos(2) + UpSpd(2)
                                                                                        //!< Shld&Wheel(20)  tStamp(4) + ID(2) + InsTyp(1)+ DnPos(4) + DnSpd(2) + UpPos(4) + UpSpd(2)
#define SingleCyDataUnit_ArmHead                                            10          //!< 减去时间戳的长度 ID(2) + DnPos(2) + DnSpd(2) + UpPos(2) + UpSpd(2)
#define SingleCyDataUnit_ShlWhel                                            15          //!< 减去时间戳的长度 ID(2) + InsTyp(1)+ DnPos(4) + DnSpd(2) + UpPos(4) + UpSpd(2)





//我司自制的控制器，寄存器定义如下。现在头/肩/手臂/轮子都用的是如下控制器，只有爪子还在用厂商的控制器。

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



//寄存器地址（HEX）    名称    描述    读写    取值范围    初始值    备注
//0            设备型号主分类    设备型号，主分类    R    0-255    1(0X01)    主要公司内部各种硬件分类区别，诸如下位机、超声波、电机驱动等
//1//    设备型号次分类    设备型号，次分类    R    0-255    1(0X01)    主要是同一大类下，各小类内部细分
//2            固件版本主版本号    固件版本（主版本号）    R    0-255    1(0X01)    固件主板本尽量与硬件版本对应
////3    固件版本副板本号    固件版本（副板本号）    R    0-255    0(0x00)    同主板本下不同软件版本
////4    地址号    从站ID    RW    1-253    1(0x01)    
////5    波特率    通信波特率    RW    见下方波特率表    10（1152000）    默认校验方式8N1
////6    应答延迟时间    接收到有效信息后反馈的延迟时间    RW    0-255    250(0XFA)    延迟时间等于取值的2倍微秒，比如取值10，则代表延迟20us
#define  EwayMotor_AppMode      0x7//7    电机应用模式    关节模式或者车轮模式    RW    0-1    0    0代表关节模式，1代表车轮模式
#define  EwayMotor_CtrlMode     0x8//8    电机控制模式    电机控制模式：速度，位置和转矩控制    RW    0-3    1(0X01)    0代表速度控制模式，1代表位置控制模式，2代表转矩（或电流）控制模式，3代表占空比模式
#define  EwayMotor_PosLoopMode  0x9//9    位置闭环控制类型    位置闭环区分绝对位置控制还是相对位置    RW    0-1    0    0代表绝对位置，1代表相对位置，车轮模式有效
//A    电机极个数    无刷电机极个数    RW    0-255    6    
//B    减速机减速比L    减速机减速比低字节    RW    0-65535    2400(0XF0)    数值放大100倍
//C    减速机减速比H    减速机减速比高字节                
//D    最小角度值L        RW    0-4095    0    关节模式有效
//E    最小角度值H                    
//F    最大角度值L        RW    0-4095    4095    关节模式有效
//10    最大角度值H                    
//11    最高输入电压        RW    8-60    32    单位V
//12    最低输入电压        RW    8-60    19    单位V
//13    最高温度        RW    20-120    80    单位摄氏度
//14    电机额定电流        RW    0-50    15    数值放大了10倍，单位0.1A
//15    电机最大负载电流        RW    0-80    25    数值放大了10倍，单位0.1A
//16    最大允许转矩        RW    0-50    45    数值放大了10倍，单位0.1N.m
//17    位置闭环控制允许误差        RW    0-255    0    
//18    制动电流        RW    0-30    20    数值放大了10倍，单位0.1A
//19    关机报警        RW    0-255    127    电机停止输出，bit0：过热，bit1：过流，bit2：过载，bit3：过压，bit4：欠压，bit5：角度超出范围，bit6：硬件故障
//1A    LED报警        RW    0-255    127    LED按2HZ闪烁，bit0：过热，bit1：过流，bit2：过载，bit3：过压，bit4：欠压，bit5：角度超出范围，bit6：硬件故障
//1B    速度控制KP-L    速度PID闭环比例增益低字节    RW    0-9999        数值放大了10000倍
//1C    速度控制KP-H    速度PID闭环比例增益高字节    RW            
//1D    速度控制KI-L    速度PID闭环积分增益低字节    RW    0-9999        数值放大了10000倍
//1E    速度控制KI-H    速度PID闭环积分增益高字节    RW            
//1F    速度控制KD-L    速度PID闭环微分增益低字节    RW    0-9999        数值放大了10000倍
//20    速度控制KD-H    速度PID闭环微分增益高字节    RW            
#define EwayMotor_PosCtrl_KP_L   0x21    //21    位置控制KP-L    位置PID闭环比例增益低字节    RW    0-9999        数值放大了10000倍
//22    位置控制KP-H    位置PID闭环比例增益高字节    RW            
//23    位置控制KI-L    位置PID闭环积分增益低字节    RW    0-9999        数值放大了10000倍
//24    位置控制KI-H    位置PID闭环积分增益高字节    RW            
//25    位置控制KD-L    位置PID闭环微分增益低字节    RW    0-9999        数值放大了10000倍
//26    位置控制KD-H    位置PID闭环微分增益高字节    RW            
//27    电流控制KP-L    电流PID闭环比例增益低字节    RW    0-9999        数值放大了10000倍
//28    电流控制KP-H    电流PID闭环比例增益高字节    RW            
//29    电流控制KI-L    电流PID闭环积分增益低字节    RW    0-9999        数值放大了10000倍
//2A    电流控制KI-H    电流PID闭环积分增益高字节    RW            
//2B    电流控制KD-L    电流PID闭环微分增益低字节    RW    0-9999        数值放大了10000倍
//2C    电流控制KD-H    电流PID闭环微分增益高字节    RW            
                        
//3C    电机相序学习状态        RW    0-1    0    0代表未学习，1代表已学习，只有通过电机相序学习才能将未学习状态改为学习状态
//3D    电机相序数据1        RW    1-6    0    0代表尚未学习
//3E    电机相序数据2        RW    1-6    0    0代表尚未学习
//3F    电机相序数据3        RW    1-6    0    0代表尚未学习
//40    电机相序数据4        RW    1-6    0    0代表尚未学习
//41    电机相序数据5        RW    1-6    0    0代表尚未学习
//42    电机相序数据6        RW    1-6    0    0代表尚未学习
                        
//50    转矩开关        RW    0-1    0    0代表恢复转矩保持，1代表不输出转矩
#define EwayMotor_Stop   0x51//51    停止        RW    0-2    0    0代表减速停止，1代表紧急制动，2代表自由停止
//52    设定占空比L        RW    -10000~10000    0    放大了100倍，单位%
//53    设定占空比H                    
//54    设定加速度L    电机转速加速度    RW    0-65535    800    数值放大了10倍，单位为转每秒（针对电机本机输出转速）
//55    设定加速度H                    
#define EwayMotorReg_TargSpd_L        0x56            //56    设定目标速度（电机转速）L    未经减速箱减速的电机转速    RW    -32768~32767    0    单位为RPM
#define EwayMotorReg_TargSpd_H        0x57            //57    设定目标速度（电机转速）H                    
#define EwayMotorReg_JointTargPos_L   0x58            //58    设定目标位置L-关节模式    磁编码器位置    RW    0-4095    0    0-4095对应0-360度
#define EwayMotorReg_JointTargPos_H   0x59            //59    设定目标位置H-关节模式                    
#define EwayMotorReg_WheelTargPos_LL  0x5A            //5A    设定目标位置LL-车轮模式        RW    -2147483648-2147483647    0    换相一次一个位置
#define EwayMotorReg_WheelTargPos_L   0x5B            //5B    设定目标位置L-车轮模式                    
#define EwayMotorReg_WheelTargPos_H   0x5C            //5C    设定目标位置H-车轮模式                    
#define EwayMotorReg_WheelTargPos_HH  0x5D            //5D    设定目标位置HH-车轮模式                    
//5E    设定目标转矩/电流L    待定    RW            
//5F    设定目标转矩/电流H                    
                        
//78    状态1        R    0-255    0    bit0：过热，bit1：过流，bit2：过载，bit3：过压，bit4：欠压，bit5：角度超出范围，bit6：驱动错误，bit7：霍尔检测错误
//79    状态2        R    0-255    0    bit0：角度检测错误，bit1：无法到达目标速度，bit2：堵转，bit7：未定义错误
//7A    实时占空比L        R    -10000-10000    0    放大了100倍，单位%
//7B    实时占空比H                    
//7C    输入电压L        R    0-6000    0    电压V，数值放大了100倍，单位0.01V
//7D    输入电压H                    
//7E    输入电流L        R    -2000 -  2000    0    电流A，数值放大了100倍，单位0.01A
//7F    输入电流H                    
//80    驱动板温度        R    0-120    25    
//81    实时速度（电机转速）L        R    -32768~32767    0    单位为RPM，减速前转速
//82    实时速度（电机转速）H                    
//83    实时转速（减速箱输出轴转速）L        R    -32768-32767    0    经减速箱减速后输出轴转速，输出轴转速放大了100倍，单位0.01RPM
//84    实时转速（减速箱输出轴转速）H                    
//85    当前位置L-关节模式        R    0-4095    0    0-4095对应0-360度
//86    当前位置H-关节模式                    
//87    当前位置LL-车轮模式        R    -2147483648-2147483647    0    换相一次一个位置
//88    当前位置L-车轮模式                    
//89    当前位置H-车轮模式                    
//8A    当前位置HH-车轮模式                    
//8B    当前转矩L    待定    R            
//8C    当前转矩H                    
                        
//B4    U相电压L        R    0-6000    0    数值放大了100倍，单位0.01V
//B5    U相电压H                    
//B6    V相电压L        R    0-6000    0    数值放大了100倍，单位0.01V
//B7    V相电压H                    
//B8    W相电压L        R    0-6000    0    数值放大了100倍，单位0.01V
//B9    W相电压H                    
//BA    U相电流L        R    -2000 -  2000    0    数值放大了100倍，单位0.01A
//BB    U相电流H                    
//BC    V相电流L        R    -2000 -  2000    0    数值放大了100倍，单位0.01A
//BD    V相电流H                    
//BE    W相电流L        R    -2000 -  2000    0    数值放大了100倍，单位0.01A
//BF    W相电流H                    
#define EwayMotorReg_WheelReady     0xC0            //0xC0    检查轮毂电机是否已经准备好           


//!< reg0x51:
#define EwayMotor_Stop_Off          0x00     //!< 取消 减速/紧急/自由 停止
#define EwayMotor_Stop_SlowDown     0x01     //!< 减速停止
#define EwayMotor_Stop_UrgentStop   0x02     //!< 紧急停止
#define EwayMotor_Stop_FreeStop     0x03     //!< 自由停止

#define EwayMotor_CtrlMode_Speed    0x01    //!< 速度指令标识
#define EwayMotor_CtrlMode_Posit    0x02    //!< 位置指令标识


typedef enum{
    StopMode_SlowDnOn = 44,
    StopMode_UrgentOn = 55,
    StopMode_FreeOn  = 66,
    StopMode_Off = 77
}Motor_StopMode;


typedef enum{
    Ctrl_Mode_Posit  = 00,       //!< 电机控制模式工作在位置模式下
    Ctrl_Mode_Speed  = 11,       //!< 电机控制模式工作在速度模式下
    Ctrl_Mode_Torque = 22,       //!< 电机控制模式工作在转矩(电流)模式下
    Ctrl_Mode_DutyCy = 33        //!< 电机控制模式工作在占空比模式下
}Motor_CtrlMode;

typedef enum{
    App_Mode_Joint  = 00,       //!< 电机应用模式为关节模式
    App_Mode_Wheel  = 11        //!< 电机应用模式为车轮模式
}Motor_ApplicationMode;


//reg:0x09 
typedef enum{
    Pos_ClosedLoop_Absolute = 00,   //!< 位置闭环控制类型为绝对位置
    Pos_ClosedLoop_Relative = 11    //!< 位置闭环控制类型为相对位置
}Motor_PosClosedLoopCtrlMode;


//reg:0x0B+(0C<<8)          减速比  Reduction Ratio
typedef enum{
    tA1 = 0,            //!< 229.8
    tA2,                //!< 240
    tA3,                //!< 360
    tA4,                //!< 200
    tALift,             //!< 5          肩部升降电机的减速比
    tWhel               //!< 1          轮子减速比 1 
}Motor_ReductionRatioMode;



typedef enum{
    EWAYBOT_ARMS_LEFT = 0,              //!< 左臂
    EWAYBOT_ARMS_RIGHT,                 //!< 右臂
    EWAYBOT_SHOULDER,                   //!< 肩膀
    EWAYBOT_HEAD,                       //!< 头
    EWAYBOT_GRIPPERS,                   //!< 爪子
    EWAYBOT_WHEEL,                      //!< 轮子
    EWAYBOT_PSM,                        //!< 电源板
    EWAYBOT_PSU,                        //!< 电源充电板
    EWAYBOT_EMB                         //!< Emb系统本身
}EwayMotor_Device_Type;


typedef struct{
    u8 mQueryRecd[2];    //!< byte0: bit7-4 发送Query个数      bit3-0 收到 fCode=0x02 的回复个数        byte1:具体查询的内容    目前并未使用
	u8 mExeCmRecd[2];    //!< byte0: bit7-4 发送写指令个数     bit3-0 收到 fCode=0x03 的回复个数        byte1:具体下发的内容    0:速度模式 1:位置模式    目前只在速度位置模式切换时使用
}MotorReRecord;        //!< 发送数据包的记录与收到回复的记录



/*
typedef struct{
    MotorCMDBuff* pToMotorCmd;         //!< pointer of shoulder cmd buffer    
    MotorReducRatio* pMotorRatio;      //!< 
    Motor_CtrlMode mCtrlMode;          //!< motor control mode
    Motor_ApplicationMode  mAppMode;   //!< motro application mode
    Motor_PosClosedLoopCtrlMode mPosClsedLoopCtrlMode;
    u16 posKeep;                         //!< 当上位机发送完Clear指令后，是否需要位置保持的标志，若保持，则不为0
}NodeCtrlModule;
*/



typedef struct{
    u8 uStaL;        //!< 状态1,2
    u8 uStaH;
    u8 uinVolL;      //!< 输入电压L,H
    u8 uinVolH;
    u8 uinCurtL;     //!< 输入电流L,H
    u8 uinCurtH;     
    u8 udevTmpt;     //!< 驱动板温度
    u8 uSpdRPML;     //!< 实时转速
    u8 uSpdRPMH;
    u8 uPosJointL;   //!< 当前位置，关节模式
    u8 uPosJointH;
    s32 uPosWheel;   //!< 当前位置，车轮模式
    u8 uTorqueL;     //!< 当前转矩L,H
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

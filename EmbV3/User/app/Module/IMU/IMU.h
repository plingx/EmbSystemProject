/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file IMU.h
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
#ifndef _IMU_H_
#define _IMU_H_

#include "bsp.h"

#define EWAY_IMU_PARAM_NUMS   9             //!< 加速度Ax,Ay,Az,角速度Gx,Gy,Gz,角度Roll,Pitch,Yaw


#define Aclx_ADDR 0x34
#define Roll_ADDR 0x3D


#define RopeDisplacementSensorEn_MaskBit    0x01    //!< 是否使用了拉线传感器
#define UpLimitPinTouched_MaskBit           0x80    //!< 是否压了上下限位
#define DnLimitPinTouched_MaskBit           0x40


typedef struct{
    u16 resIMU;
    u8 sta;
}ImuCommModule;


typedef struct{
    u32 result;         //!< 实时数值结果
    u32 Res;            //!< 经过下位机处理的结果
}EwayRDSModule;

typedef struct{
    u8 En;              //!< 是否有拉线传感器
    u8 init;            //!< 是否标定 有拉线传感器时，标识是否标定，无拉线传感器时标识是否初始化EEPROM
    u32 LowLmt;         //!< 下限位位置，拉线传感器数值
    u32 HigLmt;         //!< 上限位位置，拉线传感器数值
    u32 Lmt_SwitchLen;  //!< 标称值  57900/39200
}EwayEmbSysInfoRDSModule;         //!< 存储于eeprom中的Rope Displacement Sensor参数


/*
JY901,i2c
0x34,0x35,0x36,0x3D,0x3E,0x3F
Ax    Ay   Az   Wx   Wy   Wz
Aclx Acly Aclz  R    P    Y
*/

typedef struct{ 
    s16 Res[EWAY_IMU_PARAM_NUMS];   //!< 获取的结果缓存    Roll,Pitch,Yaw,,Ax,Ay,Az,Wx,Wy,Wz
    ImuCommModule Comm;
}EwayIMUModule;

s8 sysGetImuData(EwayIMUModule* pModule);
s8 sysGetRopeDisplacementSensorData(EwayRDSModule* pModule);


#include "IMU_Status.h"


#endif

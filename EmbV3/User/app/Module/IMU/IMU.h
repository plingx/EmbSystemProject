/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file IMU.h
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
#ifndef _IMU_H_
#define _IMU_H_

#include "bsp.h"

#define EWAY_IMU_PARAM_NUMS   9             //!< ���ٶ�Ax,Ay,Az,���ٶ�Gx,Gy,Gz,�Ƕ�Roll,Pitch,Yaw


#define Aclx_ADDR 0x34
#define Roll_ADDR 0x3D


#define RopeDisplacementSensorEn_MaskBit    0x01    //!< �Ƿ�ʹ�������ߴ�����
#define UpLimitPinTouched_MaskBit           0x80    //!< �Ƿ�ѹ��������λ
#define DnLimitPinTouched_MaskBit           0x40


typedef struct{
    u16 resIMU;
    u8 sta;
}ImuCommModule;


typedef struct{
    u32 result;         //!< ʵʱ��ֵ���
    u32 Res;            //!< ������λ������Ľ��
}EwayRDSModule;

typedef struct{
    u8 En;              //!< �Ƿ������ߴ�����
    u8 init;            //!< �Ƿ�궨 �����ߴ�����ʱ����ʶ�Ƿ�궨�������ߴ�����ʱ��ʶ�Ƿ��ʼ��EEPROM
    u32 LowLmt;         //!< ����λλ�ã����ߴ�������ֵ
    u32 HigLmt;         //!< ����λλ�ã����ߴ�������ֵ
    u32 Lmt_SwitchLen;  //!< ���ֵ  57900/39200
}EwayEmbSysInfoRDSModule;         //!< �洢��eeprom�е�Rope Displacement Sensor����


/*
JY901,i2c
0x34,0x35,0x36,0x3D,0x3E,0x3F
Ax    Ay   Az   Wx   Wy   Wz
Aclx Acly Aclz  R    P    Y
*/

typedef struct{ 
    s16 Res[EWAY_IMU_PARAM_NUMS];   //!< ��ȡ�Ľ������    Roll,Pitch,Yaw,,Ax,Ay,Az,Wx,Wy,Wz
    ImuCommModule Comm;
}EwayIMUModule;

s8 sysGetImuData(EwayIMUModule* pModule);
s8 sysGetRopeDisplacementSensorData(EwayRDSModule* pModule);


#include "IMU_Status.h"


#endif

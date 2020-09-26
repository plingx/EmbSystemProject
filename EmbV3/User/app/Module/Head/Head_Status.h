/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Head_Status.h
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
#ifndef _APP_MODULE_HEADS_STATUS_H_
#define _APP_MODULE_HEADS_STATUS_H_
	

#include "bsp.h"

#define  HEAD_NUM 2
#define  SHOULDER_NUM 1
#define  EWAYSERVO_PAYLOAD_BYTES  15


typedef struct{
	u16 uJointPos;
	u16 uSpeed;
	u32 uStatus;
	u16 uVoltage;
	u16 uCurrent;
	u8 uTemperature;
}SingleHeadStatus;

typedef struct{
	SingleHeadStatus sSingleHeadStatus[HEAD_NUM];
}HeadsStatus;

typedef struct{
	u32 uWheelPos;  
	u16 uSpeed;
	u32 uStatus;
	u16 uVoltage;
	u16 uCurrent;
	u8 uTemperature;
}SingleShoulderStatus;

s8 SendQueryHeadStatus(void);
s8 GetHeadsStatus(HeadsStatus *pHeadsStatus);
s8 ReportHeadsStatusToPC(u8 *uData, u16 *uLen);

#endif


















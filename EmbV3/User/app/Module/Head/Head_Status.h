/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Head_Status.h
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
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


















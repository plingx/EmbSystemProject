#ifndef  _E_M_B_BATTERYCHARGER_
#define _E_M_B_BATTERYCHARGER_

#include "bsp.h"


///�����ư�֡ͷ
#define  BatteryHeader_H                                                0xEEEE//ͷ
/****************************************************************************************************
	Instruction
 ****************************************************************************************************/
///PING,���߼��ʶ���Ƿ����ý��տ���ָ��׼��
#define  BatteryPing_H                                                  0x01
///ping ֡��
#define  BatteryPingLength_H	                                          0x07
///���Ĵ���;������ָ��
#define  BatteryRead_H                                                  0x02
///��ذ�֡����
#define  BatteryReadLength_H	                                          0x07
///д�Ĵ���,д����ָ��յ�����֤��������ִ��
#define  BatteryWrite_H                                                 0x03
///�ָ���������
#define  BatterySetInit_H                                              0xE2
///���Ĵ���BUF
//extern u8 unRegAddr[2];
///���PING
s8 BatteryCharger_PING(u8 unSlaveAddr);
//���͵�س����ѯָ��
s8 SendQuery_BatteryCharger(u8 unSlaveAddr, u8 *unSendBuffer,u8 unLength);
///д��ؼĴ���
s8 BatteryCharger_WRITE(u8 unSlaveAddr, u8 *unSendBuffer, u16 unLength);
///��ذ��ʼ��
s8 BatteryCharger_SET_INITE(u8 unSlaveAddr);

//s8 SendQuery_BatteryCharger(u8 unSlaveAddr, u8 *unSendBuffer,u16 unLength);

#define CRC_LEN   2
#define BATTERY_HEADER_LEN  5

#endif


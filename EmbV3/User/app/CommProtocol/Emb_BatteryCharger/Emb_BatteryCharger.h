#ifndef  _E_M_B_BATTERYCHARGER_
#define _E_M_B_BATTERYCHARGER_

#include "bsp.h"


///充电控制板帧头
#define  BatteryHeader_H                                                0xEEEE//头
/****************************************************************************************************
	Instruction
 ****************************************************************************************************/
///PING,在线监测识别，是否做好接收控制指令准备
#define  BatteryPing_H                                                  0x01
///ping 帧长
#define  BatteryPingLength_H	                                          0x07
///读寄存器;读数据指令
#define  BatteryRead_H                                                  0x02
///电池板帧长度
#define  BatteryReadLength_H	                                          0x07
///写寄存器,写数据指令，收到后验证无误立即执行
#define  BatteryWrite_H                                                 0x03
///恢复出厂设置
#define  BatterySetInit_H                                              0xE2
///读寄存器BUF
//extern u8 unRegAddr[2];
///电池PING
s8 BatteryCharger_PING(u8 unSlaveAddr);
//发送电池充电板查询指令
s8 SendQuery_BatteryCharger(u8 unSlaveAddr, u8 *unSendBuffer,u8 unLength);
///写电池寄存器
s8 BatteryCharger_WRITE(u8 unSlaveAddr, u8 *unSendBuffer, u16 unLength);
///电池板初始化
s8 BatteryCharger_SET_INITE(u8 unSlaveAddr);

//s8 SendQuery_BatteryCharger(u8 unSlaveAddr, u8 *unSendBuffer,u16 unLength);

#define CRC_LEN   2
#define BATTERY_HEADER_LEN  5

#endif


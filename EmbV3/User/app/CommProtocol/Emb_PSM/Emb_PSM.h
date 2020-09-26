#ifndef _E_M_B_P_S_M_H
#define _E_M_B_P_S_M_H

//s8 Psm_Init();
//s8 Psm_Ping();
//s8 Psm_Read();

#include "stm32f4xx.h"

///PSM֡ͷ
#define  PSMHeader_H                                                0xDDDD
/****************************************************************************************************
	Instruction
 ****************************************************************************************************/
///PING,���߼��ʶ���Ƿ����ý��տ���ָ��׼��
#define  PSMPing_H                                                  0x01
///ping��֡��
#define  PSMPingLength_H	                                          0x07
///���Ĵ���;������ָ��
#define  PSMRead_H                                                  0x02
///����֡��
#define  PSMReadLength_H	                                          0x07
///д�Ĵ���,д����ָ��յ�����֤��������ִ��
#define  PSMWrite_H                                                 0x03
///����,�����������λ����
#define  PSMReborn_H                                                0xE1
///�ָ���������
#define  PSMSetInit_H                                               0xE2

#define CRC_LEN  2

#define PSM_HEADER_LEN  5

///PING����PSM
s8 PSM_PING(u8 unSlaveAddr);
///��PSM�Ĵ���
s8 SendQuery_PSM(u8 unSlaveAddr, u8 *unSendBuffer,u8 unLen);
///дPSM�Ĵ���
s8 PSM_WRITE(u8 unSlaveAddr,u8 unRegAddr,u16 unRegNum,u8 *unSendBuffer);
///PSM��ʼ��
s8 PSM_SET_INITE(u8 unSlaveAddr);
///���Ĵ���BUF
extern u8 unPSMRegAddr[2];


#endif

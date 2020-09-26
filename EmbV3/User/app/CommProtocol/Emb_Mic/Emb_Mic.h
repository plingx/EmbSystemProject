/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Emb_Mic.h
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2018-05-30
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/05/30 | 0.0.1 | LingPing | Create file
*
*/
#ifndef _EMB_MIC_H_
#define _EMB_MIC_H_

///MIC֡ͷ
#define  MICHeader_H                                                0xFFFF
#define  MIC_Start_ID                                               0x01


#define	MIC_ONLINE			1 //!< ����
#define	MIC_NOLINE			0 //!< δ����



/****************************************************************************************************
	Instruction
 ****************************************************************************************************/
///PING,���߼��ʶ���Ƿ����ý��տ���ָ��׼��
#define  MICPing_H                                                  0x01
///ping��֡��
#define  MICPingLength_H	                                        0x07
///���Ĵ���;������ָ��
#define  MICRead_H                                                  0x02
///����֡��
#define  MICReadLength_H	                                        0x07
///д�Ĵ���,д����ָ��յ�����֤��������ִ��
#define  MICWrite_H                                                 0x03


//======================Mic Register Macro define=============================
#define	GET_ONLINE_STATE	0x0E	//���mic����״̬(��Ϊ��ӼĴ���)
#define	GET_VERSION			0x0F	//��ѯ�汾������
#define	GET_WAKE_ANGLE		0x10	//��ѯ���ѽǶ�
#define	SET_RESET			0x11	//��λ
#define	SET_RECE_BS_ID		0x12	//����ʰ������(0-5)
#define	SET_WAKE_MODE		0x13    //���û���  0���رջ���  1:��������
#define	SET_CALL_MODE		0x14	//����ͨ��ģʽ
#define	GET_WAKEWORD_ID		0x15	//��ѯ���Ѵ�ID
#define	GET_WAKE_SCORE		0x16	//��ѯ���ѵ÷�




///PING����MIC
s8 Mic_Ping(u8 unSlaveAddr);
///��MIC�Ĵ���
s8 Mic_Read(u8 unSlaveAddr, u8 *unSendBuffer,u16 unLength);
///дMIC�Ĵ���
s8 Mic_Write(u8 unSlaveAddr,u8 unRegAddr,u16 unRegNum,u8 *unSendBuffer);


#endif 

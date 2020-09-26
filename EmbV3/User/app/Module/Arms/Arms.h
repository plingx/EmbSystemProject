/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Arms.h
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
* @author 
* @version 0.0.1
* @date 2018-01-25
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/01/25 | 0.0.1 | Ling       | Create file
*
*/
#ifndef _APP_MODULE_ARMS_H_
#define _APP_MODULE_ARMS_H_

//!< ��������
#define CMD_ARMS2MOTOR_INFO_LEN        80   //!< define the Emb send to shoulder motor cmd buffer length  
#define CMD_ARMS2MOTOR_QUEU_NUM        4    //!< ÿ�����ڿ����������͵����ָ����Ŀ�����Ը���ʵ����Ҫ�����޸ġ�





typedef struct{
	NodeCtrlModule mControl;           //!< motor control parameters
	NodeStateModule mState;
	u16 mCntLeft;                      //!< number of motors
	u16 mCntRight;
	u16 mCommStatus;                   //!< every motor communication status  bitx:0-disconnected,1-connection
}EwayArmModule;




#include "Arms_Cmd.h"
#include "Arms_Status.h"



s8 sysPcToArmsCmdProcess(u16 Code,u32 timStamp,u16 dlen,u8* pdat);
#endif

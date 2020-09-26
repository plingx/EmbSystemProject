/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Head.h
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
#ifndef _APP_MODULE_HEADS_H_
#define _APP_MODULE_HEADS_H_


                                            //!< �㲥д6��reg��ָ�����Ϊ19���ֽ�
#define CMD_HEAD2MOTOR_INFO_LEN        24   //!< define the Emb send to head motor cmd buffer length   2*(6+1)+23 = 24
#define CMD_HEAD2MOTOR_QUEU_NUM        4    //!< ÿ�����ڿ����������͵����ָ����Ŀ�����Ը���ʵ����Ҫ�����޸ġ�



typedef struct{
	NodeCtrlModule mControl;           //!< motor control parameters
	NodeStateModule mState;            //<  motor state: registers and others
	u16 mCount;                        //!< number of motors
	u16 mCommStatus;                   //!< every motor communication status  bitx:0-disconnected,1-connection	
}EwayHeadModule;


#include "Head_Cmd.h"
#include "Head_Status.h"



s8 sysPcToHeadsCmdProcess(u16 Code,u32 timStamp,u16 dlen,u8* pdat);




#endif

/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Head.h
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
#ifndef _APP_MODULE_HEADS_H_
#define _APP_MODULE_HEADS_H_


                                            //!< 广播写6个reg的指令最长，为19个字节
#define CMD_HEAD2MOTOR_INFO_LEN        24   //!< define the Emb send to head motor cmd buffer length   2*(6+1)+23 = 24
#define CMD_HEAD2MOTOR_QUEU_NUM        4    //!< 每个周期可以向电机发送的最大指令数目，可以根据实际需要进行修改。



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

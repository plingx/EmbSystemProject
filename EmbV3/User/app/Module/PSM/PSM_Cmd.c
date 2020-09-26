/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file PSM_Cmd.c
* @brief 本文件实现下位机对电源开关板的一些命令操作
* @details   
* @author chengyi@ewaybot.com
* @version 0.0.1
* @date 2018-1-24
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/1/24 | 0.0.1 | chengyi | Create file
*
*/

#include "includes.h"



extern EwayPSMModule EwayPSM;

/* --------------------------------------------------------------------------*/
/**
* @name 
* @brief 
* @details 
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/

s8 SetPSMCmdTcpErr(void)
{
    u8 uTcpErr[2]={0x02,0x01}; //TCP掉线
    s8 res;

    res = PSM_WRITE(PSM_SlaveAddr,PSM_CtrlSwitchOnOff,2,uTcpErr);

    if(res == ERR_NONE)
    {
        EwayPSM.reRecord.sExeCmRecd += 1;
    }

    return res;
}


s8 SetPSMCmdTcpReConnect(void)
{
    u8 uTcpReConnect=0x0;
    s8 res;

    res = PSM_WRITE(PSM_SlaveAddr,PSM_CtrlSwitchOnOff,1,&uTcpReConnect);

    if(res == ERR_NONE)
    {
        EwayPSM.reRecord.sExeCmRecd += 1;
    }
    
    return res;
}

s8 SetPSMCmdLedCtrl(LedCtrlType eLedCtrl)
{
    u8 uLedCtrl=(u8)eLedCtrl;
    s8 res;

    res = PSM_WRITE(PSM_SlaveAddr,PSM_LEDCtrl,1,&uLedCtrl);

    if(res == ERR_NONE)
    {
        EwayPSM.reRecord.sExeCmRecd += 1;
    }

    return res;
}

s8 SetPSMCmdSwitchCtrl(u8 uSwitchCtrl)
{
    s8 res;
    res = PSM_WRITE(PSM_SlaveAddr,PSM_CtrlSwitchOnOff,1,&uSwitchCtrl);

    if(res == ERR_NONE)
    {
        EwayPSM.reRecord.sExeCmRecd += 1;
    }
    
    return res;
}

s8 SetPSMCmdPowerOff(void)
{
    u8 uPowerOff = 0x80; //文档代码不符，后续调试时核实
    s8 res;

    res = PSM_WRITE(PSM_SlaveAddr,PSM_PowerOffCmd,1,&uPowerOff);

    if(res == ERR_NONE)
    {
        EwayPSM.reRecord.sExeCmRecd += 1;
    }

    return res;
}

//单位为s
s8 SetPSMCmdPowerOffDelay(u8 uDelayTime)  
{
    s8 res;
    
    res = PSM_WRITE(PSM_SlaveAddr,PSM_PowerOffDelay,1,&uDelayTime);

    if(res == ERR_NONE)
    {
        EwayPSM.reRecord.sExeCmRecd += 1;
    }

    return res;
}



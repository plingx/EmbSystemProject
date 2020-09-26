/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file PSM_Cmd.c
* @brief ���ļ�ʵ����λ���Ե�Դ���ذ��һЩ�������
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
    u8 uTcpErr[2]={0x02,0x01}; //TCP����
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
    u8 uPowerOff = 0x80; //�ĵ����벻������������ʱ��ʵ
    s8 res;

    res = PSM_WRITE(PSM_SlaveAddr,PSM_PowerOffCmd,1,&uPowerOff);

    if(res == ERR_NONE)
    {
        EwayPSM.reRecord.sExeCmRecd += 1;
    }

    return res;
}

//��λΪs
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



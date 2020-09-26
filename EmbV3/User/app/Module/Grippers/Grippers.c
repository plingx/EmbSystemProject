/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Grippers.c
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
* @author 
* @version 0.0.1
* @date 2018-01-29
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/01/29 | 0.0.1 | Ling     | Create file
*
*/
#include "includes.h"


extern osMutexId PCCmdBuffOpHandleGrp[Motor_Num_Gripper];
extern EwayEmbSysDebugModule* pDebug;



MotorSTATEBuff GrippersReg[Motor_Num_Gripper]={0};
MotorRecordBuff GripperRecord[Motor_Num_Gripper]={0};
MotorCMDBuff GrippersCmd[Motor_Num_Gripper]={    
                                             {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleGrp[0]},
                                             {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleGrp[1]}
};


Motor_LastPosCmd GrippersLstPos[Motor_Num_Gripper]={0};

EwayGripperModule EwayGrippers={
    {&GrippersCmd[0],0,Ctrl_Mode_Posit,App_Mode_Joint,Pos_ClosedLoop_Absolute,GrippersLstPos,0},
    {&GrippersReg[0],&GripperRecord[0],NULL},
    {0x0000,0x0000},
    Motor_Num_Gripper,
    0x0000
};


s8 g_ProcessGripperTorqueSwitchCmd(u16 dlen,u8* pdat)
{
    if((dlen!=(PCCmd_GripperTorqueSw_Unit<<1))||(pdat==NULL))       //!< ת�ؿ���ֻ֧��2ֻצ��һ���޸ģ����޸���������ͬ�ģ��������
    {
        return ERR_NONE;
    }   

    //!< ���צ��ID
    if(((pdat[0]+(pdat[1]<<8))!=Emb_StartID_Gripper)||((pdat[3]+(pdat[4]<<8))!=(Emb_StartID_Gripper+1)))
    {
        return ERR_NONE;
    }

    //!< ���צ�������Ƿ���ͬ
    if(pdat[2]!=pdat[5])
    {
        return ERR_NONE;
    }

    //!< ����צ��ת�ؿ��ز��л����ģʽ
    if(pdat[2]==0)
    {
        EwayGrippers.mControl.mCtrlMode = Ctrl_Mode_Posit;
    }
    else
    {
        EwayGrippers.mControl.mCtrlMode = Ctrl_Mode_Torque;
    }

    return ERR_NONE;
}

s8 g_ProcessGripperTorqueLimitCmd(u16 dlen,u8* pdat)
{
    u16 iD,lmt; 
    u8 i;
    u8 Unit=PCCmd_GripperTorqueLm_Unit;
    
    //!< ������ݳ���
    if(dlen<Unit)
    {
        return ERR_NONE;
    }

    i=0;
            
    while(i<dlen)
    {
        iD = pdat[i] + (pdat[i+1]<<8);
                
        if((iD < Emb_StartID_Gripper)||(iD >= (Emb_StartID_Gripper + Motor_Num_Gripper)))       //!< ���צ��ID
        {
            i += Unit;
        }
        else
        {
                lmt = pdat[i+2] + (pdat[i+3]<<8);

                if(lmt > Dynamixel_Motor_Torque_Max)
                {
                    lmt = Dynamixel_Motor_Torque_Max;
                }

                EwayGrippers.mTorqueLmt[iD-1] = lmt;

                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_XQueue_2_XCmdBuf_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Gripper_MASK))
                {
                    Bsp_printf("Grip set Torque limit value suc,id:%d,limit:%d",iD,lmt);
                }

                i += Unit;
            }
        }

    return ERR_NONE;
}


s8 sysPcToGrippersCmdProcess(u16 Code,u32 timStamp,u16 dlen,u8* pdat)
{
    u8 i;
    s8 res=ERR_NONE;
    u8 Unit=0;
    u16 iD;
    u8 dat[20]={0};
            
        //!< ���ָ���� 
        switch(Code)
        {
            case PCCmd_GripperMove:
                        Unit = PCCmd_GripperMove_Unit;                      //!< id(2bytes) + position(2bytes) + speed(2bytes)
                                
                        if(dlen<Unit) 
                            res = ERR_INPUT_PARAMETERS;
                    
                break;
        /*
            case PCCmd_GripperPID:
                break;*/
            case PCCmd_GripperTorqueSwitch:
                
                g_ProcessGripperTorqueSwitchCmd(dlen,pdat);
                
                break;
            case PCCmd_GripperMotorLimit:
                
                g_ProcessGripperTorqueLimitCmd(dlen,pdat);
                
                break;
        
            default:
                    res = ERR_INPUT_PARAMETERS;
                    break;
        } 

        if((Code == PCCmd_GripperTorqueSwitch)||(Code == PCCmd_GripperMotorLimit))
        {
            return ERR_NONE;
        }

        if(res!=ERR_NONE)
        {
            return res;
        }
        
        //!< ���id��        
        dat[0] = (u8)Code;
        dat[1] = (u8)(Code>>8);
        dat[2] = (u8)timStamp;
        dat[3] = (u8)(timStamp>>8);
        dat[4] = (u8)(timStamp>>16);
        dat[5] = (u8)(timStamp>>24);
        
        i=0;
            
        while(i<dlen)
        {
            iD = pdat[i] + (pdat[i+1]<<8);
                
            if((iD < Emb_StartID_Gripper)||(iD >= (Emb_StartID_Gripper + Motor_Num_Gripper)))
            {
                i += Unit;
            }
            else
            {
                memcpy(&dat[6],(pdat+i),Unit);   //!< ��ָ�����fCode,fTimStamp֮��
        
                //res = l_StorePcCmdtoGrippersCmdBuff(iD,dat,(Unit+6));   //!< ÿ��ָ�����fCode fTimStamp CodeUnit(����ָ�ָͬ�ԪҲ��ͬ)���
                res = g_StorePcCmdtoCmdBuff(EwayGrippers.mControl.pToMotorCmd,(iD-Emb_StartID_Gripper),dat,(Unit+6));

                if(res != ERR_NONE)
				{
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
					    SysLogWrite(LogMajor,"Grip g_StorePcCmdtoCmdBuff() failed,rt:%d.",res);

					    Bsp_printf("Grip g_StorePcCmdtoCmdBuff() failed,rt:%d.",res);
                    }
				}
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_XQueue_2_XCmdBuf_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Gripper_MASK))
                    {
                        Bsp_printf("Grip g_StorePcCmdtoCmdBuff() suc,fCode:0x%x,id:%d,Pos:0x%x,Spd:0x%x",(dat[0]+(dat[1]<<8)),iD,(dat[8]+(dat[9]<<8)),(dat[10]+(dat[11]<<8)));
                    }
                }

                i += Unit;
            }
        }
            
        return ERR_NONE;
}



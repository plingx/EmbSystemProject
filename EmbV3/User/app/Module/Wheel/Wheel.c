/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Wheel.c
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
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


extern osMutexId PCCmdBuffOpHandleWhl[Motor_Num_Wheel];
extern EwayEmbSysDebugModule* pDebug;



MotorSTATEBuff WheelsReg[Motor_Num_Wheel]={0};
MotorCMDBuff WheelsCmd[Motor_Num_Wheel]={    
                                         {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleWhl[0]},
                                         {{{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},0,0,0,&PCCmdBuffOpHandleWhl[1]}
};
MotorRecordBuff WheelsRecord[Motor_Num_Wheel]={0};
/*
MotorReducRatio WheelsReduRat[Motor_Num_Wheel]={
    1,
    1
};
*/
MotorReducRatio WheelsReduRat[Motor_Num_Wheel]={0};
Motor_LastPosCmd WheelsLstPos[Motor_Num_Wheel]={0};

#if Emb_Wheel_Offline_Debug
u32 EwayWheelCommState[2]={0};
#endif 

EwayWheelModule EwayWheels={
    {&WheelsCmd[0],&WheelsReduRat[0],Ctrl_Mode_Posit,App_Mode_Wheel,Pos_ClosedLoop_Absolute,WheelsLstPos,0},
    {&WheelsReg[0],&WheelsRecord[0],NULL},
    {{0,0},{0,0}},
    StopMode_Off,
    Motor_Num_Wheel,
    0x0000,
    0x0000
};


s8 sysEmbSetWheelGeneralMotorPosPID(u8* pdat, u16 dlen)
{
    u8 i,slvNums=0;
    s8 res = ERR_NONE;
    u8 dat[30]={0};
    u16 iD;
    u8 slvID[5]={0};
	
    if(pdat == NULL)
    {
        return ERR_INPUT_PARAMETERS;
    }

    i=0;

    while(i<dlen)
    {
        iD = pdat[i] + (pdat[i+1]<<8);
                
        if((iD < Emb_StartID_Wheel)||(iD >= (Emb_StartID_Wheel + Motor_Num_Wheel)))
        {
                i += PCCmd_WheelPosPIDSet_Unit;
        }
        else
        {
            memcpy((dat+(slvNums*EMB_READ_WHEELMODE_POS_PID_REG_NUM)),(pdat+i+2),EMB_READ_WHEELMODE_POS_PID_REG_NUM);
            
            slvID[slvNums] = iD;
            
            slvNums += 1 ;

            if(slvNums > 5)
            {
                break;          //!< 防止溢出
            }

            i += PCCmd_WheelPosPIDSet_Unit;                                
        }
    }
    
    res = sysGeneralMotorDiscontinuousBroadcastWrite(slvID,slvNums,EwayMotor_PosCtrl_KP_L,EMB_READ_WHEELMODE_POS_PID_REG_NUM,dat,EWAYBOT_WHEEL);

    return res;
}
/* --------------------------------------------------------------------------*/
/**
* @name l_StorePcCmdtoWheelsCmdBuff
* @brief 
* @details Code(2)+TimeStamp(4)+ID(2)+Posi(8)+Speed(2)
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 l_StorePcCmdtoWheelsCmdBuff(u16 id,u8* pdat,u8 len)    
{    
    MotorCMDBuff* pCmd;
    u8* pDest; 
    u8 piD;

    piD = (u8)(id-Emb_ID_Offset_WHEEL);    //!< 轮子ID起始为16-17,因此应减去16
                                           //!< 轮子ID起始为1，因此应减去1

    pCmd = EwayWheels.mControl.pToMotorCmd + piD;    //!< 根据舵机ID取其指令缓存地址
    
    if(len>CMD_PC_2_EMB_BUFF_LEN)                         //!< code(2) + timeStamp(4) + dataUnit 
        return ERR_INPUT_PARAMETERS;

    if(pCmd->dCmdCnt>=CMD_PC_2_EMB_BUFF_NUMS)
        return ERR_DATA_OVERFLOW;
    
    pDest = &(pCmd->dCmd[(pCmd->dCmdWr)][0]);

    memcpy(pDest,pdat,len);

    pCmd->dCmdWr++;pCmd->dCmdCnt++;

    if(pCmd->dCmdWr>=CMD_PC_2_EMB_BUFF_NUMS) pCmd->dCmdWr = 0;

    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
        (pDebug->secondFun&SecFunDebugCtrl_XQueue_2_XCmdBuf_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
    {
        if(len==10)
        {
            Bsp_printf("StoretoWheelCmd,fCode:0x%x,id:%d,Spd:0x%x",(pdat[0]+(pdat[1]<<8)),id,(pdat[8]+(pdat[9]<<8)));
        }
        else
        {
            Bsp_printf("StoretoWheelCmd,fCode:0x%x,id:%d,Pos:0x%x,Spd:0x%x",(pdat[0]+(pdat[1]<<8)),id,(pdat[8]+(pdat[9]<<8)+(pdat[10]<<16)+(pdat[11]<<24)),(pdat[12]+(pdat[13]<<8)));
        }
    }

    return ERR_NONE;    
}

s8 sysPcToWheelsCmdProcess(u16 Code,u32 timStamp,u16 dlen,u8* pdat)
{
    u8 i;
    s8 res=ERR_NONE;
    u8 Unit=0;
    u16 iD;
    u8 dat[20]={0};
            
        //!< 检查指令码 
        switch(Code)
        {
            case PCCmd_WheelPositionMove:
                        Unit = PCCmd_WheelPositionMove_Unit;                      //!< id(2bytes) + position(4bytes) + speed(2bytes)
                                
                        if(dlen<Unit) 
                            res = ERR_INPUT_PARAMETERS;
                    
                break;
            case PCCmd_WheelSpeedMove:               //!< id(2bytes) + speed(2bytes)
                        Unit = PCCmd_WheelSpeedMove_Unit;
                                
                        if(dlen<Unit) 
                            res = ERR_INPUT_PARAMETERS;
                    
                    break;
        
            case PCCmd_WheelPID:
                        Unit = PCCmd_WheelPosPIDSet_Unit;
                                
                        if(dlen<Unit) 
                            res = ERR_INPUT_PARAMETERS;
                break;
            /*
            case PCCmd_WheelTorqueSwitch:
                break;
            case PCCmd_WheelMotorLimit:
                break;
        */
                default:
                    res = ERR_INPUT_PARAMETERS;
                    break;
        }    

        if(res != ERR_NONE)
        {
            return res;
        }

        if(Code == PCCmd_WheelPID)
        {                
#if Emb_Set_MotorPID
            res = sysEmbSetWheelGeneralMotorPosPID(pdat,dlen);
                
            if(ERR_NONE != res)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    res = Bsp_printf("EmbSetWheelMotorPosPID() failed.");
                }
        
                return res;
            }
            else
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_CmdProcInfo_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                {
                    res = Bsp_printf("EmbSetWheelMotorPosPID succ.P:0x%04x,I:%04x,D:%04x.",(pdat[2]+(pdat[3]<<8)),(pdat[4]+(pdat[5]<<8)),(pdat[6]+(pdat[7]<<8)));
                }
            }           
#endif
            return ERR_NONE;
        }
        
        //!< 检查id号        
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
                
            if((iD < Emb_StartID_Wheel)||(iD >= (Emb_StartID_Wheel + Motor_Num_Wheel)))
            {
                i += Unit;
            }
            else
            {
                memcpy(&dat[6],(pdat+i),Unit);
                                         //14Bytes//  2By    4By  2By    4By    2By            位置模式
                                         //10Bytes//  2By    4By  2By    0By    2By            速度模式
                        //!< 轮子指令缓存16个14bytes:Code0-1 T0-3 iD0-1 Pos0-3 Speed0-1
                //res = l_StorePcCmdtoWheelsCmdBuff(iD,dat,(Unit+6));
                res = g_StorePcCmdtoCmdBuff(EwayWheels.mControl.pToMotorCmd,(iD-Emb_StartID_Wheel),dat,(Unit+6));

                if(res != ERR_NONE)
				{
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
					    SysLogWrite(LogMajor,"Wheel g_StorePcCmdtoCmdBuff() failed,rt:%d.",res);

					    //Bsp_printf("Wheel g_StorePcCmdtoCmdBuff() failed,rt:%d.",res);
                    }
				}
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_PC_2_Emb_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_XQueue_2_XCmdBuf_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                    {
                        Bsp_printf("Wheel g_StorePcCmdtoCmdBuff() suc,fCode:0x%x,id:%d,Pos:0x%x,Spd:0x%x",(dat[0]+(dat[1]<<8)),iD,(dat[8]+(dat[9]<<8)),(dat[10]+(dat[11]<<8)));
                    }
                }              

                i += Unit;
            }
        }
            
        return ERR_NONE;
}


s8 WheelBrake(EwayMotor_Stop_type eStopType)
{

    return ERR_NONE;
}


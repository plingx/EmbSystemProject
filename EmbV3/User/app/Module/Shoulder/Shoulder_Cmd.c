/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file Shoulder_Cmd.c
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

#include "includes.h"


extern EwayEmbSysModule EwayEmbSys;
extern EwayShoulderModule EwayShoulder;
extern EwayEmbSysDebugModule* pDebug;
extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];
/*


肩部运动处理流程(20180517因添加执行Clear指令后后是否位置保持的功能，又再次修改):
(20180612因添加记录最后一条PC运动指令，并在断网时发送最后一条指令令电机保持位置的功能，又再次修改)
(20180613因添加当压住上下限位后，再次发送单条指令并不能令肩膀电机正常执行的问题，又再次修改)

(1) 检查下位机与PC通信状态，若通信中断，
                                若为位置模式，则下发lastPos指令给每一个电机。返回。
                                若为速度模式，则下发speed=0指令给每一个电机。返回。
                            若通信正常，则继续(2)

(2) 检查肩部电机指令缓存中的指令个数:若为0，则继续(3)
                                     若不为0，则继续(4)
                                     
(3) 检查是否有Clear指令后的位置保持功能:
        若有位置保持的指令:
            若需要位置保持，则根据当前位置/速度模式，下发相应的指令，清除标志，返回。
                当前电机为位置模式，则下发当前电机的位置及默认最小速度的指令包给电机，清除Clear标志，返回。
                当前电机为速度模式，则下发转换当前电机模式为位置模式并下发当前电机的位置及默认最小速度的指令包给电机，清除Clr标志，返回。
            若不需要位置保持，则do nothing,清除收到过Clear标志，返回。
            
        若没有位置保持的指令待处理:
            检查上/下限位是否有被压住的情况:
                    有  :  检查EwayShoulder.shLimit.mCtrl.en是否使能
                                若使能，则不发送保持当前位置的指令，返回。
                                若未使能，则发送保持当前位置的指令，返回。
                    
                    没有:  若EwayShoulder.shLimit.mCtrl.en值不为0，清除，返回。
                                     
(4) 取1条指令，不成功，
               成功，则继续(5)

(5) 当前指令类型与当前电机控制模式比较，若不同，则先发送模式转换命令包，并更改当前控制模式(位置or速度),继续(7)
                                        若相同，则继续(6)

(6) 获取是否压住上下限位的标识位,若并未压住上下限位，则
                                                a.可以清除mCtrl.en标志.
                                                b.直接发送指令.
                                                c.返回ERR_NONE.
                                 若压住上下限位，则继续(7)

(7) 判断指令运动趋势(向上or向下)，
		若(指令为向上运动，且当前压住了上限位) or (指令为向下运动，且当前压住了下限位) 则发送刹车指令，若开启mCtrl.en标志，则清除，并清除速度or位置指令记录。返回ERR_NONE.
		若(指令为向上运动，且当前压住了下限位) or (指令为向下运动，且当前压住了上限位) 则指令照常发送.若未开启mCtrl.en标志，则开启，并填写新的速度or位置指令记录。返回ERR_NONE.


*/

/* --------------------------------------------------------------------------*/
/**
* @name sysModuleShoulderMovement
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
s8 sysModuleShoulderMovement(EwayShoulderModule* pModule)
{
    u8 dat[20];    //!< 用于存放从缓存读出的18byte数据.
    //u8 len,fg;
    u16 fCode;
    u8 modeDat[4]={0};
    u8 tmpDat[20]={0};
    s8 res;
    u8 tmp;
    u8* pd;
    MotorCMDBuff* pmotorCmd;
    Motor_CtrlMode mode;
    s32 sPos;
    s16 sSpd;
    s32 SrcSpd;
    MotorReducRatio* pShRa = pModule->mControl.pMotorRatio;
    Motor_LastPosCmd* pLast = pModule->mControl.pmlstPos;

    //!< 检查与上位机通信状态，若为断开，则向电机下发速度为0的指令or刹车的指令or位置指令，总之就是要停下来。
    if((EwayEmbSys.Comm.wPc & 0x01) == EMB_COMM_STATUS_DISCONNECTED)
    {
        //!< 向肩膀电机发送紧急停止的指令
        tmpDat[0] = 1;    //!< 对于肩膀升降电机 0-减速停止 1-紧急停止 2-自由停止
        res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotor_Stop,1,tmpDat,EWAYBOT_SHOULDER);
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("2.1 shMov-0 PC Comm break,send (0x51-1) failed.rt:%d.line:%d",res,__LINE__);
            }
        }
        else
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
            {
                res = Bsp_printf("2.1 shMov-0 PC Comm break,send (0x51-1) Cmd Suc!");
            }
        }

        /*
        //!< 是否有收到过PC的指令，收到过则发送最后一条指令，没有收到过则什么都不发(这种情况只发生在下位机上电后，无上位机连接的情况下)
        if(EwayShoulder.mControl.pmlstPos->avail !=  LastPosAvailableOff)
        {            
            if(pModule->mControl.mCtrlMode == Ctrl_Mode_Posit)
            {
                //若为位置模式则发送当前位置及默认速度的指令            
                tmpDat[0] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                tmpDat[1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
                
                tmpDat[2] = 0x00;
                tmpDat[3] = 0x00;

                if(EmbGetLastPosCmdSendtoMotor(EwayShoulder.mControl.pmlstPos,0,&sPos)==ERR_NONE)
                {
                    res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotorReg_TargSpd_L,8,tmpDat,EWAYBOT_SHOULDER);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            res = Bsp_printf("2.1 shMov-0 PC Comm disconnected,send lastPosCmd failed.");
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                        {
                            res = Bsp_printf("2.1 shMov-0 PC Comm disconnected,send lastPosCmd Suc!pos:0x%x!",sPos);
                        }
                    }    
                }                
            }
            else        //!< 只要上电后收到过PC的指令，则就一定会有记录其最后一条指令的行为。
            {
                //!< 若为速度模式则发送speed=0的指令
                sSpd = 0;
                        
                res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotorReg_TargSpd_L,2,(u8 *)(&sSpd),EWAYBOT_SHOULDER);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("2.1 shMov-0 PC Comm disconnected,send spd=0 cmd failed.rt:%d.line:%d",res,__LINE__);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        res = Bsp_printf("2.1 shMov-0 PC Comm disconnected,send spd=0 Cmd Suc!");
                    }
                }
            }

        }*/

        return ERR_NONE;
    }

    //!< 检查缓存中是否有待发指令
    if(pModule->mControl.pToMotorCmd->dCmdCnt==0)
    {
        if((pModule->mControl.posKeep&0xFF00)!=0)     //!< 收到过Clear指令
        {
            tmp = (u8)pModule->mControl.posKeep;
            
            if(tmp == 'K')       //!< 需要位置保持
            {
                if(pModule->mControl.mCtrlMode == Ctrl_Mode_Posit)
                {
                    //!< pos mode,do nothing
                    tmpDat[0] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                    tmpDat[1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
            
                    tmpDat[2] = 0x00;
                    tmpDat[3] = 0x00;
                    memcpy(&tmpDat[4],(u8*)(&sysEwServo[Emb_StartID_Shoulder-1].uPosWheel),4);   //!< 取Position的4个字节下发下去

                    res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotorReg_TargSpd_L,8,tmpDat,EWAYBOT_SHOULDER);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            res = Bsp_printf("2.1 Sh Keep Pos Cmd, Posi mode,send(0x56-8) failed.");
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                        {
                            res = Bsp_printf("2.1 Sh Keep Pos Cmd, Posi mode,send(0x56-8) spd=0,pos:0x%02x-%02x-%02x-%02x!",tmpDat[7],tmpDat[6],tmpDat[5],tmpDat[4]);
                        }
                    }    
                }
                else if(pModule->mControl.mCtrlMode == Ctrl_Mode_Speed)
                {
                    //!< speed mode,send mode change and send current Pos and default speed
                    modeDat[0] = EwayMotor_CtrlMode; 
                    modeDat[1] = 1;
                    modeDat[2] = EWAY_MOTOR_CONTROL_MODE_POSITION;                      //!< 位置模式		
			
			        pModule->mResp.mExeCmRecd[0] += (0x01<<4);    //!< 记录成功发送了一个Single Write(0x03)的指令 对0x03指令的发送记录
                    pModule->mResp.mExeCmRecd[1]  = Ctrl_Mode_Posit;    //!< 记录下发了切换到位置模式

                    memcpy(tmpDat,modeDat,3);
                   
                    sSpd = MOTOR_DEFAULT_SPEED_MIN;
                                
                    pd = &pModule->mState.pFromMotor->mRegs[0] ;                                
                    sPos = pd[15] + (pd[16]<<8) + (pd[17]<<16) + (pd[18]<<24);

                    tmpDat[3] = EwayMotorReg_TargSpd_L;
                    tmpDat[4] = 8;                        
                    tmpDat[5] = (u8)sSpd;
                    tmpDat[6] = (u8)(sSpd>>8);
                    tmpDat[7]= 0x00;
                    tmpDat[8]= 0x00;        
                    tmpDat[9] = (u8)sPos;
                    tmpDat[10] = (u8)(sPos>>8);
                    tmpDat[11] = (u8)(sPos>>16);
                    tmpDat[12] = (u8)(sPos>>24);            

                    res = g_GeneralMotorSingleWrite(Emb_StartID_Shoulder,13,tmpDat,EWAYBOT_SHOULDER);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            SysLogWrite(LogMajor,"Sh Keep Pos Cmd,Change Speed mode to Pos and send CurrPos&spd cmd to Stop failed.rt:%d.line:%d",res,__LINE__);
                            
                            res = Bsp_printf("Sh Keep Pos Cmd,Change Speed mode to Pos and send CurrPos&spd cmd to Stop failed.rt:%d.line:%d",res,__LINE__);
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)\
                        &&(pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                        {                                                        
                            res = Bsp_printf("Sh Keep Pos Cmd,Change Speed mode to Pos and send CurrPos:0x%x to Stop succ!",sPos);
                        }
                    }                    
                }            
            }
            else if(tmp == 'N')  //!< 不需要位置保持
            {    
                  //!< do nothing
            }

            EwayShoulder.mControl.posKeep = 0x0000; 

            return ERR_NONE;
        }
        else    //!< 无位置保持的指令待处理，则开始检查是否有上下限位被压住
        {
            //!< check Up-Dn Pin Status
            if(((pModule->shLimit.pinState[SHOULDER_LIMIT_DN]&SHOULDER_LIMIT_TRIGGER_BIT)==0)||((pModule->shLimit.pinState[SHOULDER_LIMIT_UP]&SHOULDER_LIMIT_TRIGGER_BIT)==0))
            {
                if(pModule->shLimit.mCtrl.en==0)
                {
                    if(pModule->mControl.mCtrlMode == Ctrl_Mode_Posit)          //!< 压住了上/下限位，位置模式，发送当前位置
                    {
                        //!< pos mode,do nothing
                        tmpDat[0] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                        tmpDat[1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
            
                        tmpDat[2] = 0x00;
                        tmpDat[3] = 0x00;
                        memcpy(&tmpDat[4],(u8*)(&sysEwServo[Emb_StartID_Shoulder-1].uPosWheel),4);   //!< 取Position的4个字节下发下去

                        res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotorReg_TargSpd_L,8,tmpDat,EWAYBOT_SHOULDER);
                        if(ERR_NONE != res)
                        {
                            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                            {
                                res = Bsp_printf("Sh no Cmd,Up/Dn touched,shLimit.mCtrl.en=0,send CurPos(0x56-8) to Stop failed.rt:%d,line:%d",res,__LINE__);
                            }
                        }
                        else
                        {
                            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(((pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                            (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK))||(((pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
                            (pDebug->secondFun&SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK))))&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                            {
                                res = Bsp_printf("Sh no Cmd,Up/Dn touched,shLimit.mCtrl.en=0,send CurPos(0x56-8) to Stop Suc!");
                            }
                        }    
                    }
                    else if(pModule->mControl.mCtrlMode == Ctrl_Mode_Speed)    //!< 压住了上限位，速度模式，发送速度为0的指令
                    {
                        tmpDat[0] = 0;tmpDat[1] = 0;  //!< motor reg0x56:2byte RPM
                        res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotorReg_TargSpd_L,2,tmpDat,EWAYBOT_SHOULDER);
                        if(ERR_NONE != res)
                        {
                            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                            {
                                res = Bsp_printf("Sh no Cmd,Up/Dn touched,shLimit.mCtrl.en=0,send spd=0(0x56-2) to Stop failed.rt:%d,line:%d",res,__LINE__);
                            }
                        }
                        else
                        {
                            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(((pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                            (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK))||(((pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
                            (pDebug->secondFun&SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK))))&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                            {
                                res = Bsp_printf("Sh no Cmd,Up/Dn touched,shLimit.mCtrl.en=0,send spd=0(0x56-2) to Stop Suc!");
                            }
                        }   
                    }
                }   
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(((pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK))||(((pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK))))&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        res = Bsp_printf("Sh no Cmd,Up/Dn touched,shLimit.mCtrl.en=1,do nothing!");
                    }
                }
            }
            else    //!< 当前无上/下限位被压住
            {
                if(EwayShoulder.shLimit.mCtrl.en!=0)        //!< 只要不压上/下限位，则就可以清标志了
                {
                    EwayShoulder.shLimit.mCtrl.en = 0;
                    
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        res = Bsp_printf("Sh no Cmd,Up/Dn don't be tuched,Clr shLimit.en Suc!");
                    }
                }
            }

            return ERR_NONE; 
        }
    
    }

    pmotorCmd = pModule->mControl.pToMotorCmd;    

    res = g_GetPCCmdEmbToMotorCmd(pmotorCmd,0,dat);
    if(res!=ERR_NONE)
    {
        return ERR_PROCESS_FAILED;
    }

    //!< fCode T0-3 iD0-1 Pos0-3 Spd0-1
    //!<   2    4     2    4       2       =  14 bytes

    //!< fCode T0-3 iD0-1 Pos0-3 Spd0-3
    //!<   2    4     2    4       4       =  16 bytes      //!< 20180717

    fCode = dat[0]+(dat[1]<<8);

    if(fCode == PCCmd_ShoulderPosMoveMode)
    {
        mode = Ctrl_Mode_Posit;
    }
    else if(fCode == PCCmd_ShoulderSpdMoveMode)
    {
        mode = Ctrl_Mode_Speed;
    }
    else 
        return ERR_NONE;              //!< 非速度位置指令，待处理

    if(mode!=pModule->mControl.mCtrlMode)     //!< 当前指令模式与待发送指令模式不同，则发送模式更改指令，并更改肩部电机控制模式变量 
    {
        if(mode == Ctrl_Mode_Posit)
        {             
            modeDat[0] = EwayMotor_CtrlMode; //!< 更改电机控制模式，reg0x08:0-速度控制模式,1-位置控制模式,2-转矩控制模式,3-占空比模式
            modeDat[1] = 1;
            modeDat[2] = EWAY_MOTOR_CONTROL_MODE_POSITION;                      //!< 位置模式
            
            EwayShoulder.mResp.mExeCmRecd[0] += (0x01<<4);    //!< 记录发送了一个Single Write(0x03)的指令 对0x03指令的发送记录
            EwayShoulder.mResp.mExeCmRecd[1]  = Ctrl_Mode_Posit;    //!< 记录下发了切换到位置模式
        }
        else if(mode == Ctrl_Mode_Speed)
        {                
            modeDat[0] = EwayMotor_CtrlMode; //!< 更改电机控制模式，reg0x08:0-速度控制模式,1-位置控制模式,2-转矩控制模式,3-占空比模式
            modeDat[1] = 1;
            modeDat[2] = EWAY_MOTOR_CONTROL_MODE_SPEED;                  //!< 速度模式

            EwayShoulder.mResp.mExeCmRecd[0] += (0x01<<4);    //!< 记录发送了一个Single Write(0x03)的指令 对0x03指令的发送记录
            EwayShoulder.mResp.mExeCmRecd[1]  = Ctrl_Mode_Speed;    //!< 记录下发了切换到位置模式
        }
    }

    if(((pModule->shLimit.pinState[SHOULDER_LIMIT_DN]&SHOULDER_LIMIT_TRIGGER_BIT)!=0)&&((pModule->shLimit.pinState[SHOULDER_LIMIT_UP]&SHOULDER_LIMIT_TRIGGER_BIT)!=0))
    {        
        if(EwayShoulder.shLimit.mCtrl.en!=0)        //!< 只要不压上/下限位，则就可以清标志了
        {
            EwayShoulder.shLimit.mCtrl.en = 0;

            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
            (pDebug->secondFun&SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
            {
                res = Bsp_printf("Sh Cmd !=0,Up/Dn don't be tuched,Clr shLimit.en Suc!");
            }
        }
        
        //<若没有上下限位被压住，则转换并发送指令.返回
        if(mode==Ctrl_Mode_Speed)
        {
            SrcSpd = dat[12] + (dat[13]<<8) + (dat[14]<<16) + (dat[15]<<24);
                       
            //!< 检查速度值是否超过速度大小限制
            sysEmbCheckSendToShoulderMotorParameters(&SrcSpd);
            
            //!< 根据id号取出减速比并计算下发给motor的速度值
            sSpd = (s16)(SrcSpd*(*(pShRa))/MOTOR_SPEED_COEFFICIENT);
            
            EmbEnableLastPosCmdAvailableFlag(pLast,0);

            EmbRecordCmdSendToMotor(0,mode,sSpd,0,EWAYBOT_SHOULDER);    //!< 记录本周期下发的指令

            if(modeDat[0] != 0) //!< 表示有模式控制改变指令要下发
            {
                memcpy(tmpDat,modeDat,3);

                tmpDat[3] = EwayMotorReg_TargSpd_L;
                tmpDat[4] = 2;
                tmpDat[5] = (u8)sSpd;
                tmpDat[6] = (u8)(sSpd>>8);
                
                res = g_GeneralMotorSingleWrite(Emb_StartID_Shoulder,7,tmpDat,EWAYBOT_SHOULDER);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("2.1 ShMov-5,no Up-Dn Pin Touched,change mode to Spd and Snd spd cmd failed.rt:%d.line:%d",res,__LINE__);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        res = Bsp_printf("2.1 ShMov-5,no Up-Dn Pin touched.,change mode to Spd and Snd spd cmd(0x56-2) Suc.Spd:0x%x.",sSpd);
                    }
                }

            }
            else
            {                
                res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotorReg_TargSpd_L,2,(u8 *)(&sSpd),EWAYBOT_SHOULDER);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("2.1 Sh MotorBrodcstWr(0x56-2) failed.rt:%d.line:%d",res,__LINE__);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        res = Bsp_printf("2.1 ShMov-5,no Up-Dn Pin touched.send(0x56-2) Suc.Spd:0x%x.",sSpd);
                    }
                }
            }
            
        }
        else if(mode==Ctrl_Mode_Posit)
        {
            SrcSpd = dat[12] + (dat[13]<<8) + (dat[14]<<16) + (dat[15]<<24);
                       
            //!< 检查速度值是否超过速度大小限制
            sysEmbCheckSendToShoulderMotorParameters(&SrcSpd);
            
            //!< 根据id号取出减速比并计算下发给motor的速度值
            sSpd = (s16)(SrcSpd*(*(pShRa))/MOTOR_SPEED_COEFFICIENT);            
            //!< 检查速度值是否超过速度大小限制

            sPos = dat[8]+(dat[9]<<8)+(dat[10]<<16)+(dat[11]<<24);

            EmbRecordingLastPosCmdSendtoMotor(pLast,0,sPos);//----

            EmbRecordCmdSendToMotor(0,mode,sSpd,sPos,EWAYBOT_SHOULDER);    //!< 记录本周期下发的指令

            if(modeDat[0] != 0) //!< 表示有模式控制改变指令要下发
            {
                memcpy(tmpDat,modeDat,3);
                
                tmpDat[3] = EwayMotorReg_TargSpd_L;
                tmpDat[4] = 8;
                                            //!< Reg
                tmpDat[5] = (u8)sSpd;       //!< 0x56
                tmpDat[6] = (u8)(sSpd>>8);  //!< 0x57
                tmpDat[7] = 0x00;           //!< 0x58
                tmpDat[8] = 0x00;           //!< 0x59

                memcpy(&tmpDat[9],&dat[8],4);   //!< 取Position的4个字节下发下去
                                
                res = g_GeneralMotorSingleWrite(Emb_StartID_Shoulder,13,tmpDat,EWAYBOT_SHOULDER);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("2.1 ShMov-6,no Up-Dn Pin Touched,change mode to Pos and Snd(0x56-8) failed.rt:%d.line:%d",res,__LINE__);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        Bsp_printf("2.1 ShMov-6,no Up-Dn touched,change mode to Pos and Snd(0x56-8) Suc,spd:0x%x,Pos:0x%02x-%02x-%02x-%02x",sSpd,tmpDat[12],tmpDat[11],tmpDat[10],tmpDat[9]);
                    }
                }

            }
            else
            {   
                tmpDat[0] = (u8)(sSpd);
                tmpDat[1] = (u8)(sSpd>>8);
            
                tmpDat[2] = 0x00;
                tmpDat[3] = 0x00;
                memcpy(&tmpDat[4],&dat[8],4);   //!< 取Position的4个字节下发下去
            
                res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotorReg_TargSpd_L,8,tmpDat,EWAYBOT_SHOULDER);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("2.1 ShMov-6,no Up-Dn touched,send(0x56-8) failed.");
                    }
                }            
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        Bsp_printf("2.1 ShMov-6,no Up-Dn touched,send(0x56-8) suc,spd:0x%04x,Pos:0x%08x",sSpd,((tmpDat[7]<<24)+(tmpDat[6]<<16)+(tmpDat[5]<<8)+ tmpDat[4]));
                    }
                }
            }
            
        }

        return ERR_NONE;
        
    }
    else
    {
        //!< 若有上下限位被压住，则判断指令的运动趋势        
        //!< 当前是在上限位还是下限位
        if(mode==Ctrl_Mode_Speed)
        {
            //!< 若下发指令为速度向上且下限位被压住or下发指令为速度向下且上限位被压，则照常下发指令
            //!< 否则下发brake指令
            SrcSpd = dat[12] + (dat[13]<<8) + (dat[14]<<16) + (dat[15]<<24);          //!< 根据id号取出减速比并计算下发给motor的速度值          
                       
            //!< 检查速度值是否超过速度大小限制
            sysEmbCheckSendToShoulderMotorParameters(&SrcSpd);
            
            sSpd = (s16)(SrcSpd*(*(pShRa))/MOTOR_SPEED_COEFFICIENT);    
            
            if(((sSpd>0)&&((pModule->shLimit.pinState[SHOULDER_LIMIT_DN]&SHOULDER_LIMIT_TRIGGER_BIT)==0))||((sSpd<0)&&((pModule->shLimit.pinState[SHOULDER_LIMIT_UP]&SHOULDER_LIMIT_TRIGGER_BIT)==0)))
            {   //!< 若指令为向上运动且下限位被压住 or 指令为向下运动且上限位被压住，则照常发送指令
            
                EmbEnableLastPosCmdAvailableFlag(pLast,0);              //!< 记录最后一条指令

                EmbRecordCmdSendToMotor(0,mode,sSpd,0,EWAYBOT_SHOULDER);    //!< 记录本周期下发的指令

                if(pModule->shLimit.mCtrl.en != UpDnLimShlCtrl_SpdCmd)  //!< 使能压限位放行标志，并记录速度值以备后用
                {
                    pModule->shLimit.mCtrl.en = UpDnLimShlCtrl_SpdCmd;
                    
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        res = Bsp_printf("2.1 ShMov-7 Up-Dn touched,speed is available,Enable shLimit.en!");
                    }
                }
                pModule->shLimit.mCtrl.SpdCmd = sSpd;
                
                if(modeDat[0] != 0) //!< 表示有模式控制改变指令要下发
                {
                    memcpy(tmpDat,modeDat,3);

                    tmpDat[3] = EwayMotorReg_TargSpd_L;
                    tmpDat[4] = 2;
                    tmpDat[5] = (u8)sSpd;
                    tmpDat[6] = (u8)(sSpd>>8);
                
                    res = g_GeneralMotorSingleWrite(Emb_StartID_Shoulder,7,tmpDat,EWAYBOT_SHOULDER);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-7 Up-Dn touched,send(0x56-2) failed.rt:%d.line:%d",res,__LINE__);
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-7 Up-Dn touched,send(0x56-2) Suc!spd:0x%04x.",sSpd);
                        }
                    }
                }
                else
                {
                    res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotorReg_TargSpd_L,2,(u8*)(&sSpd),EWAYBOT_SHOULDER);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-7 Up-Dn touched,send(0x56-2) failed.");
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-7 Up-Dn touched,send(0x56-2) Suc!spd:0x%04x.",sSpd);
                        }
                    }
                }
            }            
            else    //!< 若指令为向上运动且上限位被压住 or 指令为向下运动且下限位被压住，则send brake
            {
                if(pModule->shLimit.mCtrl.en != UpDnLimShlCtrl_Off)
                {
                    pModule->shLimit.mCtrl.en = UpDnLimShlCtrl_Off;

                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        res = Bsp_printf("2.1 ShMov-8 Up-Dn touched,speed isn't available,Clr shLimit.en!");
                    }
                }
                
                if(modeDat[0] != 0) //!< 表示有模式控制改变指令要下发
                {
                    memcpy(tmpDat,modeDat,3);

                    tmpDat[3] = EwayMotorReg_TargSpd_L;
                    tmpDat[4] = 2;
                    tmpDat[5] = 0x00;
                    tmpDat[6] = 0x00;
                
                    res = g_GeneralMotorSingleWrite(Emb_StartID_Shoulder,7,tmpDat,EWAYBOT_SHOULDER);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-8 up-dn touched,change mode to Spd and Snd(0x56-2) failed.rt:%d.line:%d",res,__LINE__);
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-8 up-dn touched,change mode to Spd and Snd spd=0(0x56-2) Suc!");
                        }
                    }
                }
                else
                {
                    tmpDat[0] = 0;tmpDat[1] = 0;  //!<motor reg0x56:2byte RPM
                    res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotorReg_TargSpd_L,2,tmpDat,EWAYBOT_SHOULDER);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-8 up-dn touched,spd mode send spd=0(0x56-2) failed.rt:%d,line:%d.",res,__LINE__);
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-8 up-dn touched,spd mode send spd=0(0x56-2) Suc!");
                        }
                    }
                }
            }
        }
        else if(mode==Ctrl_Mode_Posit)
        {
            //!< 若下发指令为位置向上且下限位被压住or下发指令为位置向下且上限位被压，则照常下发指令
            //!< 否则下发brake指令

            sPos = dat[8] + (dat[9]<<8) + (dat[10]<<16) + (dat[11]<<24);
            
            SrcSpd = dat[12] + (dat[13]<<8) + (dat[14]<<16) + (dat[15]<<24);
                       
            //!< 检查速度值是否超过速度大小限制
            sysEmbCheckSendToShoulderMotorParameters(&SrcSpd);
            
            sSpd = (s16)(SrcSpd*(*(pShRa))/MOTOR_SPEED_COEFFICIENT);  

            //!< 指令为向上运动且压住了下限位 or 指令为下行运动且压住了上限位，则照常发，否则brake
            if(((sPos > sysEwServo[Emb_StartID_Shoulder-1].uPosWheel)&&((EwayShoulder.shLimit.pinState[SHOULDER_LIMIT_DN]&SHOULDER_LIMIT_TRIGGER_BIT)==0))||((sPos < sysEwServo[Emb_StartID_Shoulder-1].uPosWheel)&&((EwayShoulder.shLimit.pinState[SHOULDER_LIMIT_UP]&SHOULDER_LIMIT_TRIGGER_BIT)==0)))
            {   //!< 若指令为向上运动且下限位被压住 or 指令为向下运动且上限位被压住，则照常发送指令
                EmbRecordingLastPosCmdSendtoMotor(pLast,0,sPos);//----

                EmbRecordCmdSendToMotor(0,mode,sSpd,sPos,EWAYBOT_SHOULDER);    //!< 记录本周期下发的指令
                
                if(pModule->shLimit.mCtrl.en != UpDnLimShlCtrl_PosCmd)
                {
                    pModule->shLimit.mCtrl.en = UpDnLimShlCtrl_PosCmd;

                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        res = Bsp_printf("2.1 ShMov-9 Up/Dn touched,Posi is available,Enable shLimit.en!");
                    }
                }
                pModule->shLimit.mCtrl.SpdCmd = sSpd;
                pModule->shLimit.mCtrl.PosCmd = sPos;    
                
                if(modeDat[0] != 0) //!< 表示有模式控制改变指令要下发
                {
                    memcpy(tmpDat,modeDat,3);

                    tmpDat[3] = EwayMotorReg_TargSpd_L;
                    tmpDat[4] = 8;
                    tmpDat[5] = (u8)(sSpd);
                    tmpDat[6] = (u8)(sSpd>>8);                    
                    tmpDat[7] = 0x00;
                    tmpDat[8] = 0x00;
                    
                    memcpy(&tmpDat[9],(u8*)(&sPos),4);   //!< 取Position的4个字节下发下去
                
                    res = g_GeneralMotorSingleWrite(Emb_StartID_Shoulder,13,tmpDat,EWAYBOT_SHOULDER);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-9,Up/Dn touched,Change mode to Pos and send(0x56-8) failed.rt:%d.line:%d",res,__LINE__);
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-9,Up/Dn touched,Change mode to Pos and send(0x56-8),spd:0x%x,Pos:0x%x",sSpd,sPos);
                        }
                    }
                }
                else
                {                    
                    tmpDat[0] = (u8)(sSpd);
                    tmpDat[1] = (u8)(sSpd>>8);
                    
                    tmpDat[2] = 0x00;
                    tmpDat[3] = 0x00;
                    memcpy(&tmpDat[4],(u8*)(&sPos),4);   //!< 取Position的4个字节下发下去
                    
                    res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotorReg_TargSpd_L,8,tmpDat,EWAYBOT_SHOULDER);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-9 sysGeneralMotorBroadcastWrite(0x56) failed.rt:%d.",res);
                        }
                    }
                    
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        Bsp_printf("2.1 ShMov-9,Up or Dn touched,send(0x56-8),spd:0x%x,Pos:0x%x",sSpd,sPos);
                    }
                }            
            }
            else //brake
            {
                if(pModule->shLimit.mCtrl.en != UpDnLimShlCtrl_Off)
                {
                    pModule->shLimit.mCtrl.en = UpDnLimShlCtrl_Off;  

                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        res = Bsp_printf("2.1 ShMov-10 Up/Dn touched,Posi isn't available,Enable shLimit.en!");
                    }
                }
                
                if(modeDat[0] != 0) //!< 表示有模式控制改变指令要下发
                {
                    memcpy(tmpDat,modeDat,3);

                    tmpDat[3] = EwayMotorReg_TargSpd_L;             //!< 发送速度为0，位置为当前位置的brake指令
                    tmpDat[4] = 8;
                    tmpDat[5] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                    tmpDat[6] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);                    
                    tmpDat[7] = 0x00;
                    tmpDat[8] = 0x00;
                    
                    memcpy(&tmpDat[9],(u8*)(&sysEwServo[Emb_StartID_Shoulder-1].uPosWheel),4);   //!< 取Position的4个字节下发下去
                
                    res = g_GeneralMotorSingleWrite(Emb_StartID_Shoulder,13,tmpDat,EWAYBOT_SHOULDER);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-10,Up or Dn touched,Change mode to Pos and send brake(0x56-8) failed.rt:%d.line:%d",res,__LINE__);
                        }
                    }
                    else
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                        (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-10,Up or Dn touched,Change mode to Pos and send brake(0x56-8),spd:0x%x,Pos:0x%x",sSpd,sysEwServo[Emb_StartID_Shoulder-1].uPosWheel);
                        }
                    }
                }
                else
                {   
                    //!< 发送速度为0，位置为当前位置的brake指令
                    tmpDat[0] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                    tmpDat[1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
                    
                    tmpDat[2] = 0x00;
                    tmpDat[3] = 0x00;
                    memcpy(&tmpDat[4],(u8*)(&sysEwServo[Emb_StartID_Shoulder-1].uPosWheel),4);   //!< 取Position的4个字节下发下去
                    
                    res = sysGeneralMotorBroadcastWrite(Emb_StartID_Shoulder,Motor_Num_Shoulder,EwayMotorReg_TargSpd_L,8,tmpDat,EWAYBOT_SHOULDER);
                    if(ERR_NONE != res)
                    {
                        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                        {
                            res = Bsp_printf("2.1 ShMov-10 sysGeneralMotorBroadcastWrite(0x56-8) failed.");
                        }
                    }            
                    
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        Bsp_printf("2.1 ShMov-10,Up or Dn touched,send brake(0x56-8),spd:0x%x,Pos:0x%x",sSpd,sysEwServo[Emb_StartID_Shoulder-1].uPosWheel);
                    }
                }            
            }            
        }        
    }
    
    return ERR_NONE;
}



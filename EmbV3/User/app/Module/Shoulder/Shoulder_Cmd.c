/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Shoulder_Cmd.c
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

#include "includes.h"


extern EwayEmbSysModule EwayEmbSys;
extern EwayShoulderModule EwayShoulder;
extern EwayEmbSysDebugModule* pDebug;
extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];
/*


�粿�˶���������(20180517�����ִ��Clearָ�����Ƿ�λ�ñ��ֵĹ��ܣ����ٴ��޸�):
(20180612����Ӽ�¼���һ��PC�˶�ָ����ڶ���ʱ�������һ��ָ����������λ�õĹ��ܣ����ٴ��޸�)
(20180613����ӵ�ѹס������λ���ٴη��͵���ָ���������������ִ�е����⣬���ٴ��޸�)

(1) �����λ����PCͨ��״̬����ͨ���жϣ�
                                ��Ϊλ��ģʽ�����·�lastPosָ���ÿһ����������ء�
                                ��Ϊ�ٶ�ģʽ�����·�speed=0ָ���ÿһ����������ء�
                            ��ͨ�������������(2)

(2) ���粿���ָ����е�ָ�����:��Ϊ0�������(3)
                                     ����Ϊ0�������(4)
                                     
(3) ����Ƿ���Clearָ����λ�ñ��ֹ���:
        ����λ�ñ��ֵ�ָ��:
            ����Ҫλ�ñ��֣�����ݵ�ǰλ��/�ٶ�ģʽ���·���Ӧ��ָ������־�����ء�
                ��ǰ���Ϊλ��ģʽ�����·���ǰ�����λ�ü�Ĭ����С�ٶȵ�ָ�������������Clear��־�����ء�
                ��ǰ���Ϊ�ٶ�ģʽ�����·�ת����ǰ���ģʽΪλ��ģʽ���·���ǰ�����λ�ü�Ĭ����С�ٶȵ�ָ�������������Clr��־�����ء�
            ������Ҫλ�ñ��֣���do nothing,����յ���Clear��־�����ء�
            
        ��û��λ�ñ��ֵ�ָ�������:
            �����/����λ�Ƿ��б�ѹס�����:
                    ��  :  ���EwayShoulder.shLimit.mCtrl.en�Ƿ�ʹ��
                                ��ʹ�ܣ��򲻷��ͱ��ֵ�ǰλ�õ�ָ����ء�
                                ��δʹ�ܣ����ͱ��ֵ�ǰλ�õ�ָ����ء�
                    
                    û��:  ��EwayShoulder.shLimit.mCtrl.enֵ��Ϊ0����������ء�
                                     
(4) ȡ1��ָ����ɹ���
               �ɹ��������(5)

(5) ��ǰָ�������뵱ǰ�������ģʽ�Ƚϣ�����ͬ�����ȷ���ģʽת��������������ĵ�ǰ����ģʽ(λ��or�ٶ�),����(7)
                                        ����ͬ�������(6)

(6) ��ȡ�Ƿ�ѹס������λ�ı�ʶλ,����δѹס������λ����
                                                a.�������mCtrl.en��־.
                                                b.ֱ�ӷ���ָ��.
                                                c.����ERR_NONE.
                                 ��ѹס������λ�������(7)

(7) �ж�ָ���˶�����(����or����)��
		��(ָ��Ϊ�����˶����ҵ�ǰѹס������λ) or (ָ��Ϊ�����˶����ҵ�ǰѹס������λ) ����ɲ��ָ�������mCtrl.en��־���������������ٶ�orλ��ָ���¼������ERR_NONE.
		��(ָ��Ϊ�����˶����ҵ�ǰѹס������λ) or (ָ��Ϊ�����˶����ҵ�ǰѹס������λ) ��ָ���ճ�����.��δ����mCtrl.en��־������������д�µ��ٶ�orλ��ָ���¼������ERR_NONE.


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
    u8 dat[20];    //!< ���ڴ�Ŵӻ��������18byte����.
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

    //!< �������λ��ͨ��״̬����Ϊ�Ͽ����������·��ٶ�Ϊ0��ָ��orɲ����ָ��orλ��ָ���֮����Ҫͣ������
    if((EwayEmbSys.Comm.wPc & 0x01) == EMB_COMM_STATUS_DISCONNECTED)
    {
        //!< ���������ͽ���ֹͣ��ָ��
        tmpDat[0] = 1;    //!< ���ڼ��������� 0-����ֹͣ 1-����ֹͣ 2-����ֹͣ
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
        //!< �Ƿ����յ���PC��ָ��յ����������һ��ָ�û���յ�����ʲô������(�������ֻ��������λ���ϵ������λ�����ӵ������)
        if(EwayShoulder.mControl.pmlstPos->avail !=  LastPosAvailableOff)
        {            
            if(pModule->mControl.mCtrlMode == Ctrl_Mode_Posit)
            {
                //��Ϊλ��ģʽ���͵�ǰλ�ü�Ĭ���ٶȵ�ָ��            
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
            else        //!< ֻҪ�ϵ���յ���PC��ָ����һ�����м�¼�����һ��ָ�����Ϊ��
            {
                //!< ��Ϊ�ٶ�ģʽ����speed=0��ָ��
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

    //!< ��黺�����Ƿ��д���ָ��
    if(pModule->mControl.pToMotorCmd->dCmdCnt==0)
    {
        if((pModule->mControl.posKeep&0xFF00)!=0)     //!< �յ���Clearָ��
        {
            tmp = (u8)pModule->mControl.posKeep;
            
            if(tmp == 'K')       //!< ��Ҫλ�ñ���
            {
                if(pModule->mControl.mCtrlMode == Ctrl_Mode_Posit)
                {
                    //!< pos mode,do nothing
                    tmpDat[0] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                    tmpDat[1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
            
                    tmpDat[2] = 0x00;
                    tmpDat[3] = 0x00;
                    memcpy(&tmpDat[4],(u8*)(&sysEwServo[Emb_StartID_Shoulder-1].uPosWheel),4);   //!< ȡPosition��4���ֽ��·���ȥ

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
                    modeDat[2] = EWAY_MOTOR_CONTROL_MODE_POSITION;                      //!< λ��ģʽ		
			
			        pModule->mResp.mExeCmRecd[0] += (0x01<<4);    //!< ��¼�ɹ�������һ��Single Write(0x03)��ָ�� ��0x03ָ��ķ��ͼ�¼
                    pModule->mResp.mExeCmRecd[1]  = Ctrl_Mode_Posit;    //!< ��¼�·����л���λ��ģʽ

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
            else if(tmp == 'N')  //!< ����Ҫλ�ñ���
            {    
                  //!< do nothing
            }

            EwayShoulder.mControl.posKeep = 0x0000; 

            return ERR_NONE;
        }
        else    //!< ��λ�ñ��ֵ�ָ���������ʼ����Ƿ���������λ��ѹס
        {
            //!< check Up-Dn Pin Status
            if(((pModule->shLimit.pinState[SHOULDER_LIMIT_DN]&SHOULDER_LIMIT_TRIGGER_BIT)==0)||((pModule->shLimit.pinState[SHOULDER_LIMIT_UP]&SHOULDER_LIMIT_TRIGGER_BIT)==0))
            {
                if(pModule->shLimit.mCtrl.en==0)
                {
                    if(pModule->mControl.mCtrlMode == Ctrl_Mode_Posit)          //!< ѹס����/����λ��λ��ģʽ�����͵�ǰλ��
                    {
                        //!< pos mode,do nothing
                        tmpDat[0] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                        tmpDat[1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
            
                        tmpDat[2] = 0x00;
                        tmpDat[3] = 0x00;
                        memcpy(&tmpDat[4],(u8*)(&sysEwServo[Emb_StartID_Shoulder-1].uPosWheel),4);   //!< ȡPosition��4���ֽ��·���ȥ

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
                    else if(pModule->mControl.mCtrlMode == Ctrl_Mode_Speed)    //!< ѹס������λ���ٶ�ģʽ�������ٶ�Ϊ0��ָ��
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
            else    //!< ��ǰ����/����λ��ѹס
            {
                if(EwayShoulder.shLimit.mCtrl.en!=0)        //!< ֻҪ��ѹ��/����λ����Ϳ������־��
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
        return ERR_NONE;              //!< ���ٶ�λ��ָ�������

    if(mode!=pModule->mControl.mCtrlMode)     //!< ��ǰָ��ģʽ�������ָ��ģʽ��ͬ������ģʽ����ָ������ļ粿�������ģʽ���� 
    {
        if(mode == Ctrl_Mode_Posit)
        {             
            modeDat[0] = EwayMotor_CtrlMode; //!< ���ĵ������ģʽ��reg0x08:0-�ٶȿ���ģʽ,1-λ�ÿ���ģʽ,2-ת�ؿ���ģʽ,3-ռ�ձ�ģʽ
            modeDat[1] = 1;
            modeDat[2] = EWAY_MOTOR_CONTROL_MODE_POSITION;                      //!< λ��ģʽ
            
            EwayShoulder.mResp.mExeCmRecd[0] += (0x01<<4);    //!< ��¼������һ��Single Write(0x03)��ָ�� ��0x03ָ��ķ��ͼ�¼
            EwayShoulder.mResp.mExeCmRecd[1]  = Ctrl_Mode_Posit;    //!< ��¼�·����л���λ��ģʽ
        }
        else if(mode == Ctrl_Mode_Speed)
        {                
            modeDat[0] = EwayMotor_CtrlMode; //!< ���ĵ������ģʽ��reg0x08:0-�ٶȿ���ģʽ,1-λ�ÿ���ģʽ,2-ת�ؿ���ģʽ,3-ռ�ձ�ģʽ
            modeDat[1] = 1;
            modeDat[2] = EWAY_MOTOR_CONTROL_MODE_SPEED;                  //!< �ٶ�ģʽ

            EwayShoulder.mResp.mExeCmRecd[0] += (0x01<<4);    //!< ��¼������һ��Single Write(0x03)��ָ�� ��0x03ָ��ķ��ͼ�¼
            EwayShoulder.mResp.mExeCmRecd[1]  = Ctrl_Mode_Speed;    //!< ��¼�·����л���λ��ģʽ
        }
    }

    if(((pModule->shLimit.pinState[SHOULDER_LIMIT_DN]&SHOULDER_LIMIT_TRIGGER_BIT)!=0)&&((pModule->shLimit.pinState[SHOULDER_LIMIT_UP]&SHOULDER_LIMIT_TRIGGER_BIT)!=0))
    {        
        if(EwayShoulder.shLimit.mCtrl.en!=0)        //!< ֻҪ��ѹ��/����λ����Ϳ������־��
        {
            EwayShoulder.shLimit.mCtrl.en = 0;

            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
            (pDebug->secondFun&SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
            {
                res = Bsp_printf("Sh Cmd !=0,Up/Dn don't be tuched,Clr shLimit.en Suc!");
            }
        }
        
        //<��û��������λ��ѹס����ת��������ָ��.����
        if(mode==Ctrl_Mode_Speed)
        {
            SrcSpd = dat[12] + (dat[13]<<8) + (dat[14]<<16) + (dat[15]<<24);
                       
            //!< ����ٶ�ֵ�Ƿ񳬹��ٶȴ�С����
            sysEmbCheckSendToShoulderMotorParameters(&SrcSpd);
            
            //!< ����id��ȡ�����ٱȲ������·���motor���ٶ�ֵ
            sSpd = (s16)(SrcSpd*(*(pShRa))/MOTOR_SPEED_COEFFICIENT);
            
            EmbEnableLastPosCmdAvailableFlag(pLast,0);

            EmbRecordCmdSendToMotor(0,mode,sSpd,0,EWAYBOT_SHOULDER);    //!< ��¼�������·���ָ��

            if(modeDat[0] != 0) //!< ��ʾ��ģʽ���Ƹı�ָ��Ҫ�·�
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
                       
            //!< ����ٶ�ֵ�Ƿ񳬹��ٶȴ�С����
            sysEmbCheckSendToShoulderMotorParameters(&SrcSpd);
            
            //!< ����id��ȡ�����ٱȲ������·���motor���ٶ�ֵ
            sSpd = (s16)(SrcSpd*(*(pShRa))/MOTOR_SPEED_COEFFICIENT);            
            //!< ����ٶ�ֵ�Ƿ񳬹��ٶȴ�С����

            sPos = dat[8]+(dat[9]<<8)+(dat[10]<<16)+(dat[11]<<24);

            EmbRecordingLastPosCmdSendtoMotor(pLast,0,sPos);//----

            EmbRecordCmdSendToMotor(0,mode,sSpd,sPos,EWAYBOT_SHOULDER);    //!< ��¼�������·���ָ��

            if(modeDat[0] != 0) //!< ��ʾ��ģʽ���Ƹı�ָ��Ҫ�·�
            {
                memcpy(tmpDat,modeDat,3);
                
                tmpDat[3] = EwayMotorReg_TargSpd_L;
                tmpDat[4] = 8;
                                            //!< Reg
                tmpDat[5] = (u8)sSpd;       //!< 0x56
                tmpDat[6] = (u8)(sSpd>>8);  //!< 0x57
                tmpDat[7] = 0x00;           //!< 0x58
                tmpDat[8] = 0x00;           //!< 0x59

                memcpy(&tmpDat[9],&dat[8],4);   //!< ȡPosition��4���ֽ��·���ȥ
                                
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
                memcpy(&tmpDat[4],&dat[8],4);   //!< ȡPosition��4���ֽ��·���ȥ
            
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
        //!< ����������λ��ѹס�����ж�ָ����˶�����        
        //!< ��ǰ��������λ��������λ
        if(mode==Ctrl_Mode_Speed)
        {
            //!< ���·�ָ��Ϊ�ٶ�����������λ��ѹסor�·�ָ��Ϊ�ٶ�����������λ��ѹ�����ճ��·�ָ��
            //!< �����·�brakeָ��
            SrcSpd = dat[12] + (dat[13]<<8) + (dat[14]<<16) + (dat[15]<<24);          //!< ����id��ȡ�����ٱȲ������·���motor���ٶ�ֵ          
                       
            //!< ����ٶ�ֵ�Ƿ񳬹��ٶȴ�С����
            sysEmbCheckSendToShoulderMotorParameters(&SrcSpd);
            
            sSpd = (s16)(SrcSpd*(*(pShRa))/MOTOR_SPEED_COEFFICIENT);    
            
            if(((sSpd>0)&&((pModule->shLimit.pinState[SHOULDER_LIMIT_DN]&SHOULDER_LIMIT_TRIGGER_BIT)==0))||((sSpd<0)&&((pModule->shLimit.pinState[SHOULDER_LIMIT_UP]&SHOULDER_LIMIT_TRIGGER_BIT)==0)))
            {   //!< ��ָ��Ϊ�����˶�������λ��ѹס or ָ��Ϊ�����˶�������λ��ѹס�����ճ�����ָ��
            
                EmbEnableLastPosCmdAvailableFlag(pLast,0);              //!< ��¼���һ��ָ��

                EmbRecordCmdSendToMotor(0,mode,sSpd,0,EWAYBOT_SHOULDER);    //!< ��¼�������·���ָ��

                if(pModule->shLimit.mCtrl.en != UpDnLimShlCtrl_SpdCmd)  //!< ʹ��ѹ��λ���б�־������¼�ٶ�ֵ�Ա�����
                {
                    pModule->shLimit.mCtrl.en = UpDnLimShlCtrl_SpdCmd;
                    
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_EmbIn_ShUpDnLim_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Shoulder_MASK))
                    {
                        res = Bsp_printf("2.1 ShMov-7 Up-Dn touched,speed is available,Enable shLimit.en!");
                    }
                }
                pModule->shLimit.mCtrl.SpdCmd = sSpd;
                
                if(modeDat[0] != 0) //!< ��ʾ��ģʽ���Ƹı�ָ��Ҫ�·�
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
            else    //!< ��ָ��Ϊ�����˶�������λ��ѹס or ָ��Ϊ�����˶�������λ��ѹס����send brake
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
                
                if(modeDat[0] != 0) //!< ��ʾ��ģʽ���Ƹı�ָ��Ҫ�·�
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
            //!< ���·�ָ��Ϊλ������������λ��ѹסor�·�ָ��Ϊλ������������λ��ѹ�����ճ��·�ָ��
            //!< �����·�brakeָ��

            sPos = dat[8] + (dat[9]<<8) + (dat[10]<<16) + (dat[11]<<24);
            
            SrcSpd = dat[12] + (dat[13]<<8) + (dat[14]<<16) + (dat[15]<<24);
                       
            //!< ����ٶ�ֵ�Ƿ񳬹��ٶȴ�С����
            sysEmbCheckSendToShoulderMotorParameters(&SrcSpd);
            
            sSpd = (s16)(SrcSpd*(*(pShRa))/MOTOR_SPEED_COEFFICIENT);  

            //!< ָ��Ϊ�����˶���ѹס������λ or ָ��Ϊ�����˶���ѹס������λ�����ճ���������brake
            if(((sPos > sysEwServo[Emb_StartID_Shoulder-1].uPosWheel)&&((EwayShoulder.shLimit.pinState[SHOULDER_LIMIT_DN]&SHOULDER_LIMIT_TRIGGER_BIT)==0))||((sPos < sysEwServo[Emb_StartID_Shoulder-1].uPosWheel)&&((EwayShoulder.shLimit.pinState[SHOULDER_LIMIT_UP]&SHOULDER_LIMIT_TRIGGER_BIT)==0)))
            {   //!< ��ָ��Ϊ�����˶�������λ��ѹס or ָ��Ϊ�����˶�������λ��ѹס�����ճ�����ָ��
                EmbRecordingLastPosCmdSendtoMotor(pLast,0,sPos);//----

                EmbRecordCmdSendToMotor(0,mode,sSpd,sPos,EWAYBOT_SHOULDER);    //!< ��¼�������·���ָ��
                
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
                
                if(modeDat[0] != 0) //!< ��ʾ��ģʽ���Ƹı�ָ��Ҫ�·�
                {
                    memcpy(tmpDat,modeDat,3);

                    tmpDat[3] = EwayMotorReg_TargSpd_L;
                    tmpDat[4] = 8;
                    tmpDat[5] = (u8)(sSpd);
                    tmpDat[6] = (u8)(sSpd>>8);                    
                    tmpDat[7] = 0x00;
                    tmpDat[8] = 0x00;
                    
                    memcpy(&tmpDat[9],(u8*)(&sPos),4);   //!< ȡPosition��4���ֽ��·���ȥ
                
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
                    memcpy(&tmpDat[4],(u8*)(&sPos),4);   //!< ȡPosition��4���ֽ��·���ȥ
                    
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
                
                if(modeDat[0] != 0) //!< ��ʾ��ģʽ���Ƹı�ָ��Ҫ�·�
                {
                    memcpy(tmpDat,modeDat,3);

                    tmpDat[3] = EwayMotorReg_TargSpd_L;             //!< �����ٶ�Ϊ0��λ��Ϊ��ǰλ�õ�brakeָ��
                    tmpDat[4] = 8;
                    tmpDat[5] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                    tmpDat[6] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);                    
                    tmpDat[7] = 0x00;
                    tmpDat[8] = 0x00;
                    
                    memcpy(&tmpDat[9],(u8*)(&sysEwServo[Emb_StartID_Shoulder-1].uPosWheel),4);   //!< ȡPosition��4���ֽ��·���ȥ
                
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
                    //!< �����ٶ�Ϊ0��λ��Ϊ��ǰλ�õ�brakeָ��
                    tmpDat[0] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                    tmpDat[1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
                    
                    tmpDat[2] = 0x00;
                    tmpDat[3] = 0x00;
                    memcpy(&tmpDat[4],(u8*)(&sysEwServo[Emb_StartID_Shoulder-1].uPosWheel),4);   //!< ȡPosition��4���ֽ��·���ȥ
                    
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



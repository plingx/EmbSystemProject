/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Head_Cmd.c
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
extern EwayHeadModule EwayHeads;
extern EwayEmbSysDebugModule* pDebug;
extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];




s8 sysModuleHeadsMovement(EwayHeadModule* pModule)
{
	u8 tmpDat[12]={0};						//!< ��ʼ��ʱΪ0
	u8 dat[CMD_PC_2_EMB_BUFF_LEN]={0};
	u8 sndDat[30]={0};						//!< motorsNums*regNums 6*4  24byte
	u8 i,slvNums;
	u8 slvID[6];
	s8 res;
    u8 tmp;
	s16 mSpeed;
    s32 tPos;
    u16 mPos;
	MotorCMDBuff* pHdCmd = pModule->mControl.pToMotorCmd;
	MotorReducRatio* pHdRa = pModule->mControl.pMotorRatio;
		
	//!< �������λ��ͨ��״̬����Ϊ�Ͽ����������·��ٶ�Ϊ0��ָ��orɲ����ָ��orλ��ָ���֮����Ҫͣ������
	if((EwayEmbSys.Comm.wPc & 0x01) == EMB_COMM_STATUS_DISCONNECTED)
	{
	    //!< ��鵱ǰ�Ƿ���avail����Ϊʹ�ܵĵ������¼
        slvNums = 0;
	
		for(i=0;i<Motor_Num_Head;i++)
		{
			//!< ����id�ż����Ӧ�����Ƿ���ָ��
			if(((EwayHeads.mControl.pmlstPos+i)->avail) ==  LastPosAvailablePosOn)
			{						
			    slvID[slvNums] = Emb_StartID_Head + i;
					
				sndDat[(slvNums<<2)] = (u8)MOTOR_DEFAULT_SPEED_MIN;	 //!< slvNums��ʶ��ǰҪ�·�ָ��Ķ������ * 4���Ĵ�����Ϊƫ����
				sndDat[(slvNums<<2)+1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);

                if(EmbGetLastPosCmdSendtoMotor(EwayHeads.mControl.pmlstPos,i,&tPos)==ERR_NONE)
                {
                    mPos = (u16)tPos;
                    sndDat[(slvNums<<2)+2] = (u8)mPos;
                    sndDat[(slvNums<<2)+3] = (u8)(mPos>>8);         
                    
                    slvNums += 1;
                }
			}			
		}

        if((pModule->mControl.mCtrlMode == Ctrl_Mode_Posit)&&(slvNums>0))
        {
            res = sysGeneralMotorDiscontinuousBroadcastWrite(slvID,slvNums,EwayMotorReg_TargSpd_L,4,sndDat,EWAYBOT_HEAD);
            
            if(ERR_NONE != res)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    res = Bsp_printf("2.1 hdMov-0 PC Comm disconnected,send CurrPos(0x56-4) failed.");
                }
            }
            else
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
                {
                    res = Bsp_printf("2.1 hdMov-0 PC Comm disconnected,send CurrPos(0x56-4) Suc.");
                }                               
            }  
        }

		return ERR_NONE;
	}
	
	//!< ��ȡ��ǰ�����е�ָ���������ȫ��Ϊ0�������û�д��·���ָ�����ERR_NONE��
	for(i=0;i<pModule->mCount;i++)
	{		
		if(((pHdCmd+i)->dCmdCnt) > 0 ) break;
	}
	
	if(i >= pModule->mCount)       //!< ��ǰָ��buffer���Ѿ���ָ��
	{
        //!< ��鵱ǰ�Ƿ���Clearָ�������
        if((pModule->mControl.posKeep&0xFF00)!=0)     //!< �յ���Clearָ��
        {
            tmp = (u8)pModule->mControl.posKeep;
                    
            if(tmp == 'K')       //!< ��Ҫλ�ñ���
            {
                //!< pos mode,send Current Pos and speed=0 
                tmpDat[0] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                tmpDat[1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
                tmpDat[2] = sysEwServo[Emb_StartID_Head-1].uPosJointL;//!< ȡ��ǰ��Position�·���ȥ    
                tmpDat[3] = sysEwServo[Emb_StartID_Head-1].uPosJointH;

                tmpDat[4] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                tmpDat[5] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
                tmpDat[6] = sysEwServo[Emb_StartID_Head].uPosJointL;//!< ȡ��ǰ��Position�·���ȥ    
                tmpDat[7] = sysEwServo[Emb_StartID_Head].uPosJointH;

                res = sysGeneralMotorBroadcastWrite(Emb_StartID_Head,Motor_Num_Head,EwayMotorReg_TargSpd_L,4,tmpDat,EWAYBOT_HEAD);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                       res = Bsp_printf("2.1 Hd Keep Pos Cmd, Posi mode,send CurrPos(0x56-4) spd=0 failed.");
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
                    {
                        res = Bsp_printf("2.1 Hd Keep Pos Cmd, Posi mode,send CurrPos(0x56-4) spd=0 Suc.");
                    }                               
                }                          
            }
            else if(tmp == 'N')  //!< ����Ҫλ�ñ���        do nothing
            {                /*
                //!< ����λ�û����ٶ�ģʽ������0x51�Ĵ����·�����ָֹͣ��
                tmpDat[0] = 2;                  //!< reg0x51:0-����ֹͣ,1-�����ƶ�,2-����ֹͣ
                tmpDat[1] = 2;
                
                res = sysGeneralMotorBroadcastWrite(Emb_StartID_Head,Motor_Num_Head,EwayMotor_Stop,1,tmpDat,EWAYBOT_HEAD);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("2.1 Hd Not Keep Pos Cmd,BrodcstWr(0x51-2) failed.");
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
                    {
                        res = Bsp_printf("2.1 Hd Not Keep Pos Cmd,BrodcstWr(0x51-2) Suc!");
                    }
                }    */            
            }
        
            pModule->mControl.posKeep = 0x0000; 
        
            return ERR_NONE;
        }
        else
        {
		    return ERR_NONE;
        }    
	}

	//!< ȡָ�� ���ָ��ķ�Χ:�ٶȲ���С��15,���ܴ���???��λ��Ϊ�ű�������ֵ
	if(pModule->mControl.mCtrlMode==Ctrl_Mode_Posit)	  //!< Ĭ������ָ����е�ָ���Ϊλ��ָ�� 
	{
		slvNums = 0;
	
		for(i=0;i<pModule->mCount;i++)
		{
			//!< ����id�ż����Ӧ�����Ƿ���ָ��
			if(((pHdCmd+i)->dCmdCnt) > 0 )
			{
				//res = sysGeneralGetEmbToMotorCmd((pHdCmd+i),dat);	  //!< ȡ��ָ��
				res = g_GetPCCmdEmbToMotorCmd(pHdCmd,i,dat);
	
				if(res==ERR_NONE)
				{				
					//!< ָ��,ʱ���,
					//!< fCode	fTimeStamp ID  Pos	Speed
					//!<   2		4		2	4	  2 
					//!<   2		4		2	2	  2             //!< modify 20180508
						
					slvID[slvNums] = dat[6];						//!< dat[6]&dat[7] ��ŵ���ID�ţ�ȡ���ֽ�
						
					mSpeed = dat[10] + (dat[11]<<8);                //!< mSpeed = dat[12] + (dat[13]<<8);

                    mPos = dat[8] + (dat[9]<<8);
						
					//!< ����ٶ��Ƿ���[(+/-)15,(+/-)MAX],���
					sysEmbCheckSendToJointMotorParameters(&mPos,&mSpeed);

					//!< ����id��ȡ�����ٱȲ������·���motor���ٶ�ֵ
					mSpeed = mSpeed*(*(pHdRa+i))/MOTOR_SPEED_COEFFICIENT;
	
					sndDat[(slvNums<<2)] = (u8)mSpeed;	 //!< slvNums��ʶ��ǰҪ�·�ָ��Ķ������ * 4���Ĵ�����Ϊƫ����
					sndDat[(slvNums<<2)+1] = (u8)(mSpeed>>8);
	
					//!< λ����д����ֵ2�ֽ�
					sndDat[(slvNums<<2)+2] = (u8)mPos;
					sndDat[(slvNums<<2)+3] = (u8)(mPos>>8);			
	
					slvNums += 1;
                    
                    EmbRecordingLastPosCmdSendtoMotor(EwayHeads.mControl.pmlstPos,i,(s32)mPos);

                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&
                    (pDebug->secondFun&SecFunDebugCtrl_EmbIn_MotorLstCmd_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
                    {
                        Bsp_printf("Record HdMotor(%d) lastPosCmd,Pos:0x%x",(i+1),mPos);
                    }
				}				
			}			
		}
	
		//!< ������͵���Ӧ���Ͷ��� 	
		if(slvNums>0)
		{
			res = sysGeneralMotorDiscontinuousBroadcastWrite(slvID,slvNums,EwayMotorReg_TargSpd_L,4,sndDat,EWAYBOT_HEAD);
			if(ERR_NONE != res)
			{
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                    SysLogWrite(LogMajor,"2.1 Head Send to Motor BrodcstWrCmd(0x56-4) failed.rt:%d.",res);

                    res = Bsp_printf("2.1 Head Send to Motor BrodcstWrCmd(0x56-4) failed.rt:%d.",res);
                }
			}
			else
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
                {
                    res = Bsp_printf("Head SndToMCmd(0x56-4).Id:%d,Num:%d.P1:0x%x,S1:0x%x.P2:0x%x,S2:0x%x.",\
                        slvID[0],slvNums,(sndDat[2]+(sndDat[3]<<8)),(sndDat[0]+(sndDat[1]<<8)),(sndDat[6]+(sndDat[7]<<8)),(sndDat[4]+(sndDat[5]<<8)));
                }
			}
		}
	
		return ERR_NONE;
	}
	
	return ERR_NONE;
}


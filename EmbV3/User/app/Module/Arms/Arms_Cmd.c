/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Arms_Cmd.c
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

extern EwayArmModule EwayArms;
extern EwayEmbSysModule EwayEmbSys;
extern EwayEmbSysDebugModule* pDebug;
extern EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];




/*
�ֱ۵���·�ָ������        (Ŀǰ��Ҫ��ʵ��λ��ָ����·�)
(1) �жϵ�ǰ��λ����PCͨ��״̬�Ƿ�������
		��ͨ�Ų��������·�ɲ��ָ��·�ָ��ǰҪ���ǵ�ǰ����Ŀ���ģʽ��λ��or�ٶ����·�(Ĭ��Ϊλ��ģʽ)
		��ͨ�����������(2)

(2) ��ȡ��ǰָ��������
		��Cmd Cnt==0���򷵻�ERR_NONE.
		��Cmd Cnt>0,�����(3)

(3) ȡָ���뵱ǰ�������ģʽ�Ƚ�(�ٶ�orλ��)
		����ͬ����������͵������ģʽ�л�ָ�����������Ͷ��С�����(4)
		����ͬ�������(4)

(4) ������͸����������ָ�����������Ͷ���
	��ѯ6��ָ���:
	a.����ָ����ȡ����Ӧ����(id,position,speed),�����ݵ�����Լ���Posi&Speed������Ӧ��д��ָ���
	b.��ָ�������		

(5) ���ء�


�ֱ��˶���������(20180524�����ִ��Clearָ�����Ƿ�λ�ñ��ֵĹ��ܣ����ٴ��޸�):
(1) �������λ��ͨ���Ƿ�����
        �����������(2)
        ������������2��ͷ������·������ƶ���ָ��(0x51-1)

(2) ���ָ��buff���Ƿ���ָ��
        ����ָ������(4)
        ����ָ������(3)

(3) ��鵱ǰ�Ƿ���Clearָ�������
        ���У����ж��Ƿ���Ҫλ�ñ���
                    ����Ҫλ�ñ��֣���������λ��Ϊ��ǰλ�ã��ٶ�Ϊ0��λ��ָ����Clearָ���־���˳�������ERR_NONE;
                    ������Ҫλ�ñ��֣�����0x51�Ĵ���д����ֹͣ��ָ����Clearָ���־���˳�������ERR_NONE;
                    
        ��û�У����˳�����ERR_NONE;

(4) ��ǰ��ָ�����ָ������
        ��Ϊλ��ָ����·����˳�������ERR_NONE;
        ���������ԣ��˳�������ERR_NONE;


*/
/*
����ֱ��·�ָ�������Ƿ���ȷ
1. PC�·������ֱ۵�ָ�debug�²鿴����ǰ�������Ƿ���ȷ��
	(1) �·�6��ָ������----ok
	(2) �·�4��ָ������----ok

2. PC�·�����۵�ָ�EMB�·�������Ƿ���ȷ��ָ���Ƿ��б������ȷִ?
	(1)  �·����������ȷ----ok
	(2)  ���ִ����ȷ

3.ʣ���������Ŀ:
	(1)  ��ȷ����ٶ�λ�õļ�ֵ

*/

s8 sysModuleArmsMovement(EwayArmModule* pModule,EwayMotor_Device_Type devType)
{
	u8 tmpDat[30]={0};						//!< ��ʼ��ʱΪ0
	u8 dat[CMD_PC_2_EMB_BUFF_LEN]={0};
	u8 sndDat[30]={0};						//!< motorsNums*regNums 6*4  24byte
	u8 i,slvNums;
	u8 slvID[6]={0};
	s8 res;
	u8 startID;
	s16 mSpeed;
    u16 mPos;
	u16 mCnt=0;
    u16 ClrmaskBit;
    u8 tmp=0;
    s32 tPos;
	MotorCMDBuff* pArmCmd;
    Motor_LastPosCmd* pArmLstPos;
	MotorReducRatio* pArmRa = pModule->mControl.pMotorRatio;

	if(devType == EWAYBOT_ARMS_LEFT)
	{
		startID = Emb_StartID_ArmL;
		pArmCmd = pModule->mControl.pToMotorCmd;
		pArmRa = pModule->mControl.pMotorRatio;
		mCnt = EwayArms.mCntLeft;
        ClrmaskBit = 0x0F00;
        pArmLstPos = EwayArms.mControl.pmlstPos;
	}
	else if(devType == EWAYBOT_ARMS_RIGHT)
	{
		startID = Emb_StartID_ArmR;
		pArmCmd = (pModule->mControl.pToMotorCmd) + Motor_Num_ArmLeft;
		pArmRa = (pModule->mControl.pMotorRatio) + Motor_Num_ArmLeft;
		mCnt = EwayArms.mCntRight;
        ClrmaskBit = 0xF000;
        pArmLstPos = EwayArms.mControl.pmlstPos + Motor_Num_ArmLeft;
	}
	else
	{
		return ERR_INPUT_PARAMETERS;
	}
		
	//!< �������λ��ͨ��״̬����Ϊ�Ͽ����������·��ٶ�Ϊ0��ָ��orɲ����ָ��orλ��ָ���֮����Ҫͣ������
	if((EwayEmbSys.Comm.wPc & 0x01) == EMB_COMM_STATUS_DISCONNECTED)
	{/*
	    for(i=0;i<mCnt;i++)
        {
            tmpDat[i] = 1;                  //!< reg0x51:0-����ֹͣ,1-�����ƶ�,2-����ֹͣ
        }   
        
        res = sysGeneralMotorBroadcastWrite(startID,mCnt,EwayMotor_Stop,1,tmpDat,devType);
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("2.1 ArmMov-0 PC Comm disconnected,send(0x51-1) failed.");
            }
        }
        else
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
            {
                //Bsp_printf("2.1 ArmMov-0 PC Comm disconnected,send(0x51-1) Succ!");
            }
        }*/
        //!< ��鵱ǰ�Ƿ���avail����Ϊʹ�ܵĵ������¼
        slvNums = 0;
	
		for(i=0;i<mCnt;i++)
		{
			//!< ����id�ż����Ӧ�����Ƿ���ָ��
			if(((pArmLstPos+i)->avail) ==  LastPosAvailablePosOn)
			{						
			    slvID[slvNums] = dat[6];						//!< dat[6]&dat[7] ��ŵ���ID�ţ�ȡ���ֽ�
					
				sndDat[(slvNums<<2)] = (u8)MOTOR_DEFAULT_SPEED_MIN;	 //!< slvNums��ʶ��ǰҪ�·�ָ��Ķ������ * 4���Ĵ�����Ϊƫ����
				sndDat[(slvNums<<2)+1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);

                if(EmbGetLastPosCmdSendtoMotor(pArmLstPos,i,&tPos)==ERR_NONE)
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
            res = sysGeneralMotorDiscontinuousBroadcastWrite(slvID,slvNums,EwayMotorReg_TargSpd_L,4,sndDat,devType);
            
            if(ERR_NONE != res)
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
                   res = Bsp_printf("2.1 Arm(%d)Mov-0 PC Comm disconnected,send CurPos failed.",(u8)devType);
                }
            }
            else
            {
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_PeriodCommStatus_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                {
                    res = Bsp_printf("2.1 Arm(%d)Mov-0 PC Comm disconnected,send CurPos(0x56-4) Suc.",(u8)devType);
                }                               
            } 
        }
      
		return ERR_NONE;
	}
	
	//!< ��ȡ��ǰ�����е�ָ���������ȫ��Ϊ0�������û�д��·���ָ�����ERR_NONE��	
	for(i=0;i<mCnt;i++)
	{		
		if(((pArmCmd+i)->dCmdCnt) > 0 ) break;
	}
	
	if(i >= mCnt)
	{
        //!< ��鵱ǰ�Ƿ���Clearָ�������
        if((EwayArms.mControl.posKeep&ClrmaskBit)!=0)     //!< �յ���Clearָ��
        {
            tmp = (u8)EwayArms.mControl.posKeep;
                    
            if(tmp == 'K')       //!< ��Ҫλ�ñ���
            {
                //!< pos mode,send Current Pos and speed= minimal value
                for(i=0;i<mCnt;i++)
                {
                    tmpDat[(i<<2)] = (u8)MOTOR_DEFAULT_SPEED_MIN;
                    tmpDat[(i<<2)+1] = (u8)(MOTOR_DEFAULT_SPEED_MIN>>8);
                    tmpDat[(i<<2)+2] = sysEwServo[startID-1+i].uPosJointL;//!< ȡ��ǰ��Position�·���ȥ    
                    tmpDat[(i<<2)+3] = sysEwServo[startID-1+i].uPosJointH;                    
                }

                res = sysGeneralMotorBroadcastWrite(startID,mCnt,EwayMotorReg_TargSpd_L,4,tmpDat,devType);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                       res = Bsp_printf("2.1 Arm(%d) Keep Pos Cmd, Posi mode,send CurrPos(0x56-4) failed.",(u8)devType);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                    {
                        res = Bsp_printf("2.1 Arm(%d) Keep Pos Cmd, Posi mode,send CurrPos(0x56-4) Suc.",(u8)devType);
                    }                               
                }                          
            }
            else if(tmp == 'N')  //!< ����Ҫλ�ñ���   do nothing
            {                /*         
                //!< ����λ�û����ٶ�ģʽ������0x51�Ĵ����·�����ָֹͣ��                
                for(i=0;i<mCnt;i++)
                {
                    tmpDat[i] = 2;              //!< reg0x51:0-����ֹͣ,1-�����ƶ�,2-����ֹͣ
                }
                
                res = sysGeneralMotorBroadcastWrite(startID,mCnt,EwayMotor_Stop,1,tmpDat,devType);
                if(ERR_NONE != res)
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
                        res = Bsp_printf("2.1 Arm(%d) Not Keep Pos Cmd,BrodcstWr(0x51-2) failed.",(u8)devType);
                    }
                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                    (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                    {
                        res = Bsp_printf("2.1 Arm(%d) Not Keep Pos Cmd,BrodcstWr(0x51-2) Suc!",(u8)devType);
                    }
                }                */
            }
        
            EwayArms.mControl.posKeep &= (~ClrmaskBit); 
        
            return ERR_NONE;
        }
        else
        {
		    return ERR_NONE;
        }    
	}
	
	//!< ȡָ�� ���ָ��ķ�Χ:�ٶ����ƣ�λ������
	if(EwayArms.mControl.mCtrlMode==Ctrl_Mode_Posit)	  //!< Ĭ������ָ����е�ָ���Ϊλ��ָ�� 
	{
		slvNums = 0;
	
		for(i=0;i<mCnt;i++)
		{
			//!< ����id�ż����Ӧ�����Ƿ���ָ��
			if(((pArmCmd+i)->dCmdCnt) > 0 )
			{
				res = sysGeneralGetEmbToMotorCmd((pArmCmd+i),dat);	  //!< ȡ��ָ��
	
				if(res==ERR_NONE)
				{				
					//!< ָ��,ʱ���,
					//!< fCode	fTimeStamp ID  Pos	Speed
					//!<   2		4		2	4	  2 
					//!<   2		4		2	2	  2         modified 20180508
						
					slvID[slvNums] = dat[6];						//!< dat[6]&dat[7] ��ŵ���ID�ţ�ȡ���ֽ�
						
					mSpeed = dat[10] + (dat[11]<<8);
					mPos = dat[8] + (dat[9]<<8);
						
					//!< ����ٶ��Ƿ���[(+/-)15,(+/-)MAX],���					
					sysEmbCheckSendToJointMotorParameters(&mPos,&mSpeed);

					//!< ����id��ȡ�����ٱȲ������·���motor���ٶ�ֵ
					mSpeed = mSpeed*(*(pArmRa+i))/MOTOR_SPEED_COEFFICIENT;
	
					sndDat[(slvNums<<2)] = (u8)mSpeed;	 //!< slvNums��ʶ��ǰҪ�·�ָ��Ķ������ * 4���Ĵ�����Ϊƫ����
					sndDat[(slvNums<<2)+1] = (u8)(mSpeed>>8);
	
					//!< λ��ֱ����д����ֵ
					sndDat[(slvNums<<2)+2] = (u8)mPos;
					sndDat[(slvNums<<2)+3] = (u8)(mPos>>8);			
	
					slvNums += 1;

                    EmbRecordingLastPosCmdSendtoMotor(pArmLstPos,i,(s32)mPos);

                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EmbSysInterLogicProc)&&
                    (pDebug->secondFun&SecFunDebugCtrl_EmbIn_MotorLstCmd_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK))
                    {
                        Bsp_printf("Record armMotor(%d) lastPosCmd,Pos:0x%x",(i+1),mPos);
                    }
				}				
			}			
		}
	
		//!< ������͵���Ӧ���Ͷ��� 	
		if(slvNums>0)
		{
			res = sysGeneralMotorDiscontinuousBroadcastWrite(slvID,slvNums,EwayMotorReg_TargSpd_L,4,sndDat,devType);
			if(ERR_NONE != res)
			{
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                {
				    res = Bsp_printf("2.1 Arm(%d) Send PosCmd to Motor BrodcstWr() failed.",(u8)devType);
                }
			}
			else
			{
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                {
                    if(((devType==EWAYBOT_ARMS_LEFT)&&(pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK))||((devType==EWAYBOT_ARMS_RIGHT)&&(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                    {
                        res = Bsp_printf("Arm(%d)SndPosCmdToM,Num:%d.P1:0x%x,S1:0x%x.P2:0x%x,S2:0x%x.P3:0x%x,S3:0x%x.P4:0x%x,S4:0x%x.P5:0x%x,S5:0x%x.P6:0x%x,S6:0x%x.",\
                              (u8)devType,slvNums,(sndDat[2]+(sndDat[3]<<8)),(sndDat[0]+(sndDat[1]<<8)),(sndDat[6]+(sndDat[7]<<8)),(sndDat[4]+(sndDat[5]<<8)),\
                              (sndDat[10]+(sndDat[11]<<8)),(sndDat[8]+(sndDat[9]<<8)),(sndDat[14]+(sndDat[15]<<8)),(sndDat[12]+(sndDat[13]<<8)),\
                              (sndDat[18]+(sndDat[19]<<8)),(sndDat[16]+(sndDat[17]<<8)),(sndDat[22]+(sndDat[23]<<8)),(sndDat[20]+(sndDat[21]<<8)));
                    }
                    
                }
		    }
        }
	
		return ERR_NONE;
	}
	
	return ERR_NONE;
}




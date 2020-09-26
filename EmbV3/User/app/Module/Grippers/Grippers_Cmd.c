/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Grippers_Cmd.c
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
#include "Emb_Gripper.h"


extern EwayEmbSysModule EwayEmbSys;
extern EwayEmbSysDebugModule* pDebug;
extern DYNAMIXEL_SERVO_STATUS sysDyServo[SYS_MAX_DYNAMIXEL_SERVO_NUMS];



/*
(1) �������λ��ͨ��:
		��ͨ�Ų��������·�ֹͣ�˶�ָ��
		��ͨ�������������(2)

(2) ���ÿ�������ָ����Ƿ��д�����ָ��:
		������ָ��,�򷵻�ERR_NONE;
		����ָ������(3)

(3) �жϵ�ǰצ�ӵĿ���ģʽ�Ƿ�Ϊλ���ٶ�ģʽ:
		������,�򲻴���
		���ǣ������(4)

(4) ����·�ָ��
		��������Ƿ���(���·���λ�ü��ٶ���ֵ�Ƿ񳬳��޶���Χ)
	


����ERR_NONE;

*/
s8 sysModuleGrippersMovement(EwayGripperModule* pModule)
{
	u8 dat[CMD_PC_2_EMB_BUFF_LEN]={0};
	u8 i;
	s8 res;
	u8 slvID[2]={0};
	u8 sndDat[10]={0};
	u8 slvNums;
	u16 mSpeed,mPos;
	//float mTmp;
	MotorCMDBuff* pGpCmd = pModule->mControl.pToMotorCmd;
	
	if((EwayEmbSys.Comm.wPc & 0x01) == EMB_COMM_STATUS_DISCONNECTED)
	{
		//!< ������Ͽ�ʱ����צ�ӷ���ָֹͣ�����צ��λ��дΪ��ǰλ�ã��ٶ�Ϊ0
		/*
		for(i=0;i<Motor_Num_Gripper;i++)
        {
            sndDat[(i<<2)] = sysDyServo[i].unCurPosL;
            sndDat[(i<<2)+1] = sysDyServo[i].unCurPosH;
            sndDat[(i<<2)+2] = 0;
            sndDat[(i<<2)+3] = 0;
            
            slvID[i] = Emb_StartID_Gripper + i;
        }
        
        slvNums = Motor_Num_Gripper;
        
		res = sysSendDynamixelBroadcastSyncWritePacket(EWAYBOT_GRIPPERS,DYNAMIXEL_REG_GoalPosL,4,slvID[0],slvNums,sndDat);
	
        if(ERR_NONE != res)
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                res = Bsp_printf("2.1 GripMov-0 PC Comm disconnected,send CurPos,spd=0 failed.");
            }
        }
        else
        {
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Head_MASK))
            {
                //Bsp_printf("2.1 GripMov-0 PC Comm disconnected,sendCurPos,spd=0 Succ!");
            }
        }*/
        
		return ERR_NONE;
	}
	
	for(i=0;i<(pModule->mCount);i++)
	{		
		if(((pGpCmd+i)->dCmdCnt) > 0 ) break;
	}
	
	if(i >= pModule->mCount)
	{
		return ERR_NONE;
	}

	if(pModule->mControl.mCtrlMode != Ctrl_Mode_Posit)
	{
		//!< Ŀǰֻ����λ��ģʽ��ָ����������צ�ӵ�ת�ؿ��غ�ת�����ƹ���
		return ERR_NONE;
	}

	slvNums = 0;
	for(i=0;i<(pModule->mCount);i++)
	{
		//!< ����id�Ų�ѯ��Ӧָ����Ƿ��д���ָ��
		if(((pGpCmd+i)->dCmdCnt) > 0 )
		{
			res = sysGeneralGetEmbToMotorCmd((pGpCmd+i),dat);	  //!< ȡ��ָ��
	
			if(res==ERR_NONE)
			{				
				//!< ָ��,ʱ���,
				//!< fCode	fTimeStamp ID  Pos	Speed
				//!<   2		4		2	2	  2 
						
				slvID[slvNums] = dat[6];						      //!< dat[6]&dat[7] ��ŵ���ID�ţ�ȡ���ֽ�

				mPos = dat[8] + (dat[9]<<8);
				//!< ���λ���Ƿ���   0-4096
				if(mPos>DYNAMIXEL_POSITION_MAX) 
				{
					continue;                                        //!< λ�ó��޲��·���ָ��
				}

				//!< дλ��				
				sndDat[slvNums<<2] = (u8)mPos;                      //!< slvNums��ʶ��ǰҪ�·�ָ��Ķ������ * 4���Ĵ�����Ϊƫ����
				sndDat[(slvNums<<2)+1] = (u8)(mPos>>8);
				
				mSpeed = dat[10] + (dat[11]<<8);					//!< RPMת��Ϊצ�ӵ��ٶȼĴ�����ֵ  RPM = 0.114*RegisterValue
				//mSpeed = (u16)((float)mSpeed/0.114);
				//Tmp = ((float)(mSpeed))/0.114;
				//mSpeed = (u16)mTmp;
				mSpeed = mSpeed/11.4;            //!< ��λ���·����ٶ�������100����rpm,��˻�Ҫ����100,��(0.114*100)   20180508 modify ling
				
                if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
                (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Gripper_MASK))
                {
                    Bsp_printf("Snd to Gripper(%d) Pos:0x%x,Spd:0x%x.",dat[6],(dat[8] + (dat[9]<<8)),(dat[10] + (dat[11]<<8)));
                }

				if((mSpeed==0)||(mSpeed>DYNAMIXEL_SPEED_JOINT_MAX))	//!< ����ٶ��Ƿ���
				{
					mSpeed = DYNAMIXEL_SPEED_DEFAULT;
				}
				
				//!< д�ٶ�
				sndDat[(slvNums<<2)+2] = (u8)mSpeed;
				sndDat[(slvNums<<2)+3] = (u8)(mSpeed>>8);	
					
				slvNums += 1;
			}				
		}
	}

	if(slvNums>0)
	{
		res = sysSendDynamixelBroadcastSyncWritePacket(EWAYBOT_GRIPPERS,DYNAMIXEL_REG_GoalPosL,4,slvID[0],slvNums,sndDat);
		if(ERR_NONE != res)
		{
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
			    SysLogWrite(LogMajor,"GripperSndSyncWrite() failed.rt:0x%x.",res);

			    res = Bsp_printf("GripperSndSyncWrite() failed.rt:0x%x.",res);
            }
		}
		else
		{
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Emb_Proc_PC_Cmd_MASK)&&\
            (pDebug->secondFun&SecFunDebugCtrl_XMovementProcess_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Gripper_MASK))
            {
                if(slvNums==1)
                {                
                    res = Bsp_printf("GripperSndSyncWrite,id0:0x%x,Pos:0x%x,Spd:0x%x.",slvID[0],(sndDat[0]+(sndDat[1]<<8)),(sndDat[2]+(sndDat[3]<<8)));
                }
                else if(slvNums==2)
                {                
                    res = Bsp_printf("GripperSndSyncWrite,id0:0x%x,Pos:0x%x,Spd:0x%x,id1:0x%x,Pos:0x%x,Spd:0x%x.",slvID[0],(sndDat[0]+(sndDat[1]<<8)),(sndDat[2]+(sndDat[3]<<8)),slvID[1],(sndDat[4]+(sndDat[5]<<8)),(sndDat[6]+(sndDat[7]<<8)));
                }             
            }
		}
	}

	return ERR_NONE;
}


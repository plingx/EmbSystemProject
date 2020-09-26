/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Emb_MyServo.c
* @brief ������ݵ��շ����ȫ�ּ����غ���
* @details ��˾����շ����ݵ����Э����룬�����������Э�����ݵ�������������ӵ���յ������ݡ�
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2017-12-20
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2017/12/28 | 0.0.1 | LingPing | Create file
*
*/

#include "includes.h"
#include "Emb_MyServo.h"




extern EwayEmbSysModule EwayEmbSys;
extern EwayArmModule EwayArms;
extern EwayShoulderModule EwayShoulder;
extern EwayHeadModule EwayHeads;
extern EwayGripperModule EwayGrippers;
extern EwayWheelModule EwayWheels;
extern EwayEmbSysDebugModule* pDebug;


extern __IO u32 SysTimeStamp;
extern COMRECVSTRUCT ComRecv[];
extern u8 MotorCfgReg[SYS_MAX_GENERAL_SERVO_NUMS][GENERAL_SERVO_CFG_REGS];


EWAYBOT_SERVO_STATUS sysEwServo[SYS_MAX_GENERAL_SERVO_NUMS];
u8 ArmSvdelayTm[6]={0x01,0x11,0x21,0x31,0x41,0x51};
const float MotorReductionRatio[6]={MOTOR_tA1,MOTOR_tA2,MOTOR_tA3,MOTOR_tA4,MOTOR_tALift,MOTOR_tWhel};



s8 g_GetMotorDeviceTypeAccordingAddr(u8 addr,EwayMotor_Device_Type* devTy);
s8 sysGetMotorSendRecordPointer(EwayMotor_Device_Type devType,MotorRecordBuff** pMRec,u8* max_iD,u8* startId);





/* --------------------------------------------------------------------------*/
/**
* @name sysGetMotorCommPort
* @brief 
* @details Get the com port according to device type
*
* @param[in] devType Eway Motor device type:arms,shoulder,head,wheels and so on.
* @param[in] pCom Pointer of com port to return.
*
* @returns ERR_NONE Successful
* others Failed
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysGetMotorCommPort(EwayMotor_Device_Type devType,COM_PORT_E* pCom)
{
    s8 res=ERR_NONE;
    
    switch(devType)
    {
        case EWAYBOT_ARMS_LEFT:
                *pCom = COM3;            
            break;
        case EWAYBOT_ARMS_RIGHT:
                *pCom = COM4;
            break;
        case EWAYBOT_SHOULDER:
                *pCom = COM2;
            break;
        case EWAYBOT_HEAD:
                *pCom = COM2;
            break;
        case EWAYBOT_GRIPPERS:
                *pCom = COM5;
            break;
        case EWAYBOT_WHEEL:
                *pCom = COM6;
            break;
        case EWAYBOT_PSM:
                *pCom = COM7;
            break;
        case EWAYBOT_PSU:
                *pCom = COM7;
            break;
        default:
                res = ERR_INPUT_PARAMETERS;
            break;
    }    
    return res;
}



/* --------------------------------------------------------------------------*/
/**
* @name sysGetGeneralMotorValidRecvdRegs
* @brief 
* @details Get the com port according to device type
*
* @param[in] devType:    Eway Motor device type:arms,shoulder,head,wheels and so on.
* @param[in] pCom:    Pointer of com port to return.
*            pbuf:    
*            length:    
*            addr:    �յ���Ӧ����е��豸addr,�����豸addrȷ���յ��豸�����ͣ�
*
* @returns ERR_NONE Successful
* others Failed
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysGetGeneralMotorValidRecvdData(EwayMotor_Device_Type devType,u8 addr,u8 fcode,u8* pbuf,u8 length)
{
    u8 id=0;
    s8 res=0;
    EwayMotor_Device_Type dev;        
    
    if(ERR_NONE!=g_GetMotorDeviceTypeAccordingAddr(addr,&dev))     //!< check addr and devType comX�Ƿ��Ӧ
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    if(dev!=devType)                                               //!< Shoulder and Head shared COM2
    {
        if((devType==EWAYBOT_SHOULDER)||(devType==EWAYBOT_HEAD))
        {
            if((dev!=EWAYBOT_SHOULDER)&&(dev!=EWAYBOT_HEAD))
            {
                return ERR_INPUT_PARAMETERS;
            }
        }
        else if((devType==EWAYBOT_WHEEL)&&(dev==EWAYBOT_ARMS_LEFT))  //!< ����ID�����ID���ص�
        {
            dev = EWAYBOT_WHEEL;
        }
        else
        {
            return ERR_INPUT_PARAMETERS;
        }        
    }    

    id = addr-1;                                                  //!< addr value have checked in the upper level function

    switch(fcode)
    {
        case MY_SERVO_Ping_H:         
        case MY_SERVO_SetZero_H:
        case MY_SERVO_Reborn_H:
        case MY_SERVO_SetInit_H:
            
            res = ERR_NONE;
            
            break;
            
        case MY_SERVO_Write_H:

            if(devType!=EWAYBOT_WHEEL)
            {
                if((addr == Emb_StartID_Shoulder)&&(devType == EWAYBOT_SHOULDER))
                {
                    EwayShoulder.mResp.mExeCmRecd[0] += 1 ;        //!< ��0x03ָ��Ľ��ռ�¼

                    //!< ������Ӧ�����ͨ��״̬
                    EwayShoulder.mCommStatus |= 0x0001;    //!< ling-20181123
                }
            }
            else
            {
                if((addr>=Emb_StartID_Wheel)&&(addr<(Emb_StartID_Wheel+Motor_Num_Wheel)))
                {
                    EwayWheels.mResp[id].mExeCmRecd[0] += 1 ;        //!< ��0x03ָ��Ľ��ռ�¼
                    //Bsp_printf("recv wheel(%d) 0x03 Resp!",id);

                    //!< ������Ӧ�����ͨ��״̬
                    EwayWheels.mCommStatus |= 0x0001;    //!< ling-20181128
                }
            }           
            
            break;
            
        case MY_SERVO_Read_H:
            if(devType!=EWAYBOT_WHEEL)
            {
                if(((id<12)||(id==12)||(id==13)||(id==14))&&(fcode==MY_SERVO_Read_H)&&(length<GENERAL_SERVO_CFG_REGS))
                {                
                    memcpy(&MotorCfgReg[id][0],pbuf,length);

                    //!< ����Ӧ���ձ�־λ
                    EwayEmbSys.genmCommSta.mCfgRegCommSta |= (0x00000001<<id);

                    //!< ������Ӧ�����ͨ��״̬
					if(id==12)    //!< ling-20181123
                    {
                        EwayShoulder.mCommStatus |= 0x0001;
                    }
					
					if((id==13)||(id==14))
					{
						EwayHeads.mCommStatus |= 0x0001<<(id-13);
					}
                    
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Motor_2_Emb_MASK)\
                    &&(pDebug->secondFun&SecFunDebugCtrl_SinglReadResp_MASK))
                    {
                        res = Bsp_printf("recv MY_SERVO_Read_H pkt.id(1-15):%d,regNum:%d.",addr,length);  
                    }                  
                }
            }
            else
            {
                if((id<2)&&(fcode==MY_SERVO_Read_H)&&(length<GENERAL_SERVO_CFG_REGS))
                {                
                    memcpy(&MotorCfgReg[id+15][0],pbuf,length);
                    
                    EwayEmbSys.genmCommSta.mCfgRegCommSta |= (0x00000001<<(id+15));         

                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Motor_2_Emb_MASK)\
                    &&(pDebug->secondFun&SecFunDebugCtrl_SinglReadResp_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                    {
                        res = Bsp_printf("recv Wheel MY_SERVO_Read_H pkt.id:%d,regNum:%d.",addr,length); 
                    }
                }
            }
            
            res = ERR_NONE;
            
            break;
        case MY_SERVO_BROADCAST_READ:
            if((dev==EWAYBOT_ARMS_LEFT)||(dev==EWAYBOT_ARMS_RIGHT))   //����ұ�
            {
                if((length == EMB_READ_GENERAL_MOTOR_CFG_REG_NUM)&&(id<Motor_Num_Arm))
                {
                    memcpy(&MotorCfgReg[id][0],pbuf,length);

                    EwayEmbSys.genmCommSta.mCfgRegCommSta |= (0x00000001<<id);   //!< ����Ӧ���ձ�־λ      

                    //Bsp_printf("rec cfgs,ID:%d,RR:%d",id,(pbuf[4]+(pbuf[5]<<8))); 

                    res = ERR_NONE;
                }
                else if((length == SERVO_REG_READ_NUM_ARM)&&(id<Motor_Num_Arm))
                {
                    sysEwServo[id].uSpdRPML   = *(pbuf+11);
                    sysEwServo[id].uSpdRPMH   = *(pbuf+12);
                    memcpy(&((EwayArms.mState.pFromMotor+id)->mRegs[0]),pbuf,SERVO_REG_READ_NUM_ARM);
                        
                    //!< ������Ӧ�����ͨ��״̬
                    EwayArms.mCommStatus |= (0x0001<<id);
                    
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Motor_2_Emb_MASK)\
                    &&(pDebug->secondFun&SecFunDebugCtrl_BroadReadResp_MASK)&&((pDebug->jointSw&JointSwDebugCtrl_ArmLeft_MASK)||(pDebug->jointSw&JointSwDebugCtrl_ArmRight_MASK)))
                    {
                        res = Bsp_printf("recv Arm MY_SERVO_BROADCAST_READ pkt.id(1-15):%d,regNum:%d.",addr,length);            
                    }
                    
                    res = ERR_NONE;
                }                
                else
                {
                    res = ERR_INPUT_PARAMETERS;
                }
            }
            else if(dev == EWAYBOT_SHOULDER)
            {
                if((length == EMB_READ_GENERAL_MOTOR_CFG_REG_NUM)&&(addr == Emb_StartID_Shoulder))
                {
                    memcpy(&MotorCfgReg[id][0],pbuf,length);

                    EwayEmbSys.genmCommSta.mCfgRegCommSta |= (0x00000001<<id);   //!< ����Ӧ���ձ�־λ

                    //Bsp_printf("rec cfgs,ID:%d,RR:%d",id,(pbuf[4]+(pbuf[5]<<8)));

                    res = ERR_NONE;
                }
                else if((length == SERVO_REG_READ_NUM_HEAD_SHOULDER)&&(addr == Emb_StartID_Shoulder))
                {
                    sysEwServo[id].uSpdRPML   = *(pbuf+9);
                    sysEwServo[id].uSpdRPMH   = *(pbuf+10);
                    sysEwServo[id].uPosWheel  = (*(pbuf+15))+((*(pbuf+16))<<8)+((*(pbuf+17))<<16)+((*(pbuf+18))<<24);

                    memcpy(&(EwayShoulder.mState.pFromMotor->mRegs[0]),pbuf,SERVO_REG_READ_NUM_HEAD_SHOULDER);

                    //!< ������Ӧ�����ͨ��״̬
                    EwayShoulder.mCommStatus |= 0x0001;
                     
                    //res = Bsp_printf("recv MY_SERVO_BROADCAST_READ pkt.id(1-15):%d,regNum:%d.",addr,length);            
                    
                    res = ERR_NONE;
                }
                else if((length == EMB_READ_WHEELMODE_POS_PID_REG_NUM)&&(addr == Emb_StartID_Shoulder))
                {
                    
                }
                else
                {                    
                    res = ERR_INPUT_PARAMETERS;
                }
            }
            else if(dev == EWAYBOT_HEAD)
            {
                if((length == EMB_READ_GENERAL_MOTOR_CFG_REG_NUM)&&(id>Motor_Num_Arm)&&(id<(Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head)))
                {
                    memcpy(&MotorCfgReg[id][0],pbuf,length);

                    EwayEmbSys.genmCommSta.mCfgRegCommSta |= (0x00000001<<id);   //!< ����Ӧ���ձ�־λ

                    //Bsp_printf("rec cfgs,ID:%d,RR:%d.",id,(pbuf[4]+(pbuf[5]<<8)));
                    
                    res = ERR_NONE;
                
                }
                else if((length == SERVO_REG_READ_NUM_HEAD_SHOULDER)&&(id>Motor_Num_Arm)&&(id<(Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head)))
                {
                    sysEwServo[id].uSpdRPML   = *(pbuf+11);
                    sysEwServo[id].uSpdRPMH   = *(pbuf+12);
                    sysEwServo[id].uPosWheel  = (*(pbuf+15))+((*(pbuf+16))<<8)+((*(pbuf+17))<<16)+((*(pbuf+18))<<24);

                    if((addr>=Emb_StartID_Head)&&(addr<(Emb_StartID_Head+Motor_Num_Head)))       //!< 13,14
                    {
                        memcpy((EwayHeads.mState.pFromMotor+(addr-Emb_StartID_Head)),pbuf,SERVO_REG_READ_NUM_HEAD_SHOULDER);
                    } 

                    //!< ������Ӧ�����ͨ��״̬
                    EwayHeads.mCommStatus |= 0x0001<<(addr-Emb_StartID_Head);
                  
                    //res = Bsp_printf("recv MY_SERVO_BROADCAST_READ pkt.id(1-15):%d,regNum:%d.",addr,length);
                    
                    res = ERR_NONE;
                }
                else
                {                    
                    res = ERR_INPUT_PARAMETERS;
                }
            }
            else if(dev == EWAYBOT_WHEEL)
            {
                if((length == EMB_READ_WHEEL_MOTOR_RDY_REG_NUM)&&(id<Motor_Num_Wheel))
                {
                    if(*pbuf!=0)
                    {
                        EwayWheels.mRdyStatus |= (0x01<<(id));
                    }

                    res = ERR_NONE;
                    
                    Bsp_printf("rec Whl(%d) Rdy reg(%d).",addr,*pbuf);
                }
                else if((length == EMB_READ_GENERAL_MOTOR_CFG_REG_NUM)&&(id<Motor_Num_Wheel))
                {
                    memcpy(&MotorCfgReg[id+15][0],pbuf,length);

                    EwayEmbSys.genmCommSta.mCfgRegCommSta |= (0x00000001<<(id+15));   //!< ����Ӧ���ձ�־λ

                    //Bsp_printf("rec Whl cfgregs pkt.ID:%d,RR:%d.",id,(pbuf[4]+(pbuf[5]<<8)));

                    res = ERR_NONE;
                }
                else if((length == SERVO_REG_READ_NUM_WHEEL)&&(id<Motor_Num_Wheel))//COM6�����ӣ�����id��1,2
                {
                    sysEwServo[id+15].uStaL      = *pbuf;
                    sysEwServo[id+15].uStaH      = *(pbuf+1);
                    sysEwServo[id+15].uinVolL    = *(pbuf+4);
                    sysEwServo[id+15].uinVolH    = *(pbuf+5);
                    sysEwServo[id+15].uinCurtL   = *(pbuf+6);
                    sysEwServo[id+15].uinCurtH   = *(pbuf+7);
                    sysEwServo[id+15].udevTmpt   = *(pbuf+8);
                    sysEwServo[id+15].uSpdRPML   = *(pbuf+11);
                    sysEwServo[id+15].uSpdRPMH   = *(pbuf+12);
                    sysEwServo[id+15].uPosWheel   = (*(pbuf+15))+((*(pbuf+16))<<8)+((*(pbuf+17))<<16)+((*(pbuf+18))<<24);

                    memcpy((EwayWheels.mState.pFromMotor+(addr-Emb_StartID_Wheel)),pbuf,SERVO_REG_READ_NUM_WHEEL);                    

                    //!< ������Ӧ�����ͨ��״̬
                    EwayWheels.mCommStatus |= 0x0001<<id;
                    
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Motor_2_Emb_MASK)\
                    &&(pDebug->secondFun&SecFunDebugCtrl_BroadReadResp_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Wheel_MASK))
                    {
                        res = Bsp_printf("recv wheel MY_SERVO_BROADCAST_READ pkt.ID:%d,regNum:%d",addr,length);
                    }

                    res = ERR_NONE;

                }
                else
                {                    
                    res = ERR_INPUT_PARAMETERS;
                }
            }

            if((res == ERR_NONE)&&(dev!=EWAYBOT_WHEEL)&&(length != EMB_READ_GENERAL_MOTOR_CFG_REG_NUM))
            {
                if(id<(Motor_Num_Arm+Motor_Num_Shoulder+Motor_Num_Head))
                {
                    sysEwServo[id].uStaL      = *pbuf;
                    sysEwServo[id].uStaH      = *(pbuf+1);
                    sysEwServo[id].uinVolL    = *(pbuf+4);
                    sysEwServo[id].uinVolH    = *(pbuf+5);
                    sysEwServo[id].uinCurtL   = *(pbuf+6);
                    sysEwServo[id].uinCurtH   = *(pbuf+7);
                    sysEwServo[id].udevTmpt   = *(pbuf+8);
                    sysEwServo[id].uPosJointL = *(pbuf+13);
                    sysEwServo[id].uPosJointH = *(pbuf+14);
                }
            }
            
            break;
        default :res = ERR_INPUT_PARAMETERS;
            break;
    }

    return res;
    
}

s8 l_GetGeneralMotorValidFunctionCode(u8 fcode)
{
    s8 res=ERR_NONE;
    
    switch(fcode)
    {        
        case MY_SERVO_Ping_H:
        case MY_SERVO_Read_H:
        case MY_SERVO_Write_H:
        case MY_SERVO_BROADCAST_READ:
        case MY_SERVO_BROADCAST_WRITE:
        case MY_SERVO_SetZero_H:
        case MY_SERVO_Reborn_H:
        case MY_SERVO_SetInit_H:
            break;
        default: res = ERR_INPUT_PARAMETERS;
            break;
    }

    return res;
}

//�ɸ���ʵ������޸�
//id:
//1~6      left arm motor     
//7~12     right arm motor
//13       shoulder motor     
//14-15    head motor 
s8 g_GetMotorDeviceTypeAccordingAddr(u8 addr,EwayMotor_Device_Type* devTy)
{    
    if((addr>=1)&&(addr<=6))                    //addr value have checked in the upper level function
    {
        *devTy = EWAYBOT_ARMS_LEFT;
    }
    else if((addr>=7)&&(addr<=12))
    {
        *devTy = EWAYBOT_ARMS_RIGHT;
    }
    else if(addr==13)
    {
        *devTy = EWAYBOT_SHOULDER;
    }
    else if((addr>=14)&&(addr<=15))
    {
        *devTy = EWAYBOT_HEAD;
    }
    else
    {
        return ERR_INPUT_PARAMETERS;
    }

    return ERR_NONE;
}

/* --------------------------------------------------------------------------*/
/**
* @name g_GetMotorRecvdData
* @brief 
* @details Local function,Get the data received from motors com port.
*
* @param[in] pdat Pointer to store data received from dma uart rx buffer in 
* corresponding Com port
* @param[in] plen Pointer of data length if received data.
* @param[in] pComReceive Pointer of ComX received struct
*
* @returns ERR_NONE Successful
* others Failed
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 g_GetMotorRecvdData(u8* pdat,u16* plen,COMRECVSTRUCT * pComReceive)
{
    u16 len1,len2;

    if(pComReceive->RxCount<=COM_RX_BUFFER_SINGLE_OP_SIZE)
    {
        *plen = pComReceive->RxCount;
        
        if(pComReceive->pRxWrite>=pComReceive->pRxRead)      //!< ����Ĵ洢˳��Ϊ������ʽ
        {
            memcpy(pdat,pComReceive->pRxRead,pComReceive->RxCount);
            
            pComReceive->pRxRead = pComReceive->pRxWrite;            
            pComReceive->RxCount = 0;            
        }
        else                                                //!< ����Ĵ洢˳��Ϊ���������ָ�ʽ
        {
            len1 = (pComReceive->pBuffer+COM_RX_BUFFER_SIZE)-pComReceive->pRxRead;
            len2 = pComReceive->RxCount - len1;
            
            memcpy(pdat,pComReceive->pRxRead,len1);
            memcpy((pdat+len1),pComReceive->pBuffer,len2);

            pComReceive->pRxRead = pComReceive->pRxWrite;
            pComReceive->RxCount = 0;            
        }
    }
    else//!< ���������ݴ���256bytes,��ֻ����256�ֽ�
    {
        if(pComReceive->pRxWrite>=pComReceive->pRxRead)      //!< ����Ĵ洢˳��Ϊ������ʽ
        {
            memcpy(pdat,pComReceive->pRxRead,COM_RX_BUFFER_SINGLE_OP_SIZE);
            
            pComReceive->pRxRead += COM_RX_BUFFER_SINGLE_OP_SIZE;    
            pComReceive->RxCount -= COM_RX_BUFFER_SINGLE_OP_SIZE;
        }
        else                                                //!< ����Ĵ洢˳��Ϊ���������ָ�ʽ
        {
            len1 = (pComReceive->pBuffer+COM_RX_BUFFER_SIZE)-pComReceive->pRxRead;
            
            if(len1>COM_RX_BUFFER_SINGLE_OP_SIZE)
            {
                memcpy(pdat,pComReceive->pRxRead,COM_RX_BUFFER_SINGLE_OP_SIZE);
                
                pComReceive->pRxRead += COM_RX_BUFFER_SINGLE_OP_SIZE;    
                pComReceive->RxCount -= COM_RX_BUFFER_SINGLE_OP_SIZE;
            }
            else
            {
                len2 = COM_RX_BUFFER_SINGLE_OP_SIZE-len1;
                
                memcpy(pdat,pComReceive->pRxRead,len1);
                memcpy((pdat+len1),pComReceive->pBuffer,len2);
                
                pComReceive->pRxRead = pComReceive->pBuffer+len2;    
                pComReceive->RxCount -= COM_RX_BUFFER_SINGLE_OP_SIZE;
            }

            *plen = COM_RX_BUFFER_SINGLE_OP_SIZE;
        }
    }

    return ERR_NONE;    
}


/* --------------------------------------------------------------------------*/
/**
* @name sysParseGeneralMotorRecvdData
* @brief 
* @details To analysis the data received from Com port.
*
* @param[in] pbuf Pointer to data buffer ready to analysis.
* @param[in] length Data length ready to analysis.
* @param[in] devType Device Type:arms,head,shoulder,wheels and so on.
*
* @returns ERR_NONE Successful
* others Failed
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysParseGeneralMotorRecvdData(u8* pbuf,u16 length,EwayMotor_Device_Type devType)
{
    u16 i=0;
    u16 len,rCrc0,rCrc1;
    u16 hder=0;
    u8 sAddr,fCode;//,sta;

    if(length>COM_RX_BUFFER_SINGLE_OP_SIZE)          //packet size exceeded,error
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    while(i<length)                                   //parse the msg packet
    {
        hder = (pbuf[i])+((pbuf[i+1])<<8);                //search the header        
        
        if(hder!=COMM_SERVO_MSG_HEADER) 
        {
            i += 1; continue;
        }
        else
        {            
            sAddr = pbuf[i+2];                        //check slave addr
            
            if((sAddr>(Motor_Num_Arm+Motor_Num_Head+Motor_Num_Shoulder))||(sAddr==0)) //Ŀǰֻ��15�����
            {
                i += 1; continue;
            }            
            
            len = *(pbuf+i+3);                        //check lenght

            if(len<COMM_SERVO_RECVD_MSG_LENGTH_MINI)
            {
                i += 1; continue;
            }            
            
            rCrc0 = CRC16_Modbus((pbuf+i), (len-2));  //check crc
            rCrc1 = (pbuf[i+len-2])+((pbuf[i+len-1])<<8);
            
            if(rCrc0!=rCrc1)
            {
                i += 1; continue;
            }

            fCode = *(pbuf+i+4);                     //check function code 

            if(ERR_NONE!=l_GetGeneralMotorValidFunctionCode(fCode))
            {
                i += 1; continue;
            }

            //sta = *(pbuf+i+5);                         //����
            
            if(ERR_NONE!=sysGetGeneralMotorValidRecvdData(devType,sAddr,fCode,(pbuf+i+6),(len-COMM_SERVO_RECVD_MSG_LENGTH_MINI)))
            {
                i += 1; continue;
            }
            else
            {
                i += len; continue;
            }
        }

    }

    return ERR_NONE;
}

s8 sysParseGeneralMotorRecvdRegs(u8* pbuf,u16 length,EwayMotor_Device_Type devType)
{
    u16 i=0;
    u16 len,rCrc0,rCrc1;
    u16 hder=0;
    u8 sAddr,fCode;

    if(length>COM_RX_BUFFER_SINGLE_OP_SIZE)          //packet size exceeded,error
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    while(i<length)                                   //parse the msg packet
    {
        hder = (pbuf[i])+((pbuf[i+1])<<8);                //search the header        
        
        if(hder!=COMM_SERVO_MSG_HEADER) 
        {
            i += 1; continue;
        }
        else
        {            
            sAddr = pbuf[i+2];                        //check slave addr
            
            if((sAddr>(Motor_Num_Arm+Motor_Num_Head+Motor_Num_Shoulder))||(sAddr==0)) //Ŀǰֻ��15�����
            {
                i += 1; continue;
            }            
            
            len = *(pbuf+i+3);                        //check lenght

            if(len<COMM_SERVO_RECVD_MSG_LENGTH_MINI)
            {
                i += 1; continue;
            }            
            
            rCrc0 = CRC16_Modbus((pbuf+i), (len-2));  //check crc
            rCrc1 = (pbuf[i+len-2])+((pbuf[i+len-1])<<8);
            
            if(rCrc0!=rCrc1)
            {
                i += 1; continue;
            }

            fCode = *(pbuf+i+4);                     //check function code 

            if(ERR_NONE!=l_GetGeneralMotorValidFunctionCode(fCode))
            {
                i += 1; continue;
            }

            //sta = *(pbuf+i+5);                         //����
            
            if(ERR_NONE!=sysGetGeneralMotorValidRecvdData(devType,sAddr,fCode,(pbuf+i+6),(len-COMM_SERVO_RECVD_MSG_LENGTH_MINI)))
            {
                i += 1; continue;
            }
            else
            {
                i += len; continue;
            }
        }

    }

    return ERR_NONE;
}


s8 CheckRecvDataLen(EwayMotor_Device_Type devType , u8 uLen, u8 *pUData)
{
    if((EWAYBOT_ARMS_LEFT==devType&&StatusLen_LeftArm==uLen)
        ||(EWAYBOT_ARMS_RIGHT==devType&&StatusLen_RightArm==uLen)
        ||(EWAYBOT_SHOULDER==devType&&StatusLen_Head_Shoulder==uLen)
        ||(EWAYBOT_GRIPPERS==devType&&StatusLen_Gripper==uLen)
        ||(EWAYBOT_WHEEL==devType&&StatusLen_Wheel==uLen)
        ||(EWAYBOT_PSM==devType&&StatusLen_PSM==uLen)
        ||(EWAYBOT_PSU==devType&&StatusLen_PSU==uLen))
        return ERR_NONE;
    else
    {
        SysLogWrite(LogNormal,"*****RX Data from COM is error.  DevType = %d Len= %d*****\n\r",devType,uLen);

        for(u8 i=0;i<uLen;i++)
            SysLogWrite(LogNormal,"  data[%d]=%d ",i,pUData[uLen]);
        
        return ERR_RX_DATA_LEN;
    }
}


/* --------------------------------------------------------------------------*/
/**
* @name sysGeneralMotorRecvDataProcess
* @brief 
* @details Process the received from COMx port
*
* @param[in] devType Eway Motor device type:arms,shoulder,head,wheels and so on.
*
* @returns ERR_NONE Successful
* others Failed
*
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysGeneralMotorRecvDataProcess(EwayMotor_Device_Type devType)
{
    COM_PORT_E uni;
    u8 dat[COM_RX_BUFFER_SINGLE_OP_SIZE]={0};
    u16 len=0;
    s8 res=ERR_NONE;  

    if(sysGetMotorCommPort(devType,&uni)!=ERR_NONE)             //get com ID
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(ComRecv[uni].RxCount>=8)                                 //header0 header1 slaveAddr length functionCode Status ... crc0 crc1
    {        
        res = g_GetMotorRecvdData(dat,&len,&ComRecv[uni]);        //copy data to temp buffer    
        if( ERR_NONE != res )
        {
            //res = Bsp_printf("2-2 g_GetMotorRecvdData(%d) failed.",devType);
        }
        
    }

/*    //!< ���ε��˺���������ڴ˴����ʺϣ������ں���sysGetGeneralMotorValidRecvdData()������յ���ÿ��״̬�������ݳ��ȼ���
    if(len>0)
    {        
        //!< res=CheckRecvDataLen(devType,len,dat);
        if( ERR_NONE != res )
        {
            //res = Bsp_printf("2-2 CheckRecvDataLen(%d,%d) failed.",devType,len);
        }        
    }
    */

    if((ERR_NONE==res)&&(len>0))
    {
        res = sysParseGeneralMotorRecvdData(dat,len,devType);     //��COM�ڶ�������ֵ����ȫ�ֱ����С�
        if( ERR_NONE != res )
        {
            //res = Bsp_printf("2-2 sysParseGeneralMotorRecvdData(%d,%d) failed.",devType,len);
        }
    }

    return res;
}

s8 sysGeneralMotorRecvRegsProcess(EwayMotor_Device_Type devType)
{
    COM_PORT_E uni;
    u8 dat[COM_RX_BUFFER_SINGLE_OP_SIZE]={0};
    u16 len=0;
    s8 res=0;  
    
    if(sysGetMotorCommPort(devType,&uni)!=ERR_NONE)             //get com ID
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(ComRecv[uni].RxCount>=8)                               //header0 header1 slaveAddr length functionCode Status ... crc0 crc1
    {       
       res = g_GetMotorRecvdData(dat,&len,&ComRecv[uni]);       //copy data to temp buffer       
    }
        
    if(ERR_NONE==res)    
    {
       res = sysParseGeneralMotorRecvdRegs(dat,len,devType);     //��COM�ڶ�������ֵ����ȫ�ֱ����С�
    }
    
    return res;
}

/* --------------------------------------------------------------------------*/
/**
* @name sysGeneralMotorBroadcastRead
* @brief 
* @details Pack the Motor Broadcast Read package and send info to corresponding Com Port
*
* @param[in] unSlaveStartID Broadcast read Start Motor ID:Arms(1-12),Head(14-15),Shoulder(13),Wheels(1-2) and so on 
* @param[in] unSlaveNums Numbers of Broadcast read Slave Motor 
* @param[in] unRegStartAddr Start Register Address for every Motor
* @param[in] unRegNums Numbers of Register to read for every Motor 
* @param[in] delayTime Reponse time for every Motor after receiving the Broadcast Read 
* @param[in] devType Device type of corresponding Motors 
*
* @returns ERR_NONE Successful
* others Failed
*
* @author 
*/
/* --------------------------------------------------------------------------*/
/*
s8 sysGeneralMotorBroadcastRead(u8 unSlaveStartID,u8 unSlaveNums,u8 unRegStartAddr,u8 unRegNums,u8 *delayTime,EwayMotor_Device_Type devType)    
{
    COM_PORT_E unPort;
    u8 unSendBuf[200]={0};
    u8 uni;
    s8 res;
    u16 unCRC;

    if(sysGetMotorCommPort(devType,&unPort)!=ERR_NONE)             //get com ID
    {
        return ERR_INPUT_PARAMETERS;
    }    
    
    //check the params
    if(unSlaveNums>SYS_MAX_GENERAL_SERVO_NUMS)  return ERR_INPUT_PARAMETERS;

    //pack message
    unSendBuf[0] = COMM_SERVO_MSG_HEADER&0xFF;
    unSendBuf[1] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[2] = BROADCAST_ID;
    unSendBuf[3] = 9+(unSlaveNums<<1);
    unSendBuf[4] = MY_SERVO_BROADCAST_READ;
    unSendBuf[5] = unRegStartAddr;
    unSendBuf[6] = unRegNums;

    for (uni=0;uni<unSlaveNums;uni++)
    {
        unSendBuf[uni*2+7]=uni+unSlaveStartID+1;
        unSendBuf[uni*2+8]=delayTime[uni];
    }

    unCRC=CRC16_Modbus(&unSendBuf[0],(7+(unSlaveNums<<1)));           //crc check
    unSendBuf[((unSlaveNums<<1)+7)] = (u8)unCRC;
    unSendBuf[((unSlaveNums<<1)+8)] = (u8)(unCRC>>8);
    
    //send:
    res = UartDmaTransferConfig(unPort,unSendBuf,(u16)unSendBuf[3]);

    return res;
}
*/
/*
s8 sysGeneralMotorBroadcastRead(u8 unSlaveStartID,u8 unSlaveNums,u8 unRegStartAddr,u8 unRegNums,u8 *delayTime,EwayMotor_Device_Type devType)    
{
    COM_PORT_E unPort;
    u8 unSendBuf[200]={0};
    u8 uni;
    s8 res;
    u16 unCRC;
    QueueHandle_t pQ;
    
    if(sysGetMotorCommPort(devType,&unPort)!=ERR_NONE)               //get com ID
    {
        return ERR_INPUT_PARAMETERS;
    }    

    if(sysGetMotorSendQueue(devType,&pQ))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    //check the params
    if(unSlaveNums>SYS_MAX_GENERAL_SERVO_NUMS)    return ERR_INPUT_PARAMETERS;
    
    //pack message
    unSendBuf[1] = COMM_SERVO_MSG_HEADER&0xFF;
    unSendBuf[2] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[3] = BROADCAST_ID;
    unSendBuf[4] = 9+(unSlaveNums<<1);
    unSendBuf[5] = MY_SERVO_BROADCAST_READ;
    unSendBuf[6] = unRegStartAddr;
    unSendBuf[7] = unRegNums;
    
    for (uni=0;uni<unSlaveNums;uni++)
    {
        unSendBuf[uni*2+8]=uni+unSlaveStartID;
        unSendBuf[uni*2+9]=delayTime[uni];
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[1],(7+(unSlaveNums<<1)));           //crc check
    unSendBuf[((unSlaveNums<<1)+8)] = (u8)unCRC;
    unSendBuf[((unSlaveNums<<1)+9)] = (u8)(unCRC>>8);
        
    //send:
    unSendBuf[0] = unSendBuf[4];
    xQueueSend(pQ,unSendBuf,0);
    
    return res;
}
*/
s8 sysGeneralMotorBroadcastRead(u8 unSlaveStartID,u8 unSlaveNums,u8 unRegStartAddr,u8 unRegNums,u8 *delayTime,EwayMotor_Device_Type devType)    
{
    COM_PORT_E unPort;
    u8 unSendBuf[200]={0};        //unSendBuf[0]��ʶ�Ƿ���Ҫ��ʱ���ȴ�������أ�0-������ʱ��1-��Ҫ��ʱ    unSendBuf[1]��Ŵ����Ͱ����ݳ���
    u8 uni;
    s8 res=ERR_NONE;
    u16 unCRC;
    QueueHandle_t pQ;
    
    if(sysGetMotorCommPort(devType,&unPort)!=ERR_NONE)               //get com ID
    {
        return ERR_INPUT_PARAMETERS;
    }    

    if(sysGetMotorSendQueue(devType,&pQ))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    //check the params
    if(unSlaveNums>SYS_MAX_GENERAL_SERVO_NUMS)    return ERR_INPUT_PARAMETERS;
    
    //pack message
    unSendBuf[2] = COMM_SERVO_MSG_HEADER&0xFF;
    unSendBuf[3] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[4] = BROADCAST_ID;
    unSendBuf[5] = 9+(unSlaveNums<<1);
    unSendBuf[6] = MY_SERVO_BROADCAST_READ;
    unSendBuf[7] = unRegStartAddr;
    unSendBuf[8] = unRegNums;
    
    for (uni=0;uni<unSlaveNums;uni++)
    {
        unSendBuf[uni*2+9]=uni+unSlaveStartID;
        unSendBuf[uni*2+10]=delayTime[uni];
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[2],(7+(unSlaveNums<<1)));           //crc check
    unSendBuf[((unSlaveNums<<1)+9)] = (u8)unCRC;
    unSendBuf[((unSlaveNums<<1)+10)] = (u8)(unCRC>>8);
        
    //send:
    unSendBuf[0] = 1;
    unSendBuf[1] = unSendBuf[5];
    if(pdTRUE != xQueueSend(pQ,unSendBuf,0))
    {
        res = ERR_PROCESS_FAILED;
    }
    
    return res;
}



/* --------------------------------------------------------------------------*/
/**
* @name sysGeneralMotorBroadcastWrite
* @brief 
* @details Pack the Motor Broadcast Write package and send info to corresponding Com Port
*
* @param[in] unSlaveStartID Broadcast read Start Motor ID:Arms,Head,Shoulder,Wheels and so on
* @param[in] unSlaveNums Numbers of Broadcast read Slave Motor 
* @param[in] unRegStartAddr Start Register Address for every Motor
* @param[in] unRegNums Numbers of Register to read for every Motor 
* @param[in] delayTime Reponse time for every Motor after receiving the Broadcast Read 
* @param[in] devType Device type of corresponding Motors 
*
* @returns ERR_NONE Successful
* others Failed
*
* @author 
*/
/* --------------------------------------------------------------------------*/
/*
s8 sysGeneralMotorBroadcastWrite(u8 unSlaveStartID,u8 unSlaveNums,u8 unRegStartAddr,u8 unRegNums,u8 *pdat,EwayMotor_Device_Type devType)    
{
    u8 unSendBuf[256]={0};
    u8 uni;
    s8 res;
    u16 unCRC,len;
    COM_PORT_E unPort;
        
    if(sysGetMotorCommPort(devType,&unPort)!=ERR_NONE)             //!< get com ID
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    if(unSlaveNums>SYS_MAX_GENERAL_SERVO_NUMS)  return ERR_INPUT_PARAMETERS; //!< check the params
    
    len = 9+((unRegNums+1)*unSlaveNums);
    if(len>=256) return ERR_INPUT_PARAMETERS;                  

    //pack message
    unSendBuf[0] = COMM_SERVO_MSG_HEADER&0xFF;
    unSendBuf[1] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[2] = BROADCAST_ID;
    unSendBuf[3] = len;
    unSendBuf[4] = MY_SERVO_BROADCAST_WRITE;
    unSendBuf[5] = unRegStartAddr;
    unSendBuf[6] = unRegNums;

    for (uni=0;uni<unSlaveNums;uni++)
    {
        unSendBuf[(uni*(unRegNums+1))+7] = (unSlaveStartID+uni);   //!< Slave ID
        
        memcpy(&(unSendBuf[(uni*(unRegNums+1))+8]),(pdat+(uni*unRegNums)),unRegNums);
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[0],(7+((unRegNums+1)*unSlaveNums)));              //!< compute Crc
    unSendBuf[7+((unRegNums+1)*unSlaveNums)] = (u8)unCRC;
    unSendBuf[8+((unRegNums+1)*unSlaveNums)] = (u8)(unCRC>>8);

    //send:
    res = UartDmaTransferConfig(unPort,unSendBuf,(u16)unSendBuf[3]);    
    
    return res;
}

*/

//!< pdat��˳���ŵ���(idx + datN) 
/*
     id0 reg0 reg1 reg2 reg3...
     id1 reg0 reg1 reg2 reg3...
     id2 reg0 reg1 reg2 reg3...
     ...
*/
//!< ��Ե���slave ID�ǲ�������ָ��
s8 sysGeneralMotorDiscontinuousBroadcastWrite(u8* pSlaveID,u8 unSlaveNums,u8 unRegStartAddr,u8 unRegNums,u8 *pdat,EwayMotor_Device_Type devType)    
{
    u8 unSendBuf[256]={0};
    u8 uni;
    s8 res=ERR_NONE;
    u16 unCRC,len;
    QueueHandle_t pQ;
    MotorRecordBuff* pRecordBuff;
    u8 maxId,startId;
    
    if(sysGetMotorSendQueue(devType,&pQ))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    if(unSlaveNums>SYS_MAX_GENERAL_SERVO_NUMS)  return ERR_INPUT_PARAMETERS; //!< check the params
    
    len = 9+((unRegNums+1)*unSlaveNums);
    if(len>=256) return ERR_INPUT_PARAMETERS;                  

    //pack message
    unSendBuf[2] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[3] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[4] = BROADCAST_ID;
    unSendBuf[5] = len;
    unSendBuf[6] = MY_SERVO_BROADCAST_WRITE;
    unSendBuf[7] = unRegStartAddr;
    unSendBuf[8] = unRegNums;

    for (uni=0;uni<unSlaveNums;uni++)
    {
        unSendBuf[(uni*(unRegNums+1))+9] = *(pSlaveID+uni);   //!< Slave ID
        
        memcpy(&(unSendBuf[(uni*(unRegNums+1))+10]),(pdat+(uni*unRegNums)),unRegNums);
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[2],(7+((unRegNums+1)*unSlaveNums)));              //!< compute Crc
    unSendBuf[9+((unRegNums+1)*unSlaveNums)] = (u8)unCRC;
    unSendBuf[10+((unRegNums+1)*unSlaveNums)] = (u8)(unCRC>>8);

    //send:
    unSendBuf[0] = 0;
    unSendBuf[1] = unSendBuf[5];
    if(pdTRUE != xQueueSend(pQ,unSendBuf,0))
    {
        res = ERR_PROCESS_FAILED;
    }
#if EMB_Upload_SingleCycle_Motor_Info_DEBUG
    else        //!< ��¼�·���ָ��
    {
        if((unRegStartAddr==EwayMotorReg_TargSpd_L)&&(unRegNums==4))
        {
            if(sysGetMotorSendRecordPointer(devType,&pRecordBuff,&maxId,&startId)==ERR_NONE)
            {                   
                memset(unSendBuf,0,Write_Regs_Record_Max_Space);
                
                unSendBuf[0] = (u8)SysTimeStamp;
                unSendBuf[1] = (u8)(SysTimeStamp>>8);
                unSendBuf[2] = (u8)(SysTimeStamp>>16);            
                unSendBuf[3] = (u8)(SysTimeStamp>>24);

                for(uni=0;uni<unSlaveNums;uni++)
                {
                    if((*(pSlaveID+uni)) > maxId)
                    {
                        continue;
                    }
                    unSendBuf[4] = *(pSlaveID+uni);
                    unSendBuf[5] = 0x00;
                    
                    unSendBuf[6] = *(pdat+(uni*unRegNums)+2);    //!< DnPos
                    unSendBuf[7] = *(pdat+(uni*unRegNums)+3);
                    unSendBuf[8] = *(pdat+(uni*unRegNums));   //!< DnSpd
                    unSendBuf[9] = *(pdat+(uni*unRegNums)+1);
                    
                    memcpy(&((pRecordBuff+(*(pSlaveID+uni)-1))->wRegs[0]),unSendBuf,10);

                    memset(&unSendBuf[4],0,(Write_Regs_Record_Max_Space-4));
                }

            }
        }
    }
#endif 

    return res;
}
/*
s8 sysGeneralMotorDiscontinuousBroadcastWrite(u8* pSlaveID,u8 unSlaveNums,u8 unRegStartAddr,u8 unRegNums,u8 *pdat,EwayMotor_Device_Type devType)    
{
    u8 unSendBuf[256]={0};
    u8 uni;
    s8 res;
    u16 unCRC,len;
    QueueHandle_t pQ;
    
    if(sysGetMotorSendQueue(devType,&pQ))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    if(unSlaveNums>SYS_MAX_GENERAL_SERVO_NUMS)  return ERR_INPUT_PARAMETERS; //!< check the params
    
    len = 9+((unRegNums+1)*unSlaveNums);
    if(len>=256) return ERR_INPUT_PARAMETERS;                  

    //pack message
    unSendBuf[1] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[2] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[3] = BROADCAST_ID;
    unSendBuf[4] = len;
    unSendBuf[5] = MY_SERVO_BROADCAST_WRITE;
    unSendBuf[6] = unRegStartAddr;
    unSendBuf[7] = unRegNums;

    for (uni=0;uni<unSlaveNums;uni++)
    {
        unSendBuf[(uni*(unRegNums+1))+8] = *(pSlaveID+uni);   //!< Slave ID
        
        memcpy(&(unSendBuf[(uni*(unRegNums+1))+9]),(pdat+(uni*unRegNums)),unRegNums);
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[1],(7+((unRegNums+1)*unSlaveNums)));              //!< compute Crc
    unSendBuf[8+((unRegNums+1)*unSlaveNums)] = (u8)unCRC;
    unSendBuf[9+((unRegNums+1)*unSlaveNums)] = (u8)(unCRC>>8);

    unSendBuf[0] = unSendBuf[4];
    xQueueSend(pQ,unSendBuf,0);
    
    return res;
}
*/
/*
s8 sysGeneralMotorDiscontinuousBroadcastWrite(u8* pSlaveID,u8 unSlaveNums,u8 unRegStartAddr,u8 unRegNums,u8 *pdat,EwayMotor_Device_Type devType)    
{
    u8 unSendBuf[256]={0};
    u8 uni;
    s8 res;
    u16 unCRC,len;
    //COM_PORT_E unPort;
    QueueHandle_t pQ;
    
    //if(sysGetMotorCommPort(devType,&unPort)!=ERR_NONE)             //!< get com ID
    //{
        //return ERR_INPUT_PARAMETERS;
    //}

    if(sysGetMotorSendQueue(devType,&pQ))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    if(unSlaveNums>SYS_MAX_GENERAL_SERVO_NUMS)  return ERR_INPUT_PARAMETERS; //!< check the params
    
    len = 9+((unRegNums+1)*unSlaveNums);
    if(len>=256) return ERR_INPUT_PARAMETERS;                  

    //pack message
    unSendBuf[1] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[2] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[3] = BROADCAST_ID;
    unSendBuf[4] = len;
    unSendBuf[5] = MY_SERVO_BROADCAST_WRITE;
    unSendBuf[6] = unRegStartAddr;
    unSendBuf[7] = unRegNums;

    for (uni=0;uni<unSlaveNums;uni++)
    {
        unSendBuf[(uni*(unRegNums+1))+8] = *(pSlaveID+uni);   //!< Slave ID
        
        memcpy(&(unSendBuf[(uni*(unRegNums+1))+9]),(pdat+(uni*unRegNums)),unRegNums);
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[1],(7+((unRegNums+1)*unSlaveNums)));              //!< compute Crc
    unSendBuf[8+((unRegNums+1)*unSlaveNums)] = (u8)unCRC;
    unSendBuf[9+((unRegNums+1)*unSlaveNums)] = (u8)(unCRC>>8);

    unSendBuf[0] = unSendBuf[4];
    xQueueSend(pQ,unSendBuf,0);
    
    return res;
}
*/



s8 sysGeneralMotorBroadcastWrite(u8 unSlaveStartID,u8 unSlaveNums,u8 unRegStartAddr,u8 unRegNums,u8 *pdat,EwayMotor_Device_Type devType)    
{
    u8 unSendBuf[256]={0};        //unSendBuf[0]��ʶ�Ƿ���Ҫ��ʱ���ȴ�������أ�0-������ʱ��1-��Ҫ��ʱ    unSendBuf[1]��Ŵ����Ͱ����ݳ���
    u8 uni;
    u16 unCRC,len;
    COM_PORT_E unPort;
    QueueHandle_t pQ;
    
    if(sysGetMotorCommPort(devType,&unPort)!=ERR_NONE)             //!< get com ID
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(sysGetMotorSendQueue(devType,&pQ))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    if(unSlaveNums>SYS_MAX_GENERAL_SERVO_NUMS)  return ERR_INPUT_PARAMETERS; //!< check the params
    
    len = 9+((unRegNums+1)*unSlaveNums);
    if(len>=256) return ERR_INPUT_PARAMETERS;                  

    //pack message
    unSendBuf[2] = COMM_SERVO_MSG_HEADER&0xFF;
    unSendBuf[3] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[4] = BROADCAST_ID;
    unSendBuf[5] = len;
    unSendBuf[6] = MY_SERVO_BROADCAST_WRITE;
    unSendBuf[7] = unRegStartAddr;
    unSendBuf[8] = unRegNums;

    for (uni=0;uni<unSlaveNums;uni++)
    {
        unSendBuf[(uni*(unRegNums+1))+9] = (unSlaveStartID+uni);   //!< Slave ID
        
        memcpy(&(unSendBuf[(uni*(unRegNums+1))+10]),(pdat+(uni*unRegNums)),unRegNums);
    }
    
    unCRC=CRC16_Modbus(&unSendBuf[2],(7+((unRegNums+1)*unSlaveNums)));              //!< compute Crc
    unSendBuf[9+((unRegNums+1)*unSlaveNums)] = (u8)unCRC;
    unSendBuf[10+((unRegNums+1)*unSlaveNums)] = (u8)(unCRC>>8);

    //send:
    unSendBuf[0] = 0;
    unSendBuf[1] = unSendBuf[5];
    if(pdTRUE != xQueueSend(pQ,unSendBuf,0))
    {
        return ERR_PROCESS_FAILED;
    }
    
    return ERR_NONE;
}


s8 g_GeneralMotorPing(u8 unSlaveAddr,COM_PORT_E unPort)    
{
    u8 unSendBuf[MY_SERVO_PingLength_H];
    u16 unCRC;
    s8 res;
        
    //check SlaveAddr
    if(unSlaveAddr>0xFD)    // 0 < SlaveAddr < 0xFd (253)
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    unSendBuf[0] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[1] = (COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[2] = unSlaveAddr;
    unSendBuf[3] = MY_SERVO_PingLength_H;    
    unSendBuf[4] = MY_SERVO_Ping_H;

    unCRC = CRC16_Modbus(&unSendBuf[0],5);
    
    unSendBuf[5] = (u8)unCRC;
    unSendBuf[6] = (unCRC>>8)&0xFF;
    
    //send:
    res = UartDmaTransferConfig(unPort,unSendBuf,(u16)unSendBuf[3]);

    return res;
}
/* --------------------------------------------------------------------------*/
/**
* @name g_GeneralMotorSingleRead
* @brief 
* @details 
*
* @param[in] unSlaveAddr 
*             regSegNums Ҫ���ļĴ����εĸ���
*             pInfo      �Ĵ����׵�ַ0������0���Ĵ����׵�ַ1������1
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 g_GeneralMotorSingleRead(u8 unSlaveAddr,u8 regSegNums,u8* pInfo,EwayMotor_Device_Type devType)
{
    u8 unSendBuf[100];
    s8 res=ERR_NONE;
    u16 unCRC;
    QueueHandle_t pQ;
    
    if(unSlaveAddr>0xFD)                     //check the params   0 < SlaveAddr < 0xFd (253)
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    if(sysGetMotorSendQueue(devType,&pQ))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    unSendBuf[2] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[3] = (COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[4] = unSlaveAddr;
    unSendBuf[5] = (7+(regSegNums<<1));                       // len = 7+2*N
    unSendBuf[6] = MY_SERVO_Read_H;
    
    memcpy(&unSendBuf[7],pInfo,(regSegNums<<1));              //��дҪ���ļĴ���������
        
    unCRC = CRC16_Modbus(&unSendBuf[2],(5+(regSegNums<<1)));
        
    unSendBuf[(7+(regSegNums<<1))] = (u8)unCRC;
    unSendBuf[(8+(regSegNums<<1))] = (u8)(unCRC>>8);
    
    //send:
    unSendBuf[0] = 1;
    unSendBuf[1] = unSendBuf[5];
    
    if(pdTRUE != xQueueSend(pQ,unSendBuf,0))
    {
        res = ERR_PROCESS_FAILED;
    }
    
    return res;
}

/*
s8 g_GeneralMotorSingleRead(u8 unSlaveAddr,u8 regSegNums,u8* pInfo,COM_PORT_E unPort)
{
    u8 unSendBuf[100];
    s8 res;
    u16 unCRC;
    
    if(unSlaveAddr>0xFD)                     //check the params   0 < SlaveAddr < 0xFd (253)
    {
        return ERR_INPUT_PARAMETERS;
    }

    unSendBuf[0] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[1] = (COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[2] = unSlaveAddr;
    unSendBuf[3] = (7+(regSegNums<<1));                       // len = 7+2*N
    unSendBuf[4] = MY_SERVO_Read_H;

    memcpy(&unSendBuf[5],pInfo,(regSegNums<<1));              //��дҪ���ļĴ���������
    
    unCRC = CRC16_Modbus(&unSendBuf[0],(5+(regSegNums<<1)));
    
    unSendBuf[(5+(regSegNums<<1))] = (u8)unCRC;
    unSendBuf[(6+(regSegNums<<1))] = (u8)(unCRC>>8);

    //Send:
    res = UartDmaTransferConfig(unPort,unSendBuf,(u16)unSendBuf[3]);

    return res;
}*/
/*
s8 g_GeneralMotorSingleRead(u8 unSlaveAddr,u8 regSegNums,u8* pInfo,EwayMotor_Device_Type devType)
{
    u8 unSendBuf[100];
    s8 res=ERR_NONE;
    u16 unCRC;
    QueueHandle_t pQ;
    
    if(unSlaveAddr>0xFD)                     //check the params   0 < SlaveAddr < 0xFd (253)
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    if(sysGetMotorSendQueue(devType,&pQ))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    unSendBuf[1] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[2] = (COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[3] = unSlaveAddr;
    unSendBuf[4] = (7+(regSegNums<<1));                       // len = 7+2*N
    unSendBuf[5] = MY_SERVO_Read_H;
    
    memcpy(&unSendBuf[6],pInfo,(regSegNums<<1));              //��дҪ���ļĴ���������
        
    unCRC = CRC16_Modbus(&unSendBuf[1],(5+(regSegNums<<1)));
        
    unSendBuf[(6+(regSegNums<<1))] = (u8)unCRC;
    unSendBuf[(7+(regSegNums<<1))] = (u8)(unCRC>>8);
    
    //send:
    unSendBuf[0] = unSendBuf[4];
    
    if(pdTRUE != xQueueSend(pQ,unSendBuf,0))
    {
        res = ERR_PROCESS_FAILED;
    }
    
    return res;
}
*/
/*
s8 g_GeneralMotorSingleWrite(u8 unSlaveAddr,u8 infoLen,u8* pInfo,COM_PORT_E unPort)
{
    u8 unSendBuf[200];
    s8 res;
    u16 unCRC;
    
    if(unSlaveAddr>0xFD)                     //check the params   0 < SlaveAddr < 0xFd (253)
    {
        return ERR_INPUT_PARAMETERS;
    }

    unSendBuf[0] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[1] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[2] = unSlaveAddr;
    unSendBuf[3] = (7+infoLen);
    unSendBuf[4] = MY_SERVO_Write_H;

    memcpy(&unSendBuf[5],pInfo,infoLen);
    
    unCRC = CRC16_Modbus(&unSendBuf[0],(5+infoLen));
    
    unSendBuf[(5+infoLen)] = (u8)unCRC;
    unSendBuf[(6+infoLen)] = (u8)(unCRC>>8);

    //Send:
    res = UartDmaTransferConfig(unPort,unSendBuf,(u16)unSendBuf[3]);

    return res;
}
*/
s8 g_GeneralMotorSingleWrite(u8 unSlaveAddr,u8 infoLen,u8* pInfo,EwayMotor_Device_Type devType)
{
    COM_PORT_E unPort;
    u8 unSendBuf[20]={0};        //unSendBuf[0]��ʶ�Ƿ���Ҫ��ʱ���ȴ�������أ�0-������ʱ��1-��Ҫ��ʱ    unSendBuf[1]��Ŵ����Ͱ����ݳ���
    u16 unCRC;
    QueueHandle_t pQ;    
    
    if(unSlaveAddr>0xFD)                     //check the params   0 < SlaveAddr < 0xFd (253)
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(sysGetMotorCommPort(devType,&unPort)!=ERR_NONE)             //!< get com ID
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(sysGetMotorSendQueue(devType,&pQ))
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    unSendBuf[2] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[3] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[4] = unSlaveAddr;
    unSendBuf[5] = (7+infoLen);
    unSendBuf[6] = MY_SERVO_Write_H;

    memcpy(&unSendBuf[7],pInfo,infoLen);
    
    unCRC = CRC16_Modbus(&unSendBuf[2],(5+infoLen));
    
    unSendBuf[(7+infoLen)] = (u8)unCRC;
    unSendBuf[(8+infoLen)] = (u8)(unCRC>>8);

    //send:
    unSendBuf[0] = 1;
    unSendBuf[1] = unSendBuf[5];
    if(pdTRUE != xQueueSend(pQ,unSendBuf,0))
    {
        return ERR_PROCESS_FAILED;
    }
    
    return ERR_NONE;

}

    
s8 g_GeneralMotorSetZeroPosition(u8 unSlaveAddr,COM_PORT_E unPort)
{
    u8 unSendBuf[10];
    s8 res;
    u16 unCRC;

    if(unSlaveAddr>0xFD)                     //check the params   0 < SlaveAddr < 0xFd (253)
    {
        return ERR_INPUT_PARAMETERS;
    }

    unSendBuf[0] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[1] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[2] = unSlaveAddr;
    unSendBuf[3] = 0x07;
    unSendBuf[4] = MY_SERVO_SetZero_H;
    
    unCRC=CRC16_Modbus(&unSendBuf[0],7);
    
    unSendBuf[5] = (u8)unCRC;
    unSendBuf[6] = (u8)(unCRC>>8);
    
    //Send:
    res = UartDmaTransferConfig(unPort,unSendBuf,(u16)unSendBuf[3]);

    return res;
}

s8 g_GeneralMotorSetSoftRestart(u8 unSlaveAddr,COM_PORT_E unPort)
{
    u8 unSendBuf[10];
    u8 res;
    u16 unCRC;
    
    if(unSlaveAddr>0xFD)                     //check the params   0 < SlaveAddr < 0xFd (253)
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    unSendBuf[0] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[1] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[2] = unSlaveAddr;
    unSendBuf[3] = 0x07;
    unSendBuf[4] = MY_SERVO_Reborn_H;
        
    unCRC=CRC16_Modbus(&unSendBuf[0],7);
    
    unSendBuf[5] = (u8)unCRC;
    unSendBuf[6] = (u8)(unCRC>>8);
        
    //Send:
    res = UartDmaTransferConfig(unPort,unSendBuf,(u16)unSendBuf[3]);
    
    return res;
}

s8 g_GeneralMotorReset(u8 unSlaveAddr,COM_PORT_E unPort)
{
    u8 unSendBuf[10];
    s8 res;
    u16 unCRC;
    
    if(unSlaveAddr>0xFD)                     //check the params   0 < SlaveAddr < 0xFd (253)
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    unSendBuf[0] = (u8)COMM_SERVO_MSG_HEADER;
    unSendBuf[1] = (u8)(COMM_SERVO_MSG_HEADER>>8);
    unSendBuf[2] = unSlaveAddr;
    unSendBuf[3] = 0x07;
    unSendBuf[4] = MY_SERVO_SetInit_H;
        
    unCRC=CRC16_Modbus(&unSendBuf[0],7);
    
    unSendBuf[5] = (u8)unCRC;
    unSendBuf[6] = (u8)(unCRC>>8);
        
    //Send:
    res = UartDmaTransferConfig(unPort,unSendBuf,(u16)unSendBuf[3]);
    
    return res;
}



/*
���¹ؽڵ���ĵ�����״̬�ϱ����������еĴӵ������ȡ��������
*/
s8 l_UpdatJointMotorSingleCyStatusCollectionDat(NodeStateModule* pModuleSta,u16 mCommSta,u8 mNums)
{
    u8 i;
    u32 tSmp;
    u8* pdat0;
    u8* pdat1;
    s16 spd;

    if(pModuleSta==NULL)
    {
        return  ERR_INPUT_PARAMETERS;
    }

    if(mNums > SYS_MAX_GENERAL_SERVO_NUMS)
    {
        return  ERR_INPUT_PARAMETERS;
    }
    
    for(i=0;i<mNums;i++)
    {
        if((mCommSta&(0x0001<<i))==0)   //!< ��һͨ������ͨ�Ų��ɹ�
        {
            continue;
        }
    
        pdat0 = &((pModuleSta->pMotorRecord+i)->wRegs[0]);
    
        tSmp = (*pdat0)+((*(pdat0+1))<<8)+((*(pdat0+2))<<16)+((*(pdat0+3))<<24);
    
        if(tSmp!=0)//!< ��һͨ�������Ƿ��·������ָ��
        {
            pdat1 = &((pModuleSta->pFromMotor+i)->mRegs[0]);
            spd = (*(pdat1 + 11) + ((*(pdat1 + 12))<<8))/MOTOR_SPEED_COEFFICIENT;
    
            *(pdat0 + 10) = *(pdat1 + 13);           //!< timeStamp(4)+id(2)+DnPos(2)+DnSpd(2)+....
            *(pdat0 + 11) = *(pdat1 + 14);
            *(pdat0 + 12) = (u8)spd;
            *(pdat0 + 13) = (u8)(spd>>8);
        }
    }

    return ERR_NONE;
}


/*
���³���ģʽ�¹����ĵ��,�䵥����״̬�ϱ����������еĴӵ������ȡ��������
*/
s8 l_UpdatWheelMotorSingleCyStatusCollectionDat(EwayMotor_Device_Type devType,NodeStateModule* pModuleSta,u16 mCommSta,u8 mNums)
{    
    u8 i;
    u32 tSmp;
    u8* pdat0;
    u8* pdat1;

    if(pModuleSta==NULL)
    {
        return  ERR_INPUT_PARAMETERS;
    }

    if(mNums > SYS_MAX_GENERAL_SERVO_NUMS)
    {
        return  ERR_INPUT_PARAMETERS;
    }

    if((devType!=EWAYBOT_SHOULDER)&&(devType!=EWAYBOT_WHEEL))
    {
        return  ERR_INPUT_PARAMETERS;
    }
    
    for(i=0;i<mNums;i++)
    {
        if((mCommSta&(0x0001<<i))==0)   //!< ��һͨ������ͨ�Ų��ɹ�
        {
            continue;
        }
    
        pdat0 = &((pModuleSta->pMotorRecord+i)->wRegs[0]);
    
        tSmp = (*pdat0)+((*(pdat0+1))<<8)+((*(pdat0+2))<<16)+((*(pdat0+3))<<24);
    
        if(tSmp!=0)//!< ��һͨ�������Ƿ��·������ָ��
        {
            pdat1 = &((pModuleSta->pFromMotor+i)->mRegs[0]);
    
            *(pdat0 + 13) = *(pdat1 + 15);           //!< timeStamp(4)+id(2)+instructionType(1)+DnPos(2)+DnSpd(2)+....
            *(pdat0 + 14) = *(pdat1 + 16);
            *(pdat0 + 15) = *(pdat1 + 17);
            *(pdat0 + 16) = *(pdat1 + 18);

            switch(devType)
            {
                case EWAYBOT_SHOULDER:
                    *(pdat0 + 17) = *(pdat1 + 9);
                    *(pdat0 + 18) = *(pdat1 + 10);
                    break;

                case EWAYBOT_WHEEL:
                    *(pdat0 + 17) = *(pdat1 + 11);
                    *(pdat0 + 18) = *(pdat1 + 12);
                    break;

                default:
                    break;
            }        
        }
    }

    return ERR_NONE;

}

/* --------------------------------------------------------------------------*/
/**
* @name sysEmbMotorStatusCollect
* @brief �ռ����������״̬�������������ϱ���PC�����ݰ�(fCode=0x0040)��
* @details 
*
* @param[in] None
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void sysEmbMotorStatusCollect(void)
{
    s8 res;

    //!< �����ֱ۵����
    res = l_UpdatJointMotorSingleCyStatusCollectionDat(&EwayArms.mState,EwayArms.mCommStatus,Motor_Num_Arm);
    if(res!=ERR_NONE)
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            Bsp_printf("sysEmbMotorStatusCollect() Arms failed.");
        }
    }
    
    //!< ����ͷ�������
    res = l_UpdatJointMotorSingleCyStatusCollectionDat(&EwayHeads.mState,EwayHeads.mCommStatus,Motor_Num_Head);
    if(res!=ERR_NONE)
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            Bsp_printf("sysEmbMotorStatusCollect() Heads failed.");
        }
    }

    //!< ���¼������
    res = l_UpdatWheelMotorSingleCyStatusCollectionDat(EWAYBOT_SHOULDER,&EwayShoulder.mState,EwayShoulder.mCommStatus,Motor_Num_Shoulder);
    if(res!=ERR_NONE)
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            Bsp_printf("sysEmbMotorStatusCollect() Shoulder failed.");
        }
    }

    //!< �������ӵ����
    res = l_UpdatWheelMotorSingleCyStatusCollectionDat(EWAYBOT_WHEEL,&EwayWheels.mState,EwayWheels.mCommStatus,Motor_Num_Wheel);
    if(res!=ERR_NONE)
    {
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
            Bsp_printf("sysEmbMotorStatusCollect() Wheels failed.");
        }
    }

}



void sysEmbMotorSingleCycleCmdStatusClear(void)
{
    u8 i;    
    u8* pdat;

    //!< ÿ��������ϱ���PC�ĵ������·��������ָ�����ϱ���״̬
    for(i=0;i<Motor_Num_Arm;i++)
    {
        pdat = &((EwayArms.mState.pMotorRecord+i)->wRegs[0]);
        
        memset(pdat,0,Write_Regs_Record_Max_Space);
    }

    for(i=0;i<Motor_Num_Head;i++)
    {
        pdat = &((EwayHeads.mState.pMotorRecord+i)->wRegs[0]);
        
        memset(pdat,0,Write_Regs_Record_Max_Space);
    } 

    for(i=0;i<Motor_Num_Shoulder;i++)
    {
        pdat = &((EwayShoulder.mState.pMotorRecord+i)->wRegs[0]);
        
        memset(pdat,0,Write_Regs_Record_Max_Space);
    } 

    for(i=0;i<Motor_Num_Head;i++)
    {
        pdat = &((EwayWheels.mState.pMotorRecord+i)->wRegs[0]);
        
        memset(pdat,0,Write_Regs_Record_Max_Space);
    } 
    
}


s8 sysEmbCheckSendToJointMotorParameters(u16* pPos,s16* pSpd)
{
    if(((*pPos)>MOTOR_POSITION_MAX)||((*pPos)<MOTOR_POSITION_MIN))   //!< λ�ó���
    {
        if(MOTOR_POSITION_MAX_LIMIT_EN)
        {
            if((*pPos)>MOTOR_POSITION_MAX)
            {
                (*pPos) = MOTOR_POSITION_MAX;
            }
        }
        
        if(MOTOR_POSITION_MIN_LIMIT_EN)
        {
            if((*pPos) < MOTOR_POSITION_MIN)
            {
                (*pSpd) = MOTOR_POSITION_MIN;
            }
        }        
     }
    
#if (MOTOR_SPEED_MIN_LIMIT_EN)					
    if(((*pSpd)<(MOTOR_SPEED_MIN*100))&&((*pSpd)>((-1)*(MOTOR_SPEED_MIN*100))))   //!< �ٶ�С����Сֵ
    {
        if((*pSpd)>=0)
        {
            (*pSpd) = MOTOR_SPEED_MIN*100;
        }
                                
        if((*pSpd) < 0)
        {
            (*pSpd) = (-1)*(MOTOR_SPEED_MIN*100);
        }
     }
#endif

#if (MOTOR_SPEED_MAX_LIMIT_EN)
     if(((*pSpd)<((-1)*(MOTOR_SPEED_MAX*100)))||((*pSpd)>(MOTOR_SPEED_MAX*100)))  //!< �ٶȴ������ֵ
     {
        if((*pSpd)>=0)
        {
            (*pSpd) = MOTOR_SPEED_MAX*100;
        }
                                
        if((*pSpd) < 0)
        {
            (*pSpd) = (-1)*(MOTOR_SPEED_MAX*100);
        }
    }
#endif

    return ERR_NONE;

}


/*
����ģʽ�ĵ��ֻ����ٶ��Ƿ���
*/
s8 sysEmbCheckSendToWheelMotorParameters(s16* pSpd)
{
#if (MOTOR_SPEED_MIN_LIMIT_EN)					
    if(((*pSpd)<(MOTOR_SPEED_MIN*100))&&((*pSpd)>((-1)*(MOTOR_SPEED_MIN*100))))   //!< �ٶ�С����Сֵ
    {
        if((*pSpd)>=0)
        {
            (*pSpd) = MOTOR_SPEED_MIN*100;
        }
                                    
        if((*pSpd) < 0)
        {
            (*pSpd) = (-1)*(MOTOR_SPEED_MIN*100);
        }
    }
#endif


#if (MOTOR_SPEED_MAX_LIMIT_EN)
    if(((*pSpd)<((-1)*(MOTOR_SPEED_MAX*100)))||((*pSpd)>(MOTOR_SPEED_MAX*100)))  //!< �ٶȴ������ֵ
    {
        if((*pSpd)>=0)
        {
            (*pSpd) = MOTOR_SPEED_MAX*100;
        }
                                    
        if((*pSpd) < 0)
        {
            (*pSpd) = (-1)*(MOTOR_SPEED_MAX*100);
        }
    }
#endif


    return ERR_NONE;

}

s8 sysEmbCheckSendToShoulderMotorParameters(s32* pSpd)
{
#if (MOTOR_SPEED_MIN_LIMIT_EN)					
    if(((*pSpd)<(MOTOR_SPEED_MIN*100))&&((*pSpd)>((-1)*(MOTOR_SPEED_MIN*100))))   //!< �ٶ�С����Сֵ
    {
        if((*pSpd)>=0)
        {
            (*pSpd) = MOTOR_SPEED_MIN*100;
        }
                                    
        if((*pSpd) < 0)
        {
            (*pSpd) = (-1)*(MOTOR_SPEED_MIN*100);
        }
    }
#endif


#if (MOTOR_SPEED_MAX_LIMIT_EN)
    if(((*pSpd)<((-1)*(SHOULDER_MOTOR_SPEED_MAX*100)))||((*pSpd)>(SHOULDER_MOTOR_SPEED_MAX*100)))  //!< �ٶȴ������ֵ
    {
        if((*pSpd)>=0)
        {
            (*pSpd) = SHOULDER_MOTOR_SPEED_MAX*100;
        }
                                    
        if((*pSpd) < 0)
        {
            (*pSpd) = (-1)*(SHOULDER_MOTOR_SPEED_MAX*100);
        }
    }
#endif


    return ERR_NONE;

}



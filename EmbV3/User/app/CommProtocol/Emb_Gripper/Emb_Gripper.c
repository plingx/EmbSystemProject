#include "includes.h"
#include "Emb_Gripper.h"

u8 g_GetMotorRecvdData(u8* pdat,u16* plen,COMRECVSTRUCT * pComReceive);
extern COMRECVSTRUCT ComRecv[];
extern EwayEmbSysModule EwayEmbSys;
extern EwayGripperModule EwayGrippers;
extern EwayEmbSysDebugModule* pDebug;




DYNAMIXEL_SERVO_STATUS sysDyServo[SYS_MAX_DYNAMIXEL_SERVO_NUMS];




/*
Protocol 1.0
Instruction Packet-Command data Main Controller sends to Dynamixel.

0xFF 0xFF ID Length Instruction Param1 Param_N CheckSum

0xFF 0xFF    :beginning of the packet

ID           :ID of Dynamixel,0~253(0x00-0xFD)
              Broadcasting ID = 254(0xFE)

LENGTH       :length of packet.

INSTRUCTION  :
               PING:No execution.used when controller is ready to receive Status Packet,no param
               READ_DATA:
               WRITE_DATA,REG_WRITE,



*/

/* --------------------------------------------------------------------------*/
/**
* @name sysSendDynamixelReadDataPacket
* @brief 
* @details send informations to read register data of Dynamixel servo
*
* @param[in] devType,Eway Motor device type:grippers
* @param[in] devID,ID of Dynamixel which will receive instruction Packet.
* @param[in] regStartAdd,Start address of register will be read.
* @param[in] regNum,Number of register will be read.
*
* @returns ERR_NONE Successful
* others Failed
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysSendDynamixelReadDataPacket(EwayMotor_Device_Type devType,u8 devID,u8 regStartAdd,u8 regNum)
{
    COM_PORT_E unPort;
    u8 dat[10]={0};
    u16 sum=0;
    u8 i;
    QueueHandle_t pQ;
    
    if(sysGetMotorCommPort(devType,&unPort)!=ERR_NONE)             //!< get com ID
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(sysGetMotorSendQueue(devType,&pQ))
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(devID >= 0xFF) return ERR_INPUT_PARAMETERS;                 //!< check input ID
        
    dat[2] = GRIPPER_FRAME_HEADER0;
    dat[3] = GRIPPER_FRAME_HEADER1;
    dat[4] = devID;
    dat[5] = 0x04;                     //!< datalength instruction + regstartaddr 
    dat[6] = DYNAMIXEL_CMD_RD_DATA;    //!< instruction:READ_DATA
    dat[7] = regStartAdd;              //!< register start address
    dat[8] = regNum;                   //!< number of register
    
    for(i=4;i<9;i++)
    {
        sum += dat[i];
    }

    dat[9] = (u8)(~sum);               //!<check sum

    //Send:FF FF 01 04 02 24 08 CC
    //res = UartDmaTransferConfig(unPort,dat,8);

    //send:
    dat[0] = 1;
    dat[1] = 10;
    if(pdTRUE != xQueueSend(pQ,dat,0))
    {
        return ERR_PROCESS_FAILED;
    }

    return ERR_NONE;
}


/* --------------------------------------------------------------------------*/
/**
* @name sysSendDynamixelBroadcastSyncWritePacket
* @brief 
* @details send Broadcast SYNC_WRITE cmd to Dynamixel servo
*
* @param[in] devType,Eway Motor device type:grippers.
* @param[in] regStartAdd,Start address of register will be read.
* @param[in] regNum,Number of register will be read.
* @param[in] IDStart,Start ID
* @param[in] IDNum,Numbers of Dynamixel servo
* @param[in] pDat,Pointer of data ready to be written to Dynamixel servo regs
                  eg:IDStart = 0,IDNum = 3,regNum = 4
                  D00,D01,D02,D03,D10,D11,D12,D13,D20,D21,D22,D23
* @returns ERR_NONE Successful
* others Failed
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysSendDynamixelBroadcastSyncWritePacket(EwayMotor_Device_Type devType,u8 regStartAdd,u8 regNum,u8 IDStart,u8 IDNum,u8* pDat)
{
    COM_PORT_E unPort;
    u8 dat[50]={0};
    //s8 res=ERR_NONE;
    u16 sum=0;
    u8 i,j;
    u8* pd;
    QueueHandle_t pQ;
    
    if(sysGetMotorCommPort(devType,&unPort)!=ERR_NONE)             //!< get com ID
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(sysGetMotorSendQueue(devType,&pQ))
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(regNum>GRIPPER_SYNC_WR_MAX_REG_NUM)                        //!< 系统限定syncWrite操作每次写寄存器不超过10个，可根据情况修改。
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(IDNum>GripperNum)                                         //!< 系统限定Gripper电机个数为系统最大爪子数量
    {
        return ERR_INPUT_PARAMETERS;
    }

    //!< check the data length        
    dat[2] = GRIPPER_FRAME_HEADER0;
    dat[3] = GRIPPER_FRAME_HEADER1;
    dat[4] = GRIPPER_CMD_BROADCAST_ID;
    dat[5] = (regNum+1)*IDNum+4;       //!< datalength instruction + regstartaddr 
    dat[6] = DYNAMIXEL_CMD_SYNC_WR;    //!< instruction:READ_DATA
    dat[7] = regStartAdd;              //!< register start address
    dat[8] = regNum;                   //!< number of register

    for(i=0;i<IDNum;i++)
    {
        dat[((regNum+1)*i)+9] = (IDStart+i);

        pd = &dat[((regNum+1)*i)+10];

        for(j=0;j<regNum;j++)
        {
            *(pd+j) = *(pDat+(i*regNum)+j);
        }        
    }
    
    for(i=4;i<((regNum+1)*IDNum+9);i++)
    {
        sum += dat[i];
    }

    dat[((regNum+1)*IDNum+9)] = (u8)(~sum);               //!<check sum

    //Send:
    //res = UartDmaTransferConfig(unPort,dat,((regNum+1)*IDNum+8));
    dat[0] = 0;
    dat[1] = ((regNum+1)*IDNum+8);
    if(pdTRUE != xQueueSend(pQ,dat,0))
    {
        return ERR_PROCESS_FAILED;
    }
    
    return ERR_NONE;
}



s8 sysProcessDynamixelRecvdDataPacket(EwayMotor_Device_Type devType)
{
    COM_PORT_E uni;
    u8 dat[COM_RX_BUFFER_SINGLE_OP_SIZE]={0};
    u16 len=0;
    s8 res=0;  
    
    if(sysGetMotorCommPort(devType,&uni)!=ERR_NONE)             //get com ID
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    if(ComRecv[uni].RxCount>=6)                                 //header0 header1 slaveAddr length Status ... checkSum
    {        
        res = g_GetMotorRecvdData(dat,&len,&ComRecv[uni]);        //copy data to temp buffer        
    }
    
    if(len>0)
    {
        res = sysParseDynamixelMotorRecvdData(dat,len,devType);
    }
        
    return res;
}


/* --------------------------------------------------------------------------*/
/**
* @name sysGetDynamixelMotorValidRecvdRegs
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
s8 sysGetDynamixelMotorValidRecvdRegs(EwayMotor_Device_Type devType,u8 iD,u8 errCode,u8 rgStart,u8 rgNum,u8* pbuf)
{
    u8 id=0;
    s8 res=0;
    
    if((iD>=(GripperStartID+GripperNum))||(iD<GripperStartID))    //!< check gripper ID
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    if(devType!=EWAYBOT_GRIPPERS)                         //!< check device type
    {
        return ERR_INPUT_PARAMETERS;
    }

    if(rgNum>DYNAMIXEL_RW_REG_NUMS_MAX)                   // check register number
    {
        return ERR_INPUT_PARAMETERS;
    }

    id = iD-1;

    switch(rgStart)
    {
        case DYNAMIXEL_REG_CurPosL:

            memcpy(&sysDyServo[id].unCurPosL,pbuf,rgNum);
            memcpy(&((EwayGrippers.mState.pFromMotor+id)->mRegs[1]),pbuf,rgNum);
            (EwayGrippers.mState.pFromMotor+id)->mRegs[0] = errCode;
            

            //!< 更新相应电机的通信状态
            EwayGrippers.mCommStatus |= (0x0001<<id);            
    
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_Motor_2_Emb_MASK)\
            &&(pDebug->secondFun&SecFunDebugCtrl_SinglReadResp_MASK)&&(pDebug->jointSw&JointSwDebugCtrl_Gripper_MASK))
            {
                res = Bsp_printf("read Gripper-2:recv (%d) Response DYNAMIXEL_REG_CurPosL pkt.",iD);  
            }
            
            res = ERR_NONE;
            
            break;
            
        default :res = ERR_INPUT_PARAMETERS;
        
            break;
    }

    return res;
}



/* --------------------------------------------------------------------------*/
/**
* @name sysParseDynamixelMotorRecvdData
* @brief 
* @details To analysis the data received from Com port Gripper Motor.
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
s8 sysParseDynamixelMotorRecvdData(u8* pbuf,u16 length,EwayMotor_Device_Type devType)
{
    u16 i=0,j;
    u8 len=0;
    u16 sum0=0;
    u16 hder=0;
    u8 sAddr,fErr;

    if(length>COM_RX_BUFFER_SINGLE_OP_SIZE)          //packet size exceeded,error
    {
        return ERR_INPUT_PARAMETERS;
    }
    
    while(i<length)                                   //parse the msg packet
    {
        hder = (pbuf[i])+((pbuf[i+1])<<8);                //search the header        
        
        if(hder!=COMM_DYNAMIXEL_MOTOR_MSG_HEADER) 
        {
            i += 1; continue;
        }
            
        sAddr = pbuf[i+2];                        //check slave addr
            
        if((sAddr>(GripperNum))||(sAddr==0))    //check ID
        {
            i += 1; continue;
        }            
            
        len = *(pbuf+i+3);                        //check lenght

        if((len<COMM_DYNAMIXEL_RECVD_MSG_LENGTH_MINI)||(len>COMM_DYNAMIXEL_RECVD_MSG_LENGTH_MAXI))
        {
            i += 1; continue;
        }            

        sum0 = 0;
        for(j=2;j<(len+3);j++)
        {
            sum0 += pbuf[i+j];
        }

        sum0 = ~sum0;

        if((pbuf[i+len+3])!=((u8)sum0))
        {
            i += 1; continue;
        }

        fErr = *(pbuf+i+4);                     //!< Error byte            

        if(len==0x0A)//!< 目前只接收10字节的返回数据，读8个以0x24为起始寄存器的数据包
        {
            if(ERR_NONE==sysGetDynamixelMotorValidRecvdRegs(EWAYBOT_GRIPPERS,sAddr,fErr,DYNAMIXEL_REG_CurPosL,8,(pbuf+i+5)))
            {
                i += (len+4);
            }
            else
            {
                i += 1; continue;
            }
        }
        else
        {
            i += (len + 4); continue;
        }
    }

    return ERR_NONE;
}


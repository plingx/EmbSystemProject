/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file PSM_Status.c
* @brief  下位机对PSM板卡发送查询命令。   然后接收PSM返回的状态。
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
#include "PSM_Status.h"
#include "../../CommProtocol/Emb_PSM/Emb_PSM.h"
#include "Emb_MyServo.h"


extern EwayPSMModule EwayPSM;
extern EwayBatteryChargerModule EwayBatteryCharger;

static s8 l_ReadPSMData(PSM *pPSMStatus,u8 *uData,u8 uLen);


/**
*********************************************************************************************************
* @name CopyUSARTRecieve
* @brief 复制串口接收数据
* @details 
*
* @param[in] *psUart 串口指针
* @param[in] *punBuf 数据拷贝到BUF
* @param[in] unNum   数据个数
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*/
/*
s8 CopyUSARTRecieve(UART_T *psUart,u8 *punBuf,u16 unNum)
{
   if(NULL == psUart)
     {
         return ERR_UART_NULL;
     }
     
     if(NULL == punBuf)
     {
         return ERR_POINTER_NULL;
     }
     
     u16 uni;
     for(uni=0;uni<unNum;uni++)
        {
            punBuf[uni]= psUart->pRxBuf[psUart->usRxRead];        // 从串口接收FIFO取一个数据 //
            DISABLE_INT();
            if (++psUart->usRxRead >= psUart->usRxBufSize)
            {
                psUart->usRxRead = 0;
            }
            psUart->usRxCount--;
            ENABLE_INT();
        }
        return ERR_NONE;
}    
*/
/*
s8 GetPSMStatus(PSM *pPSMStatus)
{
    u8 unBuf[256]={0},unLength=0,unID=0;
    u16 unCheckCRC=0,usCount=0;
    UART_T *pUart = ComToUart(COM7);
    if (NULL==pUart)
    {
        SysLogWrite(LogNormal,"PSM Status g_tUart7 Pointer is NULL");
        return ERR_UART_NULL;    
    };
    // usRxWrite 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 //
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();
    
    // 如果读和写索引相同，则返回0 //
    if (StatusLen_PSM!=usCount)          
    {///<第一次的长度总是不对，后续查明原因。 chengyi 2018.3.1.
        CopyUSARTRecieve(pUart,unBuf,usCount);///<将Uart中的字节清空。
      SysLogWrite(LogNormal,"PSM GetStatus Rx Count is error! Count = %d",usCount);
        return ERR_RX_DATA;
    }
    
    CopyUSARTRecieve(pUart,unBuf,usCount);//COPY串口数据//

    if(unBuf[0]!=0xDD||unBuf[1]!=0xDD) //判断帧头
    {
        SysLogWrite(LogNormal,"PSM RX Data Head is not 0xDDDD, Head = 0x%x%x",unBuf[0],unBuf[1]);
        return ERR_RX_DATA;
    }
        
    unID=unBuf[2];   //从站地址
    if(PSM_SlaveAddr!=unID)
    {
         SysLogWrite(LogNormal,"PSM RX Data SlaverAddr = 0x%x",unID);
       return ERR_RX_DATA;
    }
    
    unLength=unBuf[3];   //帧字节长度
    
        if(StatusLen_PSM==unLength)
        {
            unCheckCRC=CRC16_Modbus(unBuf, unLength-2);  //计算CRC校验位
            
            if(unCheckCRC!=unBuf[unLength-2]+(unBuf[unLength-1]<<8))
            {
                SysLogWrite(LogNormal,"PSM RX Data CRC is wrong.CheckCalCRC=0x%x, DataCRC=0x%x%x",unCheckCRC,unBuf[unLength-1],unBuf[unLength-2]);
                return ERR_RX_DATA;
            }
            else 
            {
                if(PSMRead_H!=unBuf[4])
                {
                    SysLogWrite(LogNormal,"PSM Rx Data Read CmdCode error. the CmdCode = %d",unBuf[4]);
                    return ERR_RX_DATA;
                }
                else
                {
                    ReadPSMData(pPSMStatus,unBuf,unLength);
                }
            }
        }        

    return ERR_NONE;
} 
*/


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
s8 g_GetPsmStatus(EwayPSMModule *pPSM,u8* pd,u8 lenth)
{
    s8 res=ERR_NONE;
    u8 fCode = *(pd+4);

    if(pPSM == NULL)
    {
        return ERR_POINTER_NULL;
    }

    switch(fCode)
    {
        case PSMPing_H:
        case PSMReborn_H:
        case PSMSetInit_H:
            break;
            
        case PSMRead_H:
            
                if(lenth == PSM_QUERY_PKT_RESPONSE_LEN)                   //!< 收到了PSM Query的回复包
                {
                    res = l_ReadPSMData(&(pPSM->sPSM),pd,lenth);       //!< 存储数据

                    pPSM->reRecord.sQueryRecd += 1;                    //!< 回复计数增1

                    //Bsp_printf("read Psm-2:Emb Recvd PSMRead_H respond pkt.");
                    
                }
            
            break;
            
        case PSMWrite_H:
                
                pPSM->sPSM.unPSMSlaveryStatus = *(pd+5);               //!< 更新status

                pPSM->reRecord.sExeCmRecd += 1;                           //!< 记录收到的回复
                
                //Bsp_printf("read Psm-2:Emb Recvd PSMWrite_H respond pkt.");
               
            break;
            
        default:
            break;
    }

    return res;
}

/*
s8 GetPSMStatus(PSM *pPSMStatus)
{
    u8 unBuf[256]={0},unLength=0,unID=0;
    s8 res;
    u16 unCheckCRC=0,usCount=0;
    u8 ta,tb,i;
    UART_T *pUart = ComToUart(COM7);
    if (NULL==pUart)
    {
        SysLogWrite(LogNormal,"PSM Status g_tUart7 Pointer is NULL");
        return ERR_UART_NULL;    
    };
    //!<  usRxWrite 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();
    
    //!< 如果读和写索引相同，则返回0 
    if (StatusLen_PSM>usCount)          
    {
//#if (EMB_SYS_DEBUG&&EMB_PSM_DEBUG)
        //Bsp_printf("read Psm-2:GetPSMStatus(),data not ready,dCnt:%d.",usCount);
//#endif
        return ERR_DATA_NOT_RDY;
    }
    
    CopyUSARTRecieve(pUart,unBuf,usCount);//!< COPY串口数据

    if(unBuf[0]!=0xDD||unBuf[1]!=0xDD)   //判断帧头
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
        res = SysLogWrite(LogNormal,"read Psm-2:PSM RX Data Head is not 0xDDDD, Head = 0x%x%x",unBuf[0],unBuf[1]);

        Bsp_printf("read Psm-2:PSM RX Data Head is not 0xDDDD, Head = 0x%x%x.SysLog:%d.",unBuf[0],unBuf[1],res,0,0,0);
#endif
        if(usCount <= 78)
        {
            ta = usCount/6;
            tb = usCount%6;
            for(i=0;i<ta;i++)
            {
                Bsp_printf("%2x-%2x-%2x-%2x-%2x-%2x-",(*(unBuf+(i*6))),(int)(*(unBuf+(i*6+1))),(int)(*(unBuf+(i*6+2))),(int)(*(unBuf+(i*6+3))),(int)(*(unBuf+(i*6+4))),(int)(*(unBuf+(i*6+5))));
            }
            
            if(tb>0)
            {
                for(i=0;i<tb;i++)
                {
                    Bsp_printf("%2x-",(int)(*(unBuf+(ta*6)+i)),0,0,0,0,0);
                }
            }
        }
        else
        {
            Bsp_printf("read Psm-2:Data nums is out of range:%d.",(int)usCount,0,0,0,0,0);
        }
        
        return ERR_RX_DATA_HEADER;
    }
        
    unID=unBuf[2];                       //从站地址
    if(PSM_SlaveAddr!=unID)
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
        res = SysLogWrite(LogNormal,"read Psm-2:PSM RX Data SlaverAddr = 0x%x",unID);

        Bsp_printf("read Psm-2:PSM RX Data SlaverAddr = 0x%x.SysLog:%d.",unID,res,0,0,0,0);
#endif    
         
       return ERR_RX_DATA_ADDR;
    }
    
    unLength=unBuf[3];                   //帧字节长度
    
    if(StatusLen_PSM==unLength)
    {
        unCheckCRC=CRC16_Modbus(unBuf, unLength-2);  //计算CRC校验位
            
        if(unCheckCRC!=unBuf[unLength-2]+(unBuf[unLength-1]<<8))
        {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
        res = SysLogWrite(LogNormal,"read Psm-2:PSM RX Data CRC is wrong.CheckCalCRC=0x%x, DataCRC=0x%x%x",unCheckCRC,unBuf[unLength-1],unBuf[unLength-2]);

        Bsp_printf("read Psm-2:PSM RX Data CRC wrong.CheckCalCRC=0x%x, DataCRC=0x%x%x.SysLog:%d",unCheckCRC,unBuf[unLength-1],unBuf[unLength-2],res,0,0);
#endif                    
            return ERR_RX_DATA_CRC;
        }
        else 
        {
            if(PSMRead_H!=unBuf[4])
            {
                
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
                res = SysLogWrite(LogNormal,"read Psm-2:PSM Rx Data Read CmdCode error. the CmdCode = %d",unBuf[4]);
                
                Bsp_printf("read Psm-2:PSM Rx Data Read CmdCode error.CmdCode = %d.SysLog:%d.",unBuf[4],res,0,0,0,0);
#endif                
                return ERR_RX_DATA_LEN;
            }
            else
            {
                res = ReadPSMData(pPSMStatus,unBuf,unLength);
            }
        }
    }        

    return res;
} 
*/


s8 SendQueryPSMStatus(void)
{    
    u8 uPSMRegAddr[2]={PSM_LEDCtrl,0x22}; // 以0x35为起始地址，连续读取0x22个寄存器
    SendQuery_PSM(PSM_SlaveAddr,uPSMRegAddr,2);
    EwayPSM.reRecord.sQueryRecd += (0x01<<4);              //!< 记录发送Query
    return ERR_NONE;
}


static s8 l_ReadPSMData(PSM *pPSMStatus,u8 *uData,u8 uLen)
{
    u8 uni=0;
    
    if(NULL==pPSMStatus||NULL==uData)
    {
        return ERR_POINTER_NULL;
    }

    pPSMStatus->unPSMSlaveryStatus = uData[5];
    pPSMStatus->unRobotLedStatus   = uData[6];   
    pPSMStatus->unRobotSwitchStatus= uData[7];   
    pPSMStatus->unRobotCurrentStatus=uData[8];    
    pPSMStatus->unPSM_PowerSwitchStatus = uData[9];
    pPSMStatus->unPSM_Status1      = uData[10];    
    pPSMStatus->unPSM_Status2      = uData[11];

    for(uni=0;uni<7;uni++)
    {
        pPSMStatus->sPSM_Out[uni].unVoltage=uData[14+2*uni]+(uData[15+2*uni]<<8);
    }
    
    for(uni=0;uni<6;uni++)              //!< ling-20180823 PSM应读取6路电流，DC5V&12V&9V 3路电流之和存于PSM的Reg0x55,0x56中
    {
        pPSMStatus->sPSM_Out[uni].unCurrent=uData[28+2*uni]+(uData[29+2*uni]<<8);
    }

    pPSMStatus->sPSM_Out[6].unCurrent = pPSMStatus->sPSM_Out[5].unCurrent;
    
    return ERR_NONE;
}

s8 ReportPSMStatusToPC(PSM *pPSMStatus,u8 *uData, u8 *uLen)
{
  u8 uni=0;
    
  ///PowerLevel
  //电源开关板七路电压电流状态:   0 sPC;   1 sLeftArm_PSM;   2 sRightArm_PSM;   3 sWheel_PSM;   4 sElse_PSM;   5 sDC5V_PSM;   6 sDC12V_PSM;
    for(uni=0;uni<7;uni++)
    {
        /*电压，数值放大了100倍，单位0.01V*/
        uData[uni*4+10]=0xFF&pPSMStatus->sPSM_Out[uni].unVoltage;
        uData[uni*4+11]=0xFF&(pPSMStatus->sPSM_Out[uni].unVoltage>>8);
        
        /*电流，数值放大了100倍，单位0.01A*/
        uData[uni*4+12]=0xFF&pPSMStatus->sPSM_Out[uni].unCurrent;
        uData[uni*4+13]=0xFF&(pPSMStatus->sPSM_Out[uni].unCurrent>>8);
    }
    
    uni=uni-1;
    
    //机器人当前开机状态：   0代表未开机状态，   1代表已开机正在启动，   2代表已开机，   3代表休眠状态，   4代表急停状态，   5等待关机状态
    uData[uni*4+14]=pPSMStatus->unRobotCurrentStatus;

    //各路开关状态:      bit0代表上位机供电回路,1代表闭合，0代表打开状态；      Bit1代表左臂；  Bit2代表底盘；  Bit3代表右臂；  Bit4代表其他；  Bit5代表DCDC；  Bit6代表上位机已开机，1：已开机
    uData[uni*4+15]=pPSMStatus->unPSM_PowerSwitchStatus;

    //电源开关板当前状态2:   bit1-0代表其他模块；  00代表正常，01代表欠压，10代表过流；   bit3-2代表DCDC5V回路；   bit5-4代表DCDC12V回路；
    uData[uni*4+16]=pPSMStatus->unPSM_Status1;

    //电源开关板当前状态2:  bit1-0代表其他模块；  00代表正常，01代表欠压，10代表过流；   bit3-2代表DCDC5V回路；   bit5-4代表DCDC12V回路；
    uData[uni*4+17]=pPSMStatus->unPSM_Status2;
    
    return ERR_NONE;
}





#include "includes.h"
//#include "bsp.h"
//#include "Emb_MyServo.h"
#include "Emb_BatteryCharger.h"
//#include "BatteryCharger_Status.h"
//#include "PSM_Status.h"
//#include "PSM_Cmd.h"

extern EwayBatteryChargerModule EwayBatteryCharger;


static s8 l_ReadBatteryData(ModuleBatteryCharger *pBatteryStatus,u8 *puData, u8 uLen);


s8 SendQueryBatteryChargerStatus(void)
{    
    u8 unRegAddr[2]={BatteryCharge_CtrlFaultStatus,0x0D};  //从0x1E寄存器开始读取，共读取0x0d个寄存器
    
    SendQuery_BatteryCharger(BatteryCharger_SlaveAddr,unRegAddr,2);
    
    EwayBatteryCharger.reRecord.sQueryRecd += (0x01<<4);              //!< 记录发送Query
    
    return ERR_NONE;
}


s8 g_GetBatteryStatus(EwayBatteryChargerModule *psBattery,u8* pd,u8 lenth)
{
    s8 res=ERR_NONE;
    u8 fCode = *(pd+4);
    
    if(psBattery == NULL)
    {
        return ERR_POINTER_NULL;
    }
    
    switch(fCode)
    {
        case BatteryPing_H:
        case BatterySetInit_H:
            break;
                
        case BatteryRead_H:
                
                if(lenth == BTCharger_QUERY_PKT_RESPONSE_LEN)     //!< 收到了PSU Query的回复包
                {
                    res = l_ReadBatteryData(&(psBattery->sBattCharger),pd,lenth);

                    psBattery->reRecord.sQueryRecd += 1;
                    
                    //Bsp_printf("read Psu-2:Emb Recvd BatteryRead_H respond pkt.");                 
                }
                
            break;
                
        case BatteryWrite_H:
                    
                psBattery->sBattCharger.unBatterySlaveryStatus = *(pd+5);    //!< 更新status
    
                psBattery->reRecord.sExeCmRecd += 1;             //!< 记录收到的回复                

                Bsp_printf("read Psu-2:Emb Recvd BatteryWrite_H respond pkt.");
                   
                break;
                
        default:
            break;
    }
    
    return res;
}

s8 SendWriteBatteryChargerBaudRate(void)
{
    u8 dat[10]={0x05,0x01,0x00};  //!< 向reg05写00，令波特率为9600

    
    BatteryCharger_WRITE(BatteryCharger_SlaveAddr,dat,3);

	  return ERR_NONE;
}

/*
s8 GetBatteryStatus(ModuleBatteryCharger *psBatteryStatus)
{
    if(NULL==psBatteryStatus)
    {
        return ERR_POINTER_NULL;
    }
        
    u8 unBuf[256]={0},unLength=0,unID=0;
    u16 unCheckCRC=0,usCount=0;
    s8 res;
    UART_T *pUart = ComToUart(COM7);
    if (NULL==pUart)
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
        Bsp_printf("BatteryCharger Status g_tUart7 Pointer is NULL",0,0,0,0,0,0);    

        SysLogWrite(LogNormal,"BatteryCharger Status g_tUart7 Pointer is NULL");
#endif
        return ERR_UART_NULL;    
    };

    //!< usRxWrite 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();
        
    //!< 如果读和写索引相同，则返回0 
    if (StatusLen_PSU > usCount)       
    {
        return ERR_DATA_NOT_RDY;
    }
        
    CopyUSARTRecieve(pUart,unBuf,usCount);//!< COPY串口数据
    
    if(unBuf[0]!=0xEE||unBuf[1]!=0xEE) //判断帧头
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)

        res = SysLogWrite(LogNormal,"BatteryCharger RX Data Head is not 0xEEEE, Head=0x%x%x",unBuf[0],unBuf[1]);

        Bsp_printf("BatteryCharger RX Data Head is not 0xEEEE, Head=0x%x%x.SysLog:%d.",unBuf[0],unBuf[1],res,0,0,0);

#endif
        return ERR_RX_DATA_HEADER;
    }
            
    unID=unBuf[2];     //从站地址
    if(BatteryCharger_SlaveAddr!=unID)
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
        res = SysLogWrite(LogNormal,"BatteryCharger RX Data SlaverAddr = 0x%x",unID);

        Bsp_printf("BatteryCharger RX Data SlaverAddr = 0x%x.SysLog:%d.",unID,res,0,0,0,0);        
#endif
        return ERR_RX_DATA_ADDR;
    }
        
    unLength=unBuf[3];     //帧字节长度
        
    if(StatusLen_PSU==unLength)
    {
        unCheckCRC=CRC16_Modbus(unBuf, unLength-2);  //计算CRC校验位
            
        if(unCheckCRC!=unBuf[unLength-2]+(unBuf[unLength-1]<<8))
        {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
            res = SysLogWrite(LogNormal,"BatteryCharger RX Data CRC is incorrect. CheckCalCRC=0x%x, DataCRC=0x%x%x",unCheckCRC,unBuf[unLength-1],unBuf[unLength-2]);

            Bsp_printf("BatteryCharger RX Data CRC is incorrect. CheckCalCRC=0x%x, DataCRC=0x%x%x.SysLog:%d.",unCheckCRC,unBuf[unLength-1],unBuf[unLength-2],res,0,0);

#endif        
            return ERR_RX_DATA_CRC;
        }
        else 
        {
            if(BatteryRead_H!=unBuf[4])
            {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)            
                res = SysLogWrite(LogNormal,"BatteryCharger Rx Data Read CmdCode error. the CmdCode = %d",unBuf[4]);

                Bsp_printf("BatteryCharger Rx Data Read CmdCode error. the CmdCode = %d.SysLog:%d.",unBuf[4],res,0,0,0,0);
#endif
                return ERR_RX_DATA_LEN;
            }
            else
            {
                res = ReadBatteryData(psBatteryStatus,unBuf,unLength);
                
                if(res!=ERR_NONE)
                {
                        Bsp_printf("read Psu-2:ReadBatteryData() failed,rt:%d",res,0,0,0,0,0);
                }
            }
        }
    }    

    if(res!=ERR_NONE)
    {
        Bsp_printf("read Psu-2:ReadBatteryData() failed,rt:%d",res,0,0,0,0,0);
    }
    
    return res;
} 
*/

/*
s8 GetBatteryStatus(ModuleBatteryCharger *psBatteryStatus)
{
    if(NULL==psBatteryStatus)
    {
        return ERR_POINTER_NULL;
    }
    
    u8 unBuf[256]={0},unLength=0,unID=0;
    u16 unCheckCRC=0,usCount=0;
    UART_T *pUart = ComToUart(COM7);
    if (NULL==pUart)
    {
        SysLogWrite(LogNormal,"BatteryCharger Status g_tUart7 Pointer is NULL");
        return ERR_UART_NULL;    
    };
    // usRxWrite 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 //
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();
    
    // 如果读和写索引相同，则返回0 //
    if (StatusLen_PSU!=usCount)          
    {///<第一次的长度总是不对，后续查明原因。 chengyi 2018.3.1.
        CopyUSARTRecieve(pUart,unBuf,usCount);///<将Uart中的字节清空。
      SysLogWrite(LogNormal,"BatteryCharger GetStatus Rx Count is error! Count = %d",usCount);
        return ERR_RX_DATA;
    }
    
    CopyUSARTRecieve(pUart,unBuf,usCount);//COPY串口数据//

    if(unBuf[0]!=0xEE||unBuf[1]!=0xEE) //判断帧头
    {
        SysLogWrite(LogNormal,"BatteryCharger RX Data Head is not 0xEEEE, Head = 0x%x%x",unBuf[0],unBuf[1]);
        return ERR_RX_DATA;
    }
        
    unID=unBuf[2];   //从站地址
    if(BatteryCharger_SlaveAddr!=unID)
    {
         SysLogWrite(LogNormal,"BatteryCharger RX Data SlaverAddr = 0x%x",unID);
       return ERR_RX_DATA;
    }
    
    unLength=unBuf[3];   //帧字节长度
    
    if(StatusLen_PSU==unLength)
    {
        unCheckCRC=CRC16_Modbus(unBuf, unLength-2);  //计算CRC校验位
        
        if(unCheckCRC!=unBuf[unLength-2]+(unBuf[unLength-1]<<8))
        {
            SysLogWrite(LogNormal,"BatteryCharger RX Data CRC is incorrect. CheckCalCRC=0x%x, DataCRC=0x%x%x",unCheckCRC,unBuf[unLength-1],unBuf[unLength-2]);
            return ERR_RX_DATA;
        }
        else 
        {
            if(BatteryRead_H!=unBuf[4])
            {
                SysLogWrite(LogNormal,"BatteryCharger Rx Data Read CmdCode error. the CmdCode = %d",unBuf[4]);
                return ERR_RX_DATA;
            }
            else
            {
                ReadBatteryData(psBatteryStatus,unBuf,unLength);
            }
        }
    }        

    return ERR_NONE;
} 
*/

/*
s8 GetBatteryStatus(ModuleBatteryCharger *psBatteryStatus)
{
    if(NULL==psBatteryStatus)
    {
        return ERR_POINTER_NULL;
    }
        
    u8 unBuf[256]={0},unLength=0,unID=0;
    u16 unCheckCRC=0,usCount=0;
    s8 res;
    UART_T *pUart = ComToUart(COM7);
    if (NULL==pUart)
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
        Bsp_printf("BatteryCharger Status g_tUart7 Pointer is NULL",0,0,0,0,0,0);    

        SysLogWrite(LogNormal,"BatteryCharger Status g_tUart7 Pointer is NULL");
#endif
        return ERR_UART_NULL;    
    };

    //!< usRxWrite 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();
        
    //!< 如果读和写索引相同，则返回0 
    if (StatusLen_PSU > usCount)       
    {
        return ERR_DATA_NOT_RDY;
    }
        
    CopyUSARTRecieve(pUart,unBuf,usCount);//!< COPY串口数据
    
    if(unBuf[0]!=0xEE||unBuf[1]!=0xEE) //判断帧头
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)

        res = SysLogWrite(LogNormal,"BatteryCharger RX Data Head is not 0xEEEE, Head=0x%x%x",unBuf[0],unBuf[1]);

        Bsp_printf("BatteryCharger RX Data Head is not 0xEEEE, Head=0x%x%x.SysLog:%d.",unBuf[0],unBuf[1],res,0,0,0);

#endif
        return ERR_RX_DATA_HEADER;
    }
            
    unID=unBuf[2];     //从站地址
    if(BatteryCharger_SlaveAddr!=unID)
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
        res = SysLogWrite(LogNormal,"BatteryCharger RX Data SlaverAddr = 0x%x",unID);

        Bsp_printf("BatteryCharger RX Data SlaverAddr = 0x%x.SysLog:%d.",unID,res,0,0,0,0);        
#endif
        return ERR_RX_DATA_ADDR;
    }
        
    unLength=unBuf[3];     //帧字节长度
        
    if(StatusLen_PSU==unLength)
    {
        unCheckCRC=CRC16_Modbus(unBuf, unLength-2);  //计算CRC校验位
            
        if(unCheckCRC!=unBuf[unLength-2]+(unBuf[unLength-1]<<8))
        {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
            res = SysLogWrite(LogNormal,"BatteryCharger RX Data CRC is incorrect. CheckCalCRC=0x%x, DataCRC=0x%x%x",unCheckCRC,unBuf[unLength-1],unBuf[unLength-2]);

            Bsp_printf("BatteryCharger RX Data CRC is incorrect. CheckCalCRC=0x%x, DataCRC=0x%x%x.SysLog:%d.",unCheckCRC,unBuf[unLength-1],unBuf[unLength-2],res,0,0);

#endif        
            return ERR_RX_DATA_CRC;
        }
        else 
        {
            if(BatteryRead_H!=unBuf[4])
            {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)            
                res = SysLogWrite(LogNormal,"BatteryCharger Rx Data Read CmdCode error. the CmdCode = %d",unBuf[4]);

                Bsp_printf("BatteryCharger Rx Data Read CmdCode error. the CmdCode = %d.SysLog:%d.",unBuf[4],res,0,0,0,0);
#endif
                return ERR_RX_DATA_LEN;
            }
            else
            {
                res = ReadBatteryData(psBatteryStatus,unBuf,unLength);
                
                if(res!=ERR_NONE)
                {
                        Bsp_printf("read Psu-2:ReadBatteryData() failed,rt:%d",res,0,0,0,0,0);
                }
            }
        }
    }    

    if(res!=ERR_NONE)
    {
        Bsp_printf("read Psu-2:ReadBatteryData() failed,rt:%d",res,0,0,0,0,0);
    }
    
    return res;
} 
*/

static s8 l_ReadBatteryData(ModuleBatteryCharger *pBatteryStatus,u8 *puData, u8 uLen)
{
    if(NULL == pBatteryStatus || NULL == puData)
    {
        return ERR_POINTER_NULL;
    }
    
    pBatteryStatus->unStatus=puData[6]+(puData[7]<<8);
    pBatteryStatus->unExVoltage=puData[8]+(puData[9]<<8);
    pBatteryStatus->unOutCurrent=puData[10]+(puData[11]<<8);
    pBatteryStatus->unTemperature=puData[12];
    pBatteryStatus->unBatteryVoltage=puData[13]+(puData[14]<<8);
    pBatteryStatus->unBatteryInCurrent=puData[15]+(puData[16]<<8);
    pBatteryStatus->unBatteryCapacity=puData[17]+(puData[18]<<8);
    
    return ERR_NONE;
}


/**
*********************************************************************************************************
* @name ReportBatteryStatusToPC
* @brief 将电源状态生成数据包
* @details 
*
* @param[in] none
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* @author 
*********************************************************************************************************
*/
s8 ReportBatteryStatusToPC(ModuleBatteryCharger *pBatteryStatus,u8 *uData, u8 *uLen)
{
    ///充电控制工作板状1      bit0：过热，bit1：输出过流（输出电流超过10A），bit2:输入欠压，bit3：输入过压，bit4：电池欠压，bit5：输出短路，bit6：电池即将输出关闭，bit7：未定义错误；以上0均代表无故障
    uData[10]=0xFF&(pBatteryStatus->unStatus);

    ///充电控制工作板状2      bit0：负载供电电源（0是外部电源，1是电池），bit1：电池充满（1是充满），bit6-bit7：00是未充电，01是充电恒流，10是充电恒压，11是充电涓流；其他未定义位补
    uData[11]=0xFF&(pBatteryStatus->unStatus>>8);

    ///外部输入电压，数值放大了100倍，单位0.01V
    uData[12]=0xFF&(pBatteryStatus->unExVoltage);
    uData[13]=0xFF&(pBatteryStatus->unExVoltage>>8);

    ///负载电流，数值放大了100倍，单位0.01A
    uData[14]=0xFF&(pBatteryStatus->unOutCurrent);
    uData[15]=0xFF&(pBatteryStatus->unOutCurrent>>8);

    ///驱动板当前温度
    uData[16]=0xFF&(pBatteryStatus->unTemperature);

    ///电池电压，数值放大了100倍，单位0.01V
    uData[17]=0xFF&(pBatteryStatus->unBatteryVoltage);
    uData[18]=0xFF&(pBatteryStatus->unBatteryVoltage>>8);

    ///电池充电电流，数值放大了100倍，单位0.01A
    uData[19]=0xFF&pBatteryStatus->unBatteryInCurrent;
    uData[20]=0xFF&(pBatteryStatus->unBatteryInCurrent>>8);

    ///电池当前容量，数值放大了100倍，单位0.01%
    uData[21]=0xFF&(pBatteryStatus->unBatteryCapacity);
    uData[22]=0xFF&(pBatteryStatus->unBatteryCapacity>>8);
    
    return ERR_NONE;
}


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
    u8 unRegAddr[2]={BatteryCharge_CtrlFaultStatus,0x0D};  //��0x1E�Ĵ�����ʼ��ȡ������ȡ0x0d���Ĵ���
    
    SendQuery_BatteryCharger(BatteryCharger_SlaveAddr,unRegAddr,2);
    
    EwayBatteryCharger.reRecord.sQueryRecd += (0x01<<4);              //!< ��¼����Query
    
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
                
                if(lenth == BTCharger_QUERY_PKT_RESPONSE_LEN)     //!< �յ���PSU Query�Ļظ���
                {
                    res = l_ReadBatteryData(&(psBattery->sBattCharger),pd,lenth);

                    psBattery->reRecord.sQueryRecd += 1;
                    
                    //Bsp_printf("read Psu-2:Emb Recvd BatteryRead_H respond pkt.");                 
                }
                
            break;
                
        case BatteryWrite_H:
                    
                psBattery->sBattCharger.unBatterySlaveryStatus = *(pd+5);    //!< ����status
    
                psBattery->reRecord.sExeCmRecd += 1;             //!< ��¼�յ��Ļظ�                

                Bsp_printf("read Psu-2:Emb Recvd BatteryWrite_H respond pkt.");
                   
                break;
                
        default:
            break;
    }
    
    return res;
}

s8 SendWriteBatteryChargerBaudRate(void)
{
    u8 dat[10]={0x05,0x01,0x00};  //!< ��reg05д00�������Ϊ9600

    
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

    //!< usRxWrite �������жϺ����б���д���������ȡ�ñ���ʱ����������ٽ������� 
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();
        
    //!< �������д������ͬ���򷵻�0 
    if (StatusLen_PSU > usCount)       
    {
        return ERR_DATA_NOT_RDY;
    }
        
    CopyUSARTRecieve(pUart,unBuf,usCount);//!< COPY��������
    
    if(unBuf[0]!=0xEE||unBuf[1]!=0xEE) //�ж�֡ͷ
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)

        res = SysLogWrite(LogNormal,"BatteryCharger RX Data Head is not 0xEEEE, Head=0x%x%x",unBuf[0],unBuf[1]);

        Bsp_printf("BatteryCharger RX Data Head is not 0xEEEE, Head=0x%x%x.SysLog:%d.",unBuf[0],unBuf[1],res,0,0,0);

#endif
        return ERR_RX_DATA_HEADER;
    }
            
    unID=unBuf[2];     //��վ��ַ
    if(BatteryCharger_SlaveAddr!=unID)
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
        res = SysLogWrite(LogNormal,"BatteryCharger RX Data SlaverAddr = 0x%x",unID);

        Bsp_printf("BatteryCharger RX Data SlaverAddr = 0x%x.SysLog:%d.",unID,res,0,0,0,0);        
#endif
        return ERR_RX_DATA_ADDR;
    }
        
    unLength=unBuf[3];     //֡�ֽڳ���
        
    if(StatusLen_PSU==unLength)
    {
        unCheckCRC=CRC16_Modbus(unBuf, unLength-2);  //����CRCУ��λ
            
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
    // usRxWrite �������жϺ����б���д���������ȡ�ñ���ʱ����������ٽ������� //
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();
    
    // �������д������ͬ���򷵻�0 //
    if (StatusLen_PSU!=usCount)          
    {///<��һ�εĳ������ǲ��ԣ���������ԭ�� chengyi 2018.3.1.
        CopyUSARTRecieve(pUart,unBuf,usCount);///<��Uart�е��ֽ���ա�
      SysLogWrite(LogNormal,"BatteryCharger GetStatus Rx Count is error! Count = %d",usCount);
        return ERR_RX_DATA;
    }
    
    CopyUSARTRecieve(pUart,unBuf,usCount);//COPY��������//

    if(unBuf[0]!=0xEE||unBuf[1]!=0xEE) //�ж�֡ͷ
    {
        SysLogWrite(LogNormal,"BatteryCharger RX Data Head is not 0xEEEE, Head = 0x%x%x",unBuf[0],unBuf[1]);
        return ERR_RX_DATA;
    }
        
    unID=unBuf[2];   //��վ��ַ
    if(BatteryCharger_SlaveAddr!=unID)
    {
         SysLogWrite(LogNormal,"BatteryCharger RX Data SlaverAddr = 0x%x",unID);
       return ERR_RX_DATA;
    }
    
    unLength=unBuf[3];   //֡�ֽڳ���
    
    if(StatusLen_PSU==unLength)
    {
        unCheckCRC=CRC16_Modbus(unBuf, unLength-2);  //����CRCУ��λ
        
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

    //!< usRxWrite �������жϺ����б���д���������ȡ�ñ���ʱ����������ٽ������� 
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();
        
    //!< �������д������ͬ���򷵻�0 
    if (StatusLen_PSU > usCount)       
    {
        return ERR_DATA_NOT_RDY;
    }
        
    CopyUSARTRecieve(pUart,unBuf,usCount);//!< COPY��������
    
    if(unBuf[0]!=0xEE||unBuf[1]!=0xEE) //�ж�֡ͷ
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)

        res = SysLogWrite(LogNormal,"BatteryCharger RX Data Head is not 0xEEEE, Head=0x%x%x",unBuf[0],unBuf[1]);

        Bsp_printf("BatteryCharger RX Data Head is not 0xEEEE, Head=0x%x%x.SysLog:%d.",unBuf[0],unBuf[1],res,0,0,0);

#endif
        return ERR_RX_DATA_HEADER;
    }
            
    unID=unBuf[2];     //��վ��ַ
    if(BatteryCharger_SlaveAddr!=unID)
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
        res = SysLogWrite(LogNormal,"BatteryCharger RX Data SlaverAddr = 0x%x",unID);

        Bsp_printf("BatteryCharger RX Data SlaverAddr = 0x%x.SysLog:%d.",unID,res,0,0,0,0);        
#endif
        return ERR_RX_DATA_ADDR;
    }
        
    unLength=unBuf[3];     //֡�ֽڳ���
        
    if(StatusLen_PSU==unLength)
    {
        unCheckCRC=CRC16_Modbus(unBuf, unLength-2);  //����CRCУ��λ
            
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
* @brief ����Դ״̬�������ݰ�
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
    ///�����ƹ�����״1      bit0�����ȣ�bit1����������������������10A����bit2:����Ƿѹ��bit3�������ѹ��bit4�����Ƿѹ��bit5�������·��bit6����ؼ�������رգ�bit7��δ�����������0�������޹���
    uData[10]=0xFF&(pBatteryStatus->unStatus);

    ///�����ƹ�����״2      bit0�����ع����Դ��0���ⲿ��Դ��1�ǵ�أ���bit1����س�����1�ǳ�������bit6-bit7��00��δ��磬01�ǳ�������10�ǳ���ѹ��11�ǳ�����������δ����λ��
    uData[11]=0xFF&(pBatteryStatus->unStatus>>8);

    ///�ⲿ�����ѹ����ֵ�Ŵ���100������λ0.01V
    uData[12]=0xFF&(pBatteryStatus->unExVoltage);
    uData[13]=0xFF&(pBatteryStatus->unExVoltage>>8);

    ///���ص�������ֵ�Ŵ���100������λ0.01A
    uData[14]=0xFF&(pBatteryStatus->unOutCurrent);
    uData[15]=0xFF&(pBatteryStatus->unOutCurrent>>8);

    ///�����嵱ǰ�¶�
    uData[16]=0xFF&(pBatteryStatus->unTemperature);

    ///��ص�ѹ����ֵ�Ŵ���100������λ0.01V
    uData[17]=0xFF&(pBatteryStatus->unBatteryVoltage);
    uData[18]=0xFF&(pBatteryStatus->unBatteryVoltage>>8);

    ///��س���������ֵ�Ŵ���100������λ0.01A
    uData[19]=0xFF&pBatteryStatus->unBatteryInCurrent;
    uData[20]=0xFF&(pBatteryStatus->unBatteryInCurrent>>8);

    ///��ص�ǰ��������ֵ�Ŵ���100������λ0.01%
    uData[21]=0xFF&(pBatteryStatus->unBatteryCapacity);
    uData[22]=0xFF&(pBatteryStatus->unBatteryCapacity>>8);
    
    return ERR_NONE;
}


/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file PSM_Status.c
* @brief  ��λ����PSM�忨���Ͳ�ѯ���   Ȼ�����PSM���ص�״̬��
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
* @brief ���ƴ��ڽ�������
* @details 
*
* @param[in] *psUart ����ָ��
* @param[in] *punBuf ���ݿ�����BUF
* @param[in] unNum   ���ݸ���
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
            punBuf[uni]= psUart->pRxBuf[psUart->usRxRead];        // �Ӵ��ڽ���FIFOȡһ������ //
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
    // usRxWrite �������жϺ����б���д���������ȡ�ñ���ʱ����������ٽ������� //
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();
    
    // �������д������ͬ���򷵻�0 //
    if (StatusLen_PSM!=usCount)          
    {///<��һ�εĳ������ǲ��ԣ���������ԭ�� chengyi 2018.3.1.
        CopyUSARTRecieve(pUart,unBuf,usCount);///<��Uart�е��ֽ���ա�
      SysLogWrite(LogNormal,"PSM GetStatus Rx Count is error! Count = %d",usCount);
        return ERR_RX_DATA;
    }
    
    CopyUSARTRecieve(pUart,unBuf,usCount);//COPY��������//

    if(unBuf[0]!=0xDD||unBuf[1]!=0xDD) //�ж�֡ͷ
    {
        SysLogWrite(LogNormal,"PSM RX Data Head is not 0xDDDD, Head = 0x%x%x",unBuf[0],unBuf[1]);
        return ERR_RX_DATA;
    }
        
    unID=unBuf[2];   //��վ��ַ
    if(PSM_SlaveAddr!=unID)
    {
         SysLogWrite(LogNormal,"PSM RX Data SlaverAddr = 0x%x",unID);
       return ERR_RX_DATA;
    }
    
    unLength=unBuf[3];   //֡�ֽڳ���
    
        if(StatusLen_PSM==unLength)
        {
            unCheckCRC=CRC16_Modbus(unBuf, unLength-2);  //����CRCУ��λ
            
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
            
                if(lenth == PSM_QUERY_PKT_RESPONSE_LEN)                   //!< �յ���PSM Query�Ļظ���
                {
                    res = l_ReadPSMData(&(pPSM->sPSM),pd,lenth);       //!< �洢����

                    pPSM->reRecord.sQueryRecd += 1;                    //!< �ظ�������1

                    //Bsp_printf("read Psm-2:Emb Recvd PSMRead_H respond pkt.");
                    
                }
            
            break;
            
        case PSMWrite_H:
                
                pPSM->sPSM.unPSMSlaveryStatus = *(pd+5);               //!< ����status

                pPSM->reRecord.sExeCmRecd += 1;                           //!< ��¼�յ��Ļظ�
                
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
    //!<  usRxWrite �������жϺ����б���д���������ȡ�ñ���ʱ����������ٽ������� 
    DISABLE_INT();
    usCount = pUart->usRxCount;
    ENABLE_INT();
    
    //!< �������д������ͬ���򷵻�0 
    if (StatusLen_PSM>usCount)          
    {
//#if (EMB_SYS_DEBUG&&EMB_PSM_DEBUG)
        //Bsp_printf("read Psm-2:GetPSMStatus(),data not ready,dCnt:%d.",usCount);
//#endif
        return ERR_DATA_NOT_RDY;
    }
    
    CopyUSARTRecieve(pUart,unBuf,usCount);//!< COPY��������

    if(unBuf[0]!=0xDD||unBuf[1]!=0xDD)   //�ж�֡ͷ
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
        
    unID=unBuf[2];                       //��վ��ַ
    if(PSM_SlaveAddr!=unID)
    {
#if (EMB_SYS_DEBUG&&EMB_SYS_ERR_DEBUG)
        res = SysLogWrite(LogNormal,"read Psm-2:PSM RX Data SlaverAddr = 0x%x",unID);

        Bsp_printf("read Psm-2:PSM RX Data SlaverAddr = 0x%x.SysLog:%d.",unID,res,0,0,0,0);
#endif    
         
       return ERR_RX_DATA_ADDR;
    }
    
    unLength=unBuf[3];                   //֡�ֽڳ���
    
    if(StatusLen_PSM==unLength)
    {
        unCheckCRC=CRC16_Modbus(unBuf, unLength-2);  //����CRCУ��λ
            
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
    u8 uPSMRegAddr[2]={PSM_LEDCtrl,0x22}; // ��0x35Ϊ��ʼ��ַ��������ȡ0x22���Ĵ���
    SendQuery_PSM(PSM_SlaveAddr,uPSMRegAddr,2);
    EwayPSM.reRecord.sQueryRecd += (0x01<<4);              //!< ��¼����Query
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
    
    for(uni=0;uni<6;uni++)              //!< ling-20180823 PSMӦ��ȡ6·������DC5V&12V&9V 3·����֮�ʹ���PSM��Reg0x55,0x56��
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
  //��Դ���ذ���·��ѹ����״̬:   0 sPC;   1 sLeftArm_PSM;   2 sRightArm_PSM;   3 sWheel_PSM;   4 sElse_PSM;   5 sDC5V_PSM;   6 sDC12V_PSM;
    for(uni=0;uni<7;uni++)
    {
        /*��ѹ����ֵ�Ŵ���100������λ0.01V*/
        uData[uni*4+10]=0xFF&pPSMStatus->sPSM_Out[uni].unVoltage;
        uData[uni*4+11]=0xFF&(pPSMStatus->sPSM_Out[uni].unVoltage>>8);
        
        /*��������ֵ�Ŵ���100������λ0.01A*/
        uData[uni*4+12]=0xFF&pPSMStatus->sPSM_Out[uni].unCurrent;
        uData[uni*4+13]=0xFF&(pPSMStatus->sPSM_Out[uni].unCurrent>>8);
    }
    
    uni=uni-1;
    
    //�����˵�ǰ����״̬��   0����δ����״̬��   1�����ѿ�������������   2�����ѿ�����   3��������״̬��   4����ͣ״̬��   5�ȴ��ػ�״̬
    uData[uni*4+14]=pPSMStatus->unRobotCurrentStatus;

    //��·����״̬:      bit0������λ�������·,1����պϣ�0�����״̬��      Bit1������ۣ�  Bit2������̣�  Bit3�����ұۣ�  Bit4����������  Bit5����DCDC��  Bit6������λ���ѿ�����1���ѿ���
    uData[uni*4+15]=pPSMStatus->unPSM_PowerSwitchStatus;

    //��Դ���ذ嵱ǰ״̬2:   bit1-0��������ģ�飻  00����������01����Ƿѹ��10���������   bit3-2����DCDC5V��·��   bit5-4����DCDC12V��·��
    uData[uni*4+16]=pPSMStatus->unPSM_Status1;

    //��Դ���ذ嵱ǰ״̬2:  bit1-0��������ģ�飻  00����������01����Ƿѹ��10���������   bit3-2����DCDC5V��·��   bit5-4����DCDC12V��·��
    uData[uni*4+17]=pPSMStatus->unPSM_Status2;
    
    return ERR_NONE;
}





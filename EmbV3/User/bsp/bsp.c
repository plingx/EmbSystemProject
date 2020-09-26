/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file bsp.c
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2017-12-20
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2017/12/20 | 0.0.1 |          | Create file
*
*/
#include "bsp.h"
#include "includes.h"
#include <stdarg.h>


/**
* define the start reg addr of IMU to read
* @param None.
* @return None.
*/



extern EwayEmbSysModule EwayEmbSys;
extern EwayEmbFileSysModule EwayEmbFileSys;
extern QueueHandle_t EmbToLogWrQueHdl;
extern EwayEmbSysDebugModule* pDebug;
extern __IO uint32_t MySysTick;


s8 PrintfUart1DmaTransferConfig(u8 *pSndBuf,u16 len);

/* --------------------------------------------------------------------------*/
/**
* @name bsp_Init
* @brief 
* @details all interface in system initialization operation
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void bsp_Init(void)
{
    /*
        ����ST�̼���������ļ��Ѿ�ִ����CPUϵͳʱ�ӵĳ�ʼ�������Բ����ٴ��ظ�����ϵͳʱ�ӡ�
        �����ļ�������CPU��ʱ��Ƶ�ʡ��ڲ�Flash�����ٶȺͿ�ѡ���ⲿSRAM FSMC��ʼ����

        ϵͳʱ��ȱʡ����Ϊ168MHz�������Ҫ���ģ������޸� system_stm32f4xx.c �ļ�
    */
    //bsp_InitUart();     /* ��ʼ������ */
    //bsp_InitTimer();    /* ��ʼ��ϵͳ�δ�ʱ�� */

    /* ��Բ�ͬ��Ӧ�ó��������Ҫ�ĵײ�����ģ���ʼ������ */
  //CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������500Kbps
        //LEDInit();
     //UPDOWNInit();
     //IOInit();

    SysConfigNVICPriorityGroup();
    BspGpios_init();
    BspUarts_init();        ///<���õ���COM7<-->UART7��COM8 <--> UART8 
    BspDmaUart_init();
    BspI2c1_init();            //!< IMU data Interface
    BspI2c2_init();            //!< AT24C1024B Interface
    BspAdc3_init();            //!< Rope Displace Sensor
    BspTim9_Init();            //!< UltraSonic Radar
    BspSpi2_Init();            //!< SD card interface
    BspWatchDog_Init();
}

/* --------------------------------------------------------------------------*/
/**
* @name SysConfigNVICPriorityGroup
* @brief 
* @details system NVIC Priority Group config
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void SysConfigNVICPriorityGroup(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);     //!< 4 bits for pre-emption,0 bit for subpriority
}


/*
s8 Bsp_printf(
    char *fmt,   //!<  format string for print 
    int    arg1, //!<  first of six required args for fmt
    int    arg2,
    int    arg3,
    int    arg4,
    int    arg5,
    int    arg6
)
{
    s8 res = ERR_NONE;
    
#if _EMB_PRINT_DEBUG_    
    res = Emb_Log_Msg(fmt,arg1,arg2,arg3,arg4,arg5,arg6);
#endif
    return res;
}*/

s8 Bsp_printf(char *fmt,...)
{
    s8 res=ERR_NONE;
    va_list ap;
    char str0[LOG_BUFFER_SIZE]={0};
    char str1[(LOG_BUFFER_SIZE>>1)]={0};
    char str2[80]={0};

    va_start(ap,fmt);         //!< �Ѿ�������δ֪�Ĳ�����ʼ����ap������

    vsprintf(&str1[0],fmt,ap);

    sprintf(&str2[0],"\r\nCy:%d,sysCnt:%d ",MySysTick,EwayEmbSys.sysTimCnt);

    sprintf(str0,"%.*s%.*s\r\n",strlen(str2),str2,strlen(str1),str1);
            
    va_end(ap);

    res = PrintfUart1DmaTransferConfig((u8*)(&str0[0]),strlen((&str0[0])));
    
    return res;
    
}


/* --------------------------------------------------------------------------*/
/**
* @name SysLogWrite
* @brief 
* @details дLog��Ϣ�����У�д�����Ϣ�������಻�ܳ���80���ַ���
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 SysLogWrite(LogPrioity eLogPri,const char * strLog,...)
{
    int res = ERR_NONE;
    va_list ap;
    char str0[LOG_BUFFER_SIZE]={0};
    char str1[(LOG_BUFFER_SIZE>>1)]={0};
    char str2[50];

    if((EwayEmbFileSys.tfEn&0x0F) != 0)
    {
        va_start(ap,strLog);                  //!< get the first address of fmt parameters

        res = vsprintf(str1,strLog,ap);

        if(res > 0)
        {
            //!< ÿ����Ϣ�����Ϣͷ sysTick&sysCycle,ϵͳms���м�¼��ϵͳ30msͨ�����ڼ�¼
            sprintf(str2,"sysTic:%d,sysCy:%d ",SysGetCurrentRunTime(),EwayEmbSys.sysTimCnt);
            
            sprintf(&str0[1],"%.*s%.*s\r\n",strlen(str2),str2,strlen(str1),str1);

            str0[0] = 0x20;

            res = strlen(str0);

            if(res < CMD_EMBWRLOG_INFO_LEN)
            {
                //send:
                str0[0] = res;
                
                if(pdTRUE != xQueueSend(EmbToLogWrQueHdl,str0,0))
                {
                    res = ERR_PROCESS_FAILED;
                }
                else
                {
                    res = ERR_NONE;
                }        
            }
            else
            {
                res = ERR_DATA_OVERFLOW;
            }
        }
        else
        {
            res = ERR_DATA_OVERFLOW;
        }
        
        va_end(ap);
    
        return res;
    }

    return ERR_PROCESS_FAILED;    
}


/* --------------------------------------------------------------------------*/
/**
* @name IWDGInit
* @brief �������Ź���ʼ��,500ms����������ιһ�ι���ReloaderΪ1000ms
* @details 
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
void BspWatchDog_Init(void)
{
#if Emb_System_TaskMonitor_Enable

#if EMB_SYS_DEBUG
    //debug mode,the IWDG counter clock is stopped when the core is halted
    DBGMCU_APB1PeriphConfig(DBGMCU_APB1_FZ_DBG_IWDG_STOP,ENABLE);    
#endif

	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);    //!< ����Ĵ���д����        

	IWDG_SetPrescaler(IWDG_Prescaler_16);            //!< д���ʼ����Ƶֵ

	IWDG_SetReload(IWDG_RELOAD_VALUE);               // д���Զ�װ��ֵ

	IWDG_WriteAccessCmd(0xAAAA);              // �����Ĵ������������ؼ�����

	IWDG_WriteAccessCmd(0xCCCC);              // ʹ�ܶ������Ź�
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: IWDG_Feed
*	����˵��: �������Ź�ι����500ms����������ιһ�ι���
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/

void IWDG_Feed(void)
{		
    IWDG_WriteAccessCmd(0xAAAA);	                 
}

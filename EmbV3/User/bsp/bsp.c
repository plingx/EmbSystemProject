/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp.c
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
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
        由于ST固件库的启动文件已经执行了CPU系统时钟的初始化，所以不必再次重复配置系统时钟。
        启动文件配置了CPU主时钟频率、内部Flash访问速度和可选的外部SRAM FSMC初始化。

        系统时钟缺省配置为168MHz，如果需要更改，可以修改 system_stm32f4xx.c 文件
    */
    //bsp_InitUart();     /* 初始化串口 */
    //bsp_InitTimer();    /* 初始化系统滴答定时器 */

    /* 针对不同的应用程序，添加需要的底层驱动模块初始化函数 */
  //CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN初始化环回模式,波特率500Kbps
        //LEDInit();
     //UPDOWNInit();
     //IOInit();

    SysConfigNVICPriorityGroup();
    BspGpios_init();
    BspUarts_init();        ///<配置的是COM7<-->UART7和COM8 <--> UART8 
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

    va_start(ap,fmt);         //!< 已经将各个未知的参数初始化到ap表中了

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
* @details 写Log信息到队列，写入的信息长度至多不能超过80个字符。
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
            //!< 每条消息添加消息头 sysTick&sysCycle,系统ms运行记录，系统30ms通信周期记录
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
* @brief 独立看门狗初始化,500ms在主程序中喂一次狗。Reloader为1000ms
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

	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);    //!< 解除寄存器写保护        

	IWDG_SetPrescaler(IWDG_Prescaler_16);            //!< 写入初始化分频值

	IWDG_SetReload(IWDG_RELOAD_VALUE);               // 写入自动装载值

	IWDG_WriteAccessCmd(0xAAAA);              // 开启寄存器保护并重载计数器

	IWDG_WriteAccessCmd(0xCCCC);              // 使能独立看门狗
#endif
}

/*
*********************************************************************************************************
*	函 数 名: IWDG_Feed
*	功能说明: 独立看门狗喂狗。500ms在主程序中喂一次狗。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/

void IWDG_Feed(void)
{		
    IWDG_WriteAccessCmd(0xAAAA);	                 
}

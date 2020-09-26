#include "includes.h"
#include <stdarg.h>



extern u8 PrintfUart1DmaTransferConfig(u8 *pSndBuf,u16 len);

extern __IO uint32_t MySysTick;
extern EwayEmbFileSysModule EwayEmbFileSys;
extern EwayEmbSysModule EwayEmbSys;


/*
*********************************************************************************************************
*    �� �� ��: SysGetCurrentRunTime
*    ����˵��: ��ȡϵͳ����������
*    ��    �Σ�none
*    �� �� ֵ: ϵͳ��������
*********************************************************************************************************
*/
u32 SysGetCurrentRunTime(void)
{
    return MySysTick;
    //return 0;
}

/*
*********************************************************************************************************
*    �� �� ��: Sd_Log_Msg
*    ����˵��: ��SD����log�ļ�д��log��Ϣ
*    ��    �Σ�fmt:��д����ַ���ָ�뼰������ע���ַ����ĳ��Ȳ�Ҫ������ֵ128��.
*    �� �� ֵ: �ɹ���¼���ַ���
*********************************************************************************************************
*/
/*
u8 Sd_Log_Msg( char *fmt,...)
{    
    int res;
    va_list ap;
    char str0[LOG_BUFFER_SIZE]={0};
    char str1[(LOG_BUFFER_SIZE>>1)]={0};
    char str2[50];

    if(EwayEmbFileSys.tfEn != 0)
    {
        va_start(ap,fmt);                  //!< get the first address of fmt parameters

        vsprintf(str1,fmt,ap);

        sprintf(str2,"Cycle:%d ",SysGetCurrentRunTime());

        sprintf(str0,"%.*s%.*s",strlen(str2),str2,strlen(str1),str1);
    
        res = f_puts(str0,&(EwayEmbFileSys.fSrc.efile));

        f_sync(&(EwayEmbFileSys.fSrc.efile));                   //!< ��ʱ����sd���е�����
        
        va_end(ap);
    
        return res;
    }

    return 1;
    
}
*/


s8 Emb_Log_Msg( char *fmt,...)
{    
    s8 res=ERR_NONE;
    va_list ap;
    char str0[LOG_BUFFER_SIZE]={0};
    char str1[(LOG_BUFFER_SIZE>>1)]={0};
    char str2[40]={0};

    va_start(ap,fmt);         //!< �Ѿ�������δ֪�Ĳ�����ʼ����ap������

    vsprintf(&str1[0],fmt,ap);

    sprintf(&str2[0],"\r\nCy:%d,sysCnt:%d ",MySysTick,EwayEmbSys.sysTimCnt);

    sprintf(str0,"%.*s%.*s\r\n",strlen(str2),str2,strlen(str1),str1);
            
    va_end(ap);

    res = PrintfUart1DmaTransferConfig((u8*)(&str0[0]),strlen((&str0[0])));
    
    return res;
    
}


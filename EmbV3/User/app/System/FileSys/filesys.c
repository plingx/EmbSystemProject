#include "includes.h"
#include "ff.h"
#include "filesys.h"


extern EwayEmbSysModule EwayEmbSys;
extern EwayEmbSysDebugModule* pDebug;



//FATFS *fs,fatfs;
//FIL        fdir;    //!< �洢Ŀ¼�ļ���FIL
FIL        fSRC;    //!< log�ļ���FIL

//TCHAR dirFlN[20]="dirfile.txt";    //!< �洢Ŀ¼�ļ���
//TCHAR logFlN[20]={0};              //!< log�ļ���
//u8 sdEn=0;                         //!< ��ʼ��ʱ��log�ļ��򿪳ɹ���������Ϊʹ��.

EwayEmbFileSysModule EwayEmbFileSys={0,0,0x00,{0x00},{{"dirfile.txt"},{0}},{{0},{0}}};




/* --------------------------------------------------------------------------*/
/**
* @name SD_DirFileProcess
* @brief 
* @details Ϊ�����log�ļ��ṩ�ļ���
*
* @param[in] log�ļ���ŷ��ص�ַ
*
* @returns Ѱ�Ҵ洢Ŀ¼��¼�ļ��Ƿ�ɹ���0-successful,1-failed
* 
* @author Ling
*/
/* --------------------------------------------------------------------------*/
s8 SD_DirFileProcess(EmbFileModule* pFileDir,u32 * pflNum)
{
    FRESULT rt;
	u32 cnt_w=0;
	u32 cnt_r=0;
	u32 flNum;
	//s8 res;
	
	rt = f_open(&(pFileDir->efile),pFileDir->fName,FA_OPEN_EXISTING|FA_READ|FA_WRITE);	   //!< �Ƿ���ڴ洢�ļ�

	if(rt!=FR_OK)                  //!< �������ڣ������½����ļ�����д�����1�����������Ϊ1��log�ļ�
	{
	    rt = f_open(&(pFileDir->efile),pFileDir->fName,FA_CREATE_ALWAYS|FA_READ|FA_WRITE);  //!< �����µĴ洢Ŀ¼�ļ�

		if(rt != FR_OK)
		{
			return 	ERR_PROCESS_FAILED;					//!< �������洢Ŀ¼�ļ�ʧ�ܣ��򷵻ش���
		}
		
		flNum = 1;										//!< �½��洢Ŀ¼�ļ����˴ξʹ�log1��ʼ
		
		rt = f_write(&(pFileDir->efile),&flNum,4,&cnt_w);

		if(rt==FR_OK)
		{
		    *pflNum = flNum;

			f_close(&(pFileDir->efile));
			
            return ERR_NONE;
		}
		else
		{
		    f_close(&(pFileDir->efile));
			
            return ERR_PROCESS_FAILED;
		}
	}
	else  //!< �����ڣ���������ݣ�������1д��
	{		
        rt = f_read(&(pFileDir->efile),&flNum,4,&cnt_r);

		if(rt==FR_OK)                                    //!< ����log ok
		{			
	        f_lseek(&(pFileDir->efile),0);	                         //!< ����ָ��ָ���ʼ
	        
			flNum++;

			rt = f_write(&(pFileDir->efile),&flNum,4,&cnt_w);         //!< ����log ok,log num +1 ����д�ص��ļ���

			if(rt==FR_OK)
			{
				*pflNum = flNum;
	
			}
			else                                         //!< ��д��ʧ�ܣ��򻹼���ʹ��ԭ��log
			{
                *pflNum = flNum--;
			}
			
			f_close(&(pFileDir->efile));
			
            return ERR_NONE;
		}
		else                                             //!< ������log numʧ��
		{
			flNum = 1;
			
		    rt = f_write(&(pFileDir->efile),&flNum,4,&cnt_w);

			if(rt==FR_OK)
		    {
		        *pflNum = flNum;

			    f_close(&(pFileDir->efile));
			
                return ERR_NONE;			
		    }
		    else
		    {
		        f_close(&(pFileDir->efile));
			
                return ERR_PROCESS_FAILED;
		    }
		}
	}

}

/* --------------------------------------------------------------------------*/
/**
* @name CreateLogFile
* @brief 
* @details �����Ƿ��ȡ����һ����λ�����ݵ�ʱ�����ȷ��Log�ļ������������ļ��ɹ�����
*			fopen()�ļ��󲢲�����fclose()����֤����ѯ����������ط���ֱ��ʹ��
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 CreateLogFile(EwayEmbFileSysModule* pFileModule,u8 cMode)
{
	s8 res;
	FRESULT rt;
	u32 flNum;
	TCHAR logFlN[20]={0};              		//!< log�ļ���

	//!< ��ȡ�ļ���
	if(cMode == 0)					//!< Create by Serial Number
	{
		res = SD_DirFileProcess(&(pFileModule->fDir),&flNum);         //!< Read the dirfile,and return the current logfile number

		if(res!=0)                               //!< ��ȡdirfileʧ�ܣ�δ��õ�ǰlog�ļ���ţ�Ĭ��д�����ΪFFFFFFFF���ļ���
		{
        	flNum = 0xFFFFFFFF;
		}	
	
		sprintf(logFlN,"%d.log",flNum);         //!< �����ת�����ļ��ַ���	
	}
	else
	{
		sprintf(logFlN,"%02d-%02d-%02d_%02d_%02d.log",(u8)((EwayEmbSys.tStmp.Init>>28)+18),(u8)((EwayEmbSys.tStmp.Init&0x0F000000)>>24),((u8)((EwayEmbSys.tStmp.Init&0x00FF0000)>>16)),(u8)((EwayEmbSys.tStmp.Init&0x0000FF00)>>8),(u8)EwayEmbSys.tStmp.Init);
	}

	//!< ����Log�ļ�
	rt = f_open(&(pFileModule->fSrc.efile),logFlN,FA_OPEN_ALWAYS|FA_WRITE|FA_READ);	   //!< �������ߴ��ļ�
		
	if(rt==FR_OK)							//!< �򿪻��߽���log�ļ��ɹ������ڴ�ӡlog�ļ�����.
	{
		memcpy(pFileModule->fSrc.fName,logFlN,LOG_FILE_NAME_LENGTH);		//!< �����ļ���
		
		f_lseek(&(pFileModule->fSrc.efile),f_size(&(pFileModule->fSrc.efile)));//!< ���ļ�ָ���Ƶ��ļ�ĩβ
		
		if(cMode == 0)
		{			
			pFileModule->tfEn = 0x01;				//!< ��ʹ��FatFs��дsd card���ñ�־,Created by Serial Number
		}
		else
		{
			pFileModule->tfEn = 0x11;				//!< ��ʹ��FatFs��дsd card���ñ�־,Created by Time Stamp
		}
		
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
		    res = Bsp_printf("CreateLogFile() open log file successful.");
        }

		return ERR_NONE;
	}
	else
	{
		memset(pFileModule->fSrc.fName,0,LOG_FILE_NAME_LENGTH);		//!< Clear �ļ�������
		
		pFileModule->tfEn = 0;
		
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
		    res = Bsp_printf("CreateLogFile() open log file failed.");
        }

        return ERR_PROCESS_FAILED;
	}	
}


/* --------------------------------------------------------------------------*/
/**
* @name CreateLogFile
* @brief 
* @details �����Ƿ��ȡ����һ����λ�����ݵ�ʱ�����ȷ��Log�ļ������������ļ��ɹ�����
*			fopen()�ļ��󲢲�����fclose()����֤����ѯ����������ط���ֱ��ʹ��
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysEmbLogFileSync(void)
{
	FRESULT rt;
				
	rt = f_sync(&(EwayEmbFileSys.fSrc.efile)); 				  //!< ��ʱ����sd���е�����

	if(rt!=FR_OK)
	{
		return ERR_PROCESS_FAILED;
	}
	else
	{
		return ERR_NONE;
	}
}


/* --------------------------------------------------------------------------*/
/**
* @name sysEmbLogFileChecking
* @brief 
* @details ���ڼ����λ��log�ļ��Ƿ��Ѿ���ʼ��(TF���Ƿ��д)����û�У���ִ�г�ʼ���Ĺ���
*			���Ѿ���ʼ������Ҫ��鵱ǰ�Ǹ�����Ž�����Log�ļ���or����ʱ��������Ǹ������
*			�����ģ�����Ҫ��鵱ǰϵͳ�Ƿ��Ѿ����ӹ����Ƿ��Ѿ���ȡ����ʼ��ʱ���.
* @param[in] pFileModule Log�ļ�����ģ��ָ�룬��ʼ��Log�ļ��Ľ������EwayEmbFileSys.tfEn��
*
* @returns ERR_NONE	����Log�ļ��ɹ�
*			Others	����Log�ļ�ʧ��
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysEmbLogFileChecking(EwayEmbFileSysModule *pFileModule)
{
	s8 res;
	FRESULT rt;

	if((pFileModule->tfEn&0x0F) != LOG_FILE_TF_WR_EN)		//!< ��δ��ʼ���ɹ���������дTF��
	{
		res = sysEmbLogFileInit(pFileModule);

		if(res != ERR_NONE)
		{
			//res = Bsp_printf("sysEmbLogFileChecking() LogFile Init failed.\r\n");
		}
	}
	else if((pFileModule->tfEn == 0x01)&&(EwayEmbSys.tStmp.Init != 0))		//!< �Ѿ���ʼ���ɹ�������Log�ļ���ʹ������ŵķ�ʽ�����ģ���Ҫ��Ϊʹ��ʱ������½���Log file
	{
		//!< ��tf���Ƿ����		
		rt = f_mount(&(pFileModule->FatFs),"",1);					  //!< ��������
	
		if(rt == FR_OK)
		{
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
			    res = Bsp_printf("sysEmbLogFileChecking() f_mount OK.\r\n");
            }
		}
		else
		{
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
			//res = Bsp_printf("sysEmbLogFileChecking() f_mount Failed.\r\n");
            }
			pFileModule->tfEn = 0;
	
			return ERR_NONE;
		}

		res = CreateLogFile(&EwayEmbFileSys,1);

		if(res != ERR_NONE)
		{
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
			    res = Bsp_printf("sysEmbLogFileChecking() Create LogFile by TimeStamp Failed.");
            }
			return ERR_PROCESS_FAILED;
		}
		else
		{
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
			    res = Bsp_printf("sysEmbLogFileChecking() Create LogFile by TimeStamp Successful.");
            }
			return ERR_NONE;
		}
	}
	
	return ERR_NONE;
}



/* --------------------------------------------------------------------------*/
/**
* @name sysEmbLogFileInit
* @brief 
* @details ��λ��log�ļ���ʼ��
*
* @param[in] pFileModule Log�ļ�����ģ��ָ�룬��ʼ��Log�ļ��Ľ������EwayEmbFileSys.tfEn��
*
* @returns ERR_NONE	����Log�ļ��ɹ�
*			Others	����Log�ļ�ʧ��
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysEmbLogFileInit(EwayEmbFileSysModule *pFileModule)
{
	s8 res;
	FRESULT rt;

	//!< ��tf���Ƿ����		
	rt = f_mount(&(pFileModule->FatFs),"",1);					  //!< ��������
	
	if(rt != FR_OK)
	{
		pFileModule->tfEn = 0;
		
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
		//res = Bsp_printf("f_mount Failed.\r\n");
        }       
		return ERR_PROCESS_FAILED;
	}

	//!< ȷ���ļ���
	if(EwayEmbSys.tStmp.Init== 0)		//��δ�������ӣ���ʱ�������������ŵķ���������log�ļ�
	{
		res = CreateLogFile(&EwayEmbFileSys,0);

		if(res != ERR_NONE)
		{
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
			    res = Bsp_printf("Create LogFile by SerialNumber Failed.");
            }
			return ERR_PROCESS_FAILED;
		}
		else
		{
			//res = Bsp_printf("Create LogFile by SerialNumber Successful.");

			return ERR_NONE;
		}
	}
	else
	{
		res = CreateLogFile(&EwayEmbFileSys,1);

		if(res != ERR_NONE)
		{
            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
			    res = Bsp_printf("Create LogFile by TimeStamp Failed.");
            }
			return ERR_PROCESS_FAILED;
		}
		else
		{
			//res = Bsp_printf("Create LogFile by TimeStamp Successful.fNm:%s.",EwayEmbFileSys.fSrc.fName);

			return ERR_NONE;
		}
	}

}



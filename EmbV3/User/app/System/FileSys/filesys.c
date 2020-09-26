#include "includes.h"
#include "ff.h"
#include "filesys.h"


extern EwayEmbSysModule EwayEmbSys;
extern EwayEmbSysDebugModule* pDebug;



//FATFS *fs,fatfs;
//FIL        fdir;    //!< 存储目录文件的FIL
FIL        fSRC;    //!< log文件的FIL

//TCHAR dirFlN[20]="dirfile.txt";    //!< 存储目录文件名
//TCHAR logFlN[20]={0};              //!< log文件名
//u8 sdEn=0;                         //!< 初始化时若log文件打开成功，则置其为使能.

EwayEmbFileSysModule EwayEmbFileSys={0,0,0x00,{0x00},{{"dirfile.txt"},{0}},{{0},{0}}};




/* --------------------------------------------------------------------------*/
/**
* @name SD_DirFileProcess
* @brief 
* @details 为后面的log文件提供文件名
*
* @param[in] log文件序号返回地址
*
* @returns 寻找存储目录记录文件是否成功，0-successful,1-failed
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
	
	rt = f_open(&(pFileDir->efile),pFileDir->fName,FA_OPEN_EXISTING|FA_READ|FA_WRITE);	   //!< 是否存在存储文件

	if(rt!=FR_OK)                  //!< 若不存在，则重新建立文件，并写入序号1，并建立序号为1的log文件
	{
	    rt = f_open(&(pFileDir->efile),pFileDir->fName,FA_CREATE_ALWAYS|FA_READ|FA_WRITE);  //!< 建立新的存储目录文件

		if(rt != FR_OK)
		{
			return 	ERR_PROCESS_FAILED;					//!< 若建立存储目录文件失败，则返回错误
		}
		
		flNum = 1;										//!< 新建存储目录文件，此次就从log1开始
		
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
	else  //!< 若存在，则读回数据，并自增1写回
	{		
        rt = f_read(&(pFileDir->efile),&flNum,4,&cnt_r);

		if(rt==FR_OK)                                    //!< 读回log ok
		{			
	        f_lseek(&(pFileDir->efile),0);	                         //!< 调整指针指到最开始
	        
			flNum++;

			rt = f_write(&(pFileDir->efile),&flNum,4,&cnt_w);         //!< 读回log ok,log num +1 ，并写回到文件中

			if(rt==FR_OK)
			{
				*pflNum = flNum;
	
			}
			else                                         //!< 若写回失败，则还继续使用原有log
			{
                *pflNum = flNum--;
			}
			
			f_close(&(pFileDir->efile));
			
            return ERR_NONE;
		}
		else                                             //!< 若读回log num失败
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
* @details 根据是否获取到第一包上位机数据的时间戳来确定Log文件名，若建立文件成功，则
*			fopen()文件后并不调用fclose()，保证在轮询任务或其他地方可直接使用
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
	TCHAR logFlN[20]={0};              		//!< log文件名

	//!< 获取文件名
	if(cMode == 0)					//!< Create by Serial Number
	{
		res = SD_DirFileProcess(&(pFileModule->fDir),&flNum);         //!< Read the dirfile,and return the current logfile number

		if(res!=0)                               //!< 读取dirfile失败，未获得当前log文件序号，默认写入序号为FFFFFFFF的文件中
		{
        	flNum = 0xFFFFFFFF;
		}	
	
		sprintf(logFlN,"%d.log",flNum);         //!< 将序号转换成文件字符串	
	}
	else
	{
		sprintf(logFlN,"%02d-%02d-%02d_%02d_%02d.log",(u8)((EwayEmbSys.tStmp.Init>>28)+18),(u8)((EwayEmbSys.tStmp.Init&0x0F000000)>>24),((u8)((EwayEmbSys.tStmp.Init&0x00FF0000)>>16)),(u8)((EwayEmbSys.tStmp.Init&0x0000FF00)>>8),(u8)EwayEmbSys.tStmp.Init);
	}

	//!< 建立Log文件
	rt = f_open(&(pFileModule->fSrc.efile),logFlN,FA_OPEN_ALWAYS|FA_WRITE|FA_READ);	   //!< 建立或者打开文件
		
	if(rt==FR_OK)							//!< 打开或者建立log文件成功，串口打印log文件名称.
	{
		memcpy(pFileModule->fSrc.fName,logFlN,LOG_FILE_NAME_LENGTH);		//!< 保存文件名
		
		f_lseek(&(pFileModule->fSrc.efile),f_size(&(pFileModule->fSrc.efile)));//!< 将文件指针移到文件末尾
		
		if(cMode == 0)
		{			
			pFileModule->tfEn = 0x01;				//!< 置使用FatFs读写sd card可用标志,Created by Serial Number
		}
		else
		{
			pFileModule->tfEn = 0x11;				//!< 置使用FatFs读写sd card可用标志,Created by Time Stamp
		}
		
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
		    res = Bsp_printf("CreateLogFile() open log file successful.");
        }

		return ERR_NONE;
	}
	else
	{
		memset(pFileModule->fSrc.fName,0,LOG_FILE_NAME_LENGTH);		//!< Clear 文件名缓存
		
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
* @details 根据是否获取到第一包上位机数据的时间戳来确定Log文件名，若建立文件成功，则
*			fopen()文件后并不调用fclose()，保证在轮询任务或其他地方可直接使用
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
				
	rt = f_sync(&(EwayEmbFileSys.fSrc.efile)); 				  //!< 及时更新sd卡中的内容

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
* @details 周期检查下位机log文件是否已经初始化(TF卡是否可写)，若没有，则执行初始化的过程
*			若已经初始化，还要检查当前是根据序号建立的Log文件名or根据时间戳，若是根据序号
*			建立的，则还需要检查当前系统是否已经连接过，是否已经获取到初始的时间戳.
* @param[in] pFileModule Log文件控制模块指针，初始化Log文件的结果存入EwayEmbFileSys.tfEn中
*
* @returns ERR_NONE	建立Log文件成功
*			Others	建立Log文件失败
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysEmbLogFileChecking(EwayEmbFileSysModule *pFileModule)
{
	s8 res;
	FRESULT rt;

	if((pFileModule->tfEn&0x0F) != LOG_FILE_TF_WR_EN)		//!< 还未初始化成功，还不能写TF卡
	{
		res = sysEmbLogFileInit(pFileModule);

		if(res != ERR_NONE)
		{
			//res = Bsp_printf("sysEmbLogFileChecking() LogFile Init failed.\r\n");
		}
	}
	else if((pFileModule->tfEn == 0x01)&&(EwayEmbSys.tStmp.Init != 0))		//!< 已经初始化成功，但是Log文件是使用排序号的方式建立的，需要改为使用时间戳重新建立Log file
	{
		//!< 看tf卡是否存在		
		rt = f_mount(&(pFileModule->FatFs),"",1);					  //!< 立即挂载
	
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
* @details 下位机log文件初始化
*
* @param[in] pFileModule Log文件控制模块指针，初始化Log文件的结果存入EwayEmbFileSys.tfEn中
*
* @returns ERR_NONE	建立Log文件成功
*			Others	建立Log文件失败
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 sysEmbLogFileInit(EwayEmbFileSysModule *pFileModule)
{
	s8 res;
	FRESULT rt;

	//!< 看tf卡是否存在		
	rt = f_mount(&(pFileModule->FatFs),"",1);					  //!< 立即挂载
	
	if(rt != FR_OK)
	{
		pFileModule->tfEn = 0;
		
        if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
        {
		//res = Bsp_printf("f_mount Failed.\r\n");
        }       
		return ERR_PROCESS_FAILED;
	}

	//!< 确定文件名
	if(EwayEmbSys.tStmp.Init== 0)		//还未网络连接，无时间戳，则按照排序号的方案来建立log文件
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



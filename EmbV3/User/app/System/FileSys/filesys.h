#ifndef _FILESYS_H_
#define _FILESYS_H_


extern u8 sdEn;



#define LOG_FILE_NAME_LENGTH		20
#define LOG_FILE_TF_WR_EN			0x01
#define LOG_FILE_CREATE_BY_TSTAMP	0x10



typedef struct{
	TCHAR fName[LOG_FILE_NAME_LENGTH];
	FIL   efile;
}EmbFileModule;

typedef struct{	
	u32 fTimCnt;
	u8 errCnt;				//!< 错误状态计数器
	u8 tfEn;				//!< bit0-3 0:不能读写TF卡，1:可读写TF卡		当TF卡可读写时，bit4-7有意义 0:Log文件名使用序号排序的方式，1:Log文件名使用的是接收第一条上位机指令的时间戳来命名的方式
	FATFS FatFs;
	EmbFileModule fDir;		//!< 存储目录文件的FIL
	EmbFileModule fSrc;		//!< log文件的FIL
}EwayEmbFileSysModule;

s8 sysEmbLogFileSync(void);
s8 sysEmbLogFileInit(EwayEmbFileSysModule *pFM);
s8 sysEmbLogFileChecking(EwayEmbFileSysModule *pFileModule);
s8 sysEmbLogFileNameInit(void);


#endif



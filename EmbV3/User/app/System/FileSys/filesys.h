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
	u8 errCnt;				//!< ����״̬������
	u8 tfEn;				//!< bit0-3 0:���ܶ�дTF����1:�ɶ�дTF��		��TF���ɶ�дʱ��bit4-7������ 0:Log�ļ���ʹ���������ķ�ʽ��1:Log�ļ���ʹ�õ��ǽ��յ�һ����λ��ָ���ʱ����������ķ�ʽ
	FATFS FatFs;
	EmbFileModule fDir;		//!< �洢Ŀ¼�ļ���FIL
	EmbFileModule fSrc;		//!< log�ļ���FIL
}EwayEmbFileSysModule;

s8 sysEmbLogFileSync(void);
s8 sysEmbLogFileInit(EwayEmbFileSysModule *pFM);
s8 sysEmbLogFileChecking(EwayEmbFileSysModule *pFileModule);
s8 sysEmbLogFileNameInit(void);


#endif



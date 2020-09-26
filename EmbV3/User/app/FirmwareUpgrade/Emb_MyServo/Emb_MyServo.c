#include "YmodemProtocol.h"
#include "Emb_MyServo.h"


/******************************************************************************************************
Function:		    ActuaticCtrl_FirmwareLoad
Description:	    Use Ymodem protocol to download embedded Firmware file.

					The steps are below:
					1) Open the Serial Port.
					2) Two private connections for our company's design. Maybe will be changed later.
						a)PC --> Motor  many 'C' characters.
						b)Motor --> PC  many '************' characters and so on.
						c)PC --> Motor  one '1' character.
						d)Motor --> PC  return some characters and some 'C' characters. The 'C' character
						  is the beginning of the Ymodem protocol.
					3) Get the image file's name and send the first SOH package.
					4) Open the image file,calculate the size of the image file and copy it to malloc buf.
					5) Send the normal SOH packages.
					6) Send the last SOH package.
					7) Send the Transmit Finish bytes.
					8) Send the all-0 Package.
					9) free malloc buffer and close the iamge file. The image has been downloaded successfully.


Input Paramters:
	@ nBitRate      The Baud rate of the serial port.
	@ nAddr     	Motor's address.
	@ strFileName   The name of the image file.
	@ strFilePath   The path of the image file.
Output Parameters:

Return:
	@ ERR_NONE      Successfully
	@ Other values  Unsuccessfully
Author:				Yi Cheng
Others:
*******************************************************************************************************/
extern eint32  ActuaticCtrl_FirmwareLoad(eint32 nBitRate, eint32 nAddr,const echar *strFileName,const echar *strFilePath,FnYmodemCallback fnCallback)
{
	if(NULL == strFileName || NULL == strFilePath)
	{
		EwayG_WriteLog(LogPriDebug, _S("%s %s %d"), __EFILE__, __FUNCTION__, __LINE__);
		return Pointer_Null_Error;
	}

	eint nRet = 0;
	EwayG_WriteLog(LogPriDebug, _S("The download type is Normal mode."));
	//ActuaticCtrl_Init(7);
	//1) Open the Serial Port.
	nRet = ActuaticCtrl_CheckCom(nBitRate, nAddr);
	if (ERR_NONE != nRet)
	{
		EwayG_WriteLog(LogPriDebug, _S("%s %s %d"), __EFILE__, __FUNCTION__, __LINE__);
		return (eint32)nRet;
	}

	//1) reboot the slavery motor.  Current BaudRate is 256000.
	nRet = g_iMotorCom.Ymodem_MotorRebootPack(nAddr);
	if (ERR_NONE != nRet)
	{
		ActuaticCtrl_CloseCom();
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return (eint32)nRet;
	}

	nRet = ActuaticCtrl_FirmwareLoad_Normal(nAddr,strFileName,strFilePath,fnCallback);

	if(ERR_NONE != nRet)
	{
		ActuaticCtrl_CloseCom();
		EwayG_WriteLog(LogPriDebug, _S("%s %s %d"), __EFILE__, __FUNCTION__, __LINE__);
		return Ymodem_Download_Error;
	}

	return ERR_NONE;
}

extern eint32  ActuaticCtrl_FirmwareLoad_InBoot(const echar *strFileName,const echar *strFilePath,FnYmodemCallback fnCallback)
{
	EwayG_WriteLog(LogPriDebug, _S("-----InBoot------"));
	if(NULL == strFileName || NULL == strFilePath)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! %s %s %d"), __EFILE__, __FUNCTION__, __LINE__);
		return Pointer_Null_Error;
	}

	eint nAddr = 0xfe;
	eint nRet = 0;
	//After reboot, the current BaudRate is changed to 115200.
	nRet = ActuaticCtrl_CheckCom(115200,(eint32)nAddr);
	if(ERR_NONE != nRet)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return (eint32)nRet;
	}

	ESleep(100);

	//Wait the slavery motor bring up. Check the packages and the 'C' character.
	nRet = g_iMotorCom.Ymodem_MotorBringUpPack(nAddr,CtrlCode_Restore_Res,CtrlCode_Restore_MasterRes);
	if(ERR_NONE != nRet)
	{
		ActuaticCtrl_CloseCom();
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return (eint32)nRet;
	}

	fnCallback(Restore_Step1,Restore_UselessPackId);

	nRet=ActuaticCtrl_YmodemLoad(strFileName,strFilePath,fnCallback);
	if(ERR_NONE != nRet)
	{
		ActuaticCtrl_CloseCom();
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return (eint32)nRet;
	}

	return ERR_NONE;
}

eint32  ActuaticCtrl_FirmwareLoad_Normal(eint32 nAddr,const echar *strFileName,const echar *strFilePath,FnYmodemCallback fnCallback)
{
	if(NULL == strFileName || NULL == strFilePath)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! %s %s %d"), __EFILE__, __FUNCTION__, __LINE__);
		return Pointer_Null_Error;
	}

	if(nAddr<0 || nAddr> 253)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! %s %s %d"), __EFILE__, __FUNCTION__, __LINE__);
		return Wrong_Param_ID;
	}

	eint nRet = 0;
	//After reboot, the current BaudRate is changed to 115200.
	nRet = ActuaticCtrl_CheckCom(115200,nAddr);
	if(ERR_NONE != nRet)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return (eint32)nRet;
	}

	ESleep(100);

	//Wait the slavery motor bring up. Check the packages and the 'C' character.
	nRet = g_iMotorCom.Ymodem_MotorBringUpPack(nAddr,CtrlCode_RebootRes,CtrlCode_MasterRes);
	if(ERR_NONE != nRet)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return (eint32)nRet;
	}

	nRet=ActuaticCtrl_YmodemLoad(strFileName,strFilePath,fnCallback);
	if(ERR_NONE != nRet)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return (eint32)nRet;
	}

	return ERR_NONE;
}


eint32  ActuaticCtrl_YmodemLoad(const echar *strFileName,const echar *strFilePath,FnYmodemCallback fnCallback)
{
    eint nRet = 0;

	if(NULL == strFileName || NULL == strFilePath)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! %s %s %d"), __EFILE__, __FUNCTION__, __LINE__);
		return Pointer_Null_Error;
	}
	//Then begin the Ymodem protocol really.

	//4) Open the image file,calculate the size of the image file and copy it to malloc buf.

	FILE *fp=efopen(strFilePath,_S("rb+"));   //Open the .bin file.
	if(!fp)
	{
		cout<<"can not open the file!"<<endl;
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return OpenFile_Error;
	}

	fseek(fp,0L,SEEK_END);
	euint32 nFileSize = ftell(fp);	  //Calculate the size of the .bin file.

	euint8 *pFileBuf = (euint8 *)malloc(nFileSize);

	if(NULL == pFileBuf)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return Pointer_Null_Error;
	}

	memset(pFileBuf,0,nFileSize);

	fseek(fp,0L,SEEK_SET);
	fread (pFileBuf,nFileSize,1, fp) ;	  //Copy the file to the memory.

	euint nPackNum = nFileSize/YMODEM_SOH_PACKAGE_PAYLOAD_LEN;
	euint nLastPackByte = nFileSize%YMODEM_SOH_PACKAGE_PAYLOAD_LEN;

	EwayG_WriteLog(LogPriDebug,_S("nFileSize=%d nPackNum = %d nLastPackByte = %d" )
		,nFileSize,nPackNum,nLastPackByte);

	ActuaticCtrl_RunThreadFirmwareDownload(fnCallback);

	//3) Get the image file's name and send the first SOH package.
	eint nStrNameLen = sizeof(echar)*estrlen(strFileName);

	eint8 nNameUtf8[256] = {0};
	euint8 nFileNameUtf8[256] = {0};
	eint nLenUtf8 = 0;

	nLenUtf8 = ELocalToUtf8(strFileName, nStrNameLen, nNameUtf8, 256);

	for (eint i = 0; i < nLenUtf8; i++)
	{
		nFileNameUtf8[i] = nNameUtf8[i];
	}

	nRet = g_iMotorCom.Ymodem_FillSOHFirstPackage(nFileNameUtf8, nLenUtf8);

	if(ERR_NONE != nRet)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		ActuaticCtrl_StopThreadFirmwareDownload();
		return Ymodem_Send_First_Package_Error;
	}

	euint8 nCheckVal[1]={0};

	//ESleep(1000);
	//5) Send the normal SOH packages.
	for(euint i = 0;i<nPackNum ;i++)
	{
		nRet = 0;
		nRet=g_iMotorCom.Ymodem_FillSOHPackage(pFileBuf+i*YMODEM_SOH_PACKAGE_PAYLOAD_LEN,i+1);

		if(ERR_NONE!=nRet)
		{
			 //cout<<"ERROR!  g_iMotorCom.Ymodem_FillSOHPackage. PackageNum = "<<dec<<i<<endl;
			EwayG_WriteLog(LogPriDebug, _S("ERROR! PackageNum=%d nRet=%d %s %s %d"), i, nRet, __EFILE__, __FUNCTION__, __LINE__);
			 ActuaticCtrl_StopThreadFirmwareDownload();
			 return Ymodem_Send_Normal_Package_Error;
		}
		EwayG_WriteLog(LogPriDebug,_S("Good! i==================%d"),i);

		m_nCurrentPackId = (eint32)i; //C# UI need the eint32.

		ESleep(10);
	}
	//6) Send the last SOH package.
	nRet=g_iMotorCom.Ymodem_FillSOHLastPackage(pFileBuf+nPackNum*YMODEM_SOH_PACKAGE_PAYLOAD_LEN,nLastPackByte,nPackNum+1);

	if(ERR_NONE!=nRet)
	{
		cout<<"ERROR!  g_iMotorCom.Ymodem_FillSOHLastPackage."<<endl;
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		ActuaticCtrl_StopThreadFirmwareDownload();
		return Ymodem_Send_Last_Package_Error;
	}

	ActuaticCtrl_StopThreadFirmwareDownload();

	//7) Send the Transmit Finish bytes.
	nRet = g_iMotorCom.Ymodem_TransmitFinish();
	if(ERR_NONE!=nRet)
	{
		cout<<"ERROR!  g_iMotorCom.Ymodem_TransmitFinish."<<endl;
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return Ymodem_Send_TransmitFinisg_Package_Error;
	}
	ESleep(1000);

	nCheckVal[0] =0;
	nRet = g_iMotorCom.Ymodem_SerialRead(nCheckVal,1);
	if(ERR_NONE!=nRet)
	{
		 cout<<"ERROR!	g_iMotorCom.Ymodem_SerialRead"<<endl;
		 EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		 return Ymodem_Serial_Read_Error;
	}
	if('C' != nCheckVal[0])
	{
		cout<<"ERROR!  Can not rec the C"<<endl;
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return Ymodem_Check_C_Error;
	}

	//8) Send the all-0 Package.
	nRet = g_iMotorCom.Ymodem_FillSOHAllZeroPackage();
	if(ERR_NONE!=nRet)
	{
		cout<<"ERROR! g_iMotorCom.Ymodem_FillSOHAllZeroPackage."<<endl;
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return Ymodem_Send_AllZero_Package_Error;
	}

	//9) free malloc buffer and close the iamge file. The image has been downloaded successfully.
	free(pFileBuf);
	pFileBuf = NULL;
	fclose(fp);

	g_iMotorCom.SetThreadStatus(false);

	EwayG_WriteLog(LogPriDebug, _S("Iamge Load Finish !!!Image has been loadded successfully!!!"));

	//cout<<"Iamge Load Finish !!! Image has been loadded successfully!!!"<<endl;

	return ERR_NONE;
}


eint ActuaticCtrl_RunThreadFirmwareDownload(FnYmodemCallback fnCallback)
{
    m_bFirmwareDownloadStopThread = false;
	m_piFirmwareDownloadMutex = CEBaseFactory::GetMutexInstance();
	m_piFirmwareDownloadMutex->CreateMutex();

    m_pFirmwareDownloadMainThread = CEBaseFactory::GetThreadInstance();
    m_pFirmwareDownloadMainThread->CreateThread(ActuaticCtrl_FirmwareDownloadThreadRun,0);
	m_nCurrentPackId = 0;
	g_fnYmodemCallback = fnCallback;
	EwayG_WriteLog(LogPriDebug,_S("hi! Thread Run----"));

	return ERR_NONE;
}

eint ActuaticCtrl_StopThreadFirmwareDownload()
{
	m_piFirmwareDownloadMutex->Destroy();
	delete m_piFirmwareDownloadMutex;
	m_piFirmwareDownloadMutex = NULL;

    m_bFirmwareDownloadStopThread = true;
    m_pFirmwareDownloadMainThread->WaitForThreadQuit();
	m_nCurrentPackId = 0;
	g_fnYmodemCallback = NULL;
	EwayG_WriteLog(LogPriDebug,_S("hi! Thread Stop----"));
    return ERR_NONE;
}

void* ActuaticCtrl_FirmwareDownloadThreadRun(void* pArgc)
{
	while(!m_bFirmwareDownloadStopThread)
	{
		EwayG_WriteLog(LogPriDebug,_S("---FirmwareDownloadThreadRun--- %d"),m_nCurrentPackId);
		g_fnYmodemCallback(Restore_Step2,m_nCurrentPackId);
		ESleep(100);
	}

	return NULL;
}

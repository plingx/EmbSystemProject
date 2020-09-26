#include "YmodemProtocol.h"


/******************************************************************************************************
Function:		    YModem_ColculateCrc
Description:	    Colculate the CRC values.
					There are many different CRC algorithms.

					CRC-16
					CRC-16 (Modbus)
					CRC-16 (Sick)
					CRC-CCITT (XModem)
					CRC-CCITT (0xFFFF)
					CRC-CCITT (0x1D0F)
					CRC-CCITT (Kermit)
					CRC-DNP
					CRC-32


					This function implement  CRC-CCITT (XModem)

Input Paramters:
	@ *pbPack		The begin address of the buffer.
	@ nPackLen      The length of the buffer.

Output Parameters:

Return:             The CRC-16 values.

Author:				Yi Cheng
Others:
*******************************************************************************************************/

euint16 CMotorCom::YModem_ColculateCrc(euint8 *pData, euint16 sLen)
{
   euint16 CRC = 0;    //
   euint16 i = 0;

   while (sLen--)  //lenÊÇËùÒªŒÆËãµÄ³€¶�?
   {
       CRC = CRC^(eint)(*pData++) << 8; //
       for (i=8; i!=0; i--)
       {
           if (CRC & 0x8000)
               CRC = CRC << 1 ^ 0x1021;
           else
               CRC = CRC << 1;
       }
   }
   return CRC;
}


//FnYmodemProc  g_fnYmodemCallback = NULL;


/******************************************************************************************************
Function:		    Ymodem_SendCmd
Description:	    Ymodem Send Command interface.
Input Paramters:
	@ pbReqBuf      The begin address of the send buffer.
	@ nReqLen       The length of the send buffer.
	@ pbResBuf      The begin address of the response/Recieve buffer.
	@ nResLen       The length of the response/Recieve buffer.

Output Parameters:

Return:
	@ ERR_NONE      Successfully
	@ Other values  Unsuccessfully
Author:				Yi Cheng
Others:
*******************************************************************************************************/
eint CMotorCom::Ymodem_SendCmd(euint8* pbReqBuf,eint nReqLen,euint8* pbResBuf,eint nResLen)
{
	if(NULL == pbReqBuf || NULL == pbResBuf || nReqLen >510 || nResLen >512)
	{
		//add log
		return Wrong_Values;
	}

	eint nDataLen=0,nRet=0,nReadLen=0,nSendLen=0;
	euint8 bSendBuf[512] = { 0 };
	euint8 bRecvBuf[512] = { 0 };

	CAutoMutex iAutoMutex(m_piMutex);

	memcpy(bSendBuf, pbReqBuf, (euint16)nReqLen);

	/*
		YModem CRC:
		1. CRC begin from the 4th byte.
		2. Send the HCRC firstly,then send the LCRC.
	*/

	euint16 crcval = YModem_ColculateCrc(&bSendBuf[3],(euint16)nReqLen-3);
	bSendBuf[nReqLen]= (euint8)((crcval>>8)&0x00ff);
	bSendBuf[nReqLen+1]=(euint8)(crcval&0x00ff);

	nSendLen = nReqLen + 2;
	nDataLen = nSendLen;

/*
	for (eint n = 0; n<nDataLen; n++)
		ETRACE(_S("%02x "), bSendBuf[n]);
	ETRACE(_S("send \n"));
*/
	m_piSerialPort->ClearSendBuf();
	m_piSerialPort->ClearRecvBuf();

	while(nDataLen>0)	//send packet with crc
	{
		nRet = m_piSerialPort->Write(bSendBuf + nSendLen - nDataLen, nDataLen);
		if(nRet<=0)
			return WRITE_DATA_ERROR;
		nDataLen-=nRet;
	}

	ESleep(100);//chengyi for test

	nReadLen = nResLen;
	nDataLen = nReadLen;

	while(nDataLen>0)//receive response
	{
		nRet = m_piSerialPort->Read(bRecvBuf + nReadLen - nDataLen, nDataLen, 50);
		if (nRet <= 0)
		{
			return Read_Data_Error;
		}

		nDataLen-=nRet;
	}
	/*
	for (eint n = 0; n<nReadLen; n++)
		ETRACE(_S("%02x "), bRecvBuf[n]);
	ETRACE(_S(" recved\n"));
*/
	memcpy(pbResBuf,bRecvBuf,(euint32)nResLen);

	return ERR_NONE;
}

eint CMotorCom::Ymodem_SendCmd_NoRecv(euint8* pbReqBuf,eint nReqLen)
{
	if(NULL == pbReqBuf || nReqLen >510 )
	{
		//add log
		return Wrong_Values;
	}

	eint nDataLen=0,nRet=0,nReadLen=0,nSendLen=0;
	euint8 bSendBuf[512] = { 0 };
	euint8 bRecvBuf[512] = { 0 };

	CAutoMutex iAutoMutex(m_piMutex);

	memcpy(bSendBuf, pbReqBuf, (euint16)nReqLen);

	/*
		YModem CRC:
		1. CRC begin from the 4th byte.
		2. Send the HCRC firstly,then send the LCRC.
	*/

	euint16 crcval = YModem_ColculateCrc(bSendBuf,(euint16)nReqLen);
	bSendBuf[nReqLen]= (euint8)((crcval>>8)&0x00ff);
	bSendBuf[nReqLen+1]=(euint8)(crcval&0x00ff);

	nSendLen = nReqLen + 2;
	nDataLen = nSendLen;

	for (eint n = 0; n<nDataLen; n++)
		ETRACE(_S("%02x "), bSendBuf[n]);
	ETRACE(_S("send \n"));

	while(nDataLen>0)	//send packet with crc
	{
		nRet = m_piSerialPort->Write(bSendBuf + nSendLen - nDataLen, nDataLen);
		if(nRet<=0)
			return WRITE_DATA_ERROR;
		nDataLen-=nRet;
	}

	return ERR_NONE;
}

eint CMotorCom::Ymodem_FirmwareLoad_SendCmd(euint8* pbReqBuf, eint nReqLen, euint8* pbResBuf, eint nResLen)
{
	if (NULL == pbReqBuf || NULL == pbResBuf || nReqLen >510 || nResLen >512)
	{
		//add log
		return Wrong_Values;
	}

	eint nDataLen = 0, nRet = 0, nReadLen = 0, nSendLen = 0;
	euint8 bSendBuf[512] = { 0 };
	euint8 bRecvBuf[512] = { 0 };

	CAutoMutex iAutoMutex(m_piMutex);

	memcpy(bSendBuf, pbReqBuf, (euint16)nReqLen);

	/*
	YModem CRC:
	1. CRC begin from the 4th byte.
	2. Send the HCRC firstly,then send the LCRC.
	*/

	euint16 crcval = YModem_ColculateCrc(bSendBuf, (euint16)nReqLen);
	bSendBuf[nReqLen] = (euint8)((crcval >> 8) & 0x00ff);
	bSendBuf[nReqLen + 1] = (euint8)(crcval & 0x00ff);

	nSendLen = nReqLen + 2;
	nDataLen = nSendLen;

	for (eint n = 0; n<nDataLen; n++)
		ETRACE(_S("%02x "), bSendBuf[n]);
	ETRACE(_S("send \n"));

	while (nDataLen>0)	//send packet with crc
	{
		nRet = m_piSerialPort->Write(bSendBuf + nSendLen - nDataLen, nDataLen);
		if (nRet <= 0)
			return WRITE_DATA_ERROR;
		nDataLen -= nRet;
	}

	ESleep(6000);//chengyi for test

	nReadLen = nResLen;
	nDataLen = nReadLen;

	while (nDataLen>0)//receive response
	{
		nRet = m_piSerialPort->Read(bRecvBuf + nReadLen - nDataLen, nDataLen, 50);
		if (nRet <= 0)
		{
			return Read_Data_Error;
		}

		nDataLen -= nRet;
	}

	if (nRet <= 0)
		return nRet;
	for (eint n = 0; n<nRet; n++)
		ETRACE(_S("%02x "), bRecvBuf[n]);
	ETRACE(_S(" recved\n"));

	memcpy(pbResBuf, bRecvBuf, (euint32)nResLen);
	return ERR_NONE;
}

/******************************************************************************************************
Function:		    Ymodem_FillSOHFirstPackage
Description:	    Send Fisrt SOH package follow the Ymodem protocol.
					The protocol of the first package is below:

	BytesNum	 1   | 1       | 1        |<----------128Bytes Payload-------------------------->| 1     | 1
	defination	 SOH | PackNum | ~PackNum | image's name | Length of image's name | Other 0 bytes| H-CRC |L-CRC
	Values       1   | 0       | 0xff     | string       | len                    | 0            | H-CRC |L-CRC


					The response of this package is ACK and 'C'.

Input Paramters:
	@ pImageName       The string address of the Image's name.
	@ nImageNameSize   The length of the the Image's name.

Output Parameters:

Return:
	@ ERR_NONE      Successfully
	@ Other values  Unsuccessfully
Author:				Yi Cheng
Others:
*******************************************************************************************************/
eint CMotorCom::Ymodem_FillSOHFirstPackage(euint8 *pImageName,euint nImageNameSize)
{
	if(NULL == pImageName || nImageNameSize >YMODEM_SOH_PACKAGE_PAYLOAD_LEN)
	{
		//add log
		return Wrong_Values;
	}

	eint nRet = 0;
	euint8 bSohPackage[YMODEM_SOH_PACKAGE_LEN]={0};
	//euint8 nResBuf[YMODEM_SOH_PACKAGE_RES_LEN]={0};
	euint8 nResBuf[2]={0};

	bSohPackage[0] = YMODEM_SOH;
	bSohPackage[1] = 0x00;
	bSohPackage[2] = 0xff;

	memcpy(&bSohPackage[3],pImageName,(euint32)nImageNameSize);
	memset(&bSohPackage[3 + nImageNameSize],(eint32)nImageNameSize,1);
	memset(&bSohPackage[3 + nImageNameSize + 1],0x0
			,YMODEM_SOH_PACKAGE_PAYLOAD_LEN - (euint32)nImageNameSize -1);

	nRet = Ymodem_SendCmd(bSohPackage,YMODEM_SOH_PACKAGE_LEN-2,nResBuf,2);

	if(ERR_NONE != nRet)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __FUNCTION__, __LINE__);
		return nRet;
	}

	//Check the response values. Now comment it because of the the private continuous 'C' characters.

	if(YMODEM_ACK != nResBuf[0]||'C' != nResBuf[1])
	{
		for(eint i=0;i<2;i++)
		{
			EwayG_WriteLog(LogPriDebug,_S("ERROR! Ymodem_FillSOHFirstPackage Res[%d]=%d "),i,nResBuf[i]);
		}

		return Ymodem_Res_Error;
	}

	return ERR_NONE;
}

/******************************************************************************************************
Function:		    Ymodem_FillSOHPackage
Description:	    Send Normal SOH packages follow the Ymodem protocol.
					The protocol of the Normal package is below:

	BytesNum	 1   | 1       | 1        |<----------128Bytes Payload--------->| 1     | 1
	defination	 SOH | PackNum | ~PackNum | Image file bytes                    | H-CRC |L-CRC
	Values       1   | 0       | 0xff     | Image file bytes                    | H-CRC |L-CRC


					The response of this package is ACK.

Input Paramters:
	@ pImageName       The address of the Image file bytes.
	@ nImageNameSize   The package Number.

Output Parameters:

Return:
	@ ERR_NONE      Successfully
	@ Other values  Unsuccessfully
Author:				Yi Cheng
Others:
*******************************************************************************************************/
eint CMotorCom::Ymodem_FillSOHPackage(euint8 *pImage ,euint nPackNum)
{
	if(NULL == pImage)
	{
		//add log
		return Wrong_Values;
	}

	eint nRet = 0;
	euint8 bSohPackage[YMODEM_SOH_PACKAGE_LEN]={0};
	euint8 nResBuf[YMODEM_SOH_PACKAGE_RES_LEN] = { 0 };

	bSohPackage[0] = YMODEM_SOH;
	bSohPackage[1] = (euint8)(nPackNum % 256);// when the packge num is bigger than 255,it will begin from 0 again.
	bSohPackage[2] = (euint8)(~(nPackNum%256));

	memcpy(&bSohPackage[3],pImage,YMODEM_SOH_PACKAGE_PAYLOAD_LEN);

	nRet = Ymodem_SendCmd(bSohPackage,YMODEM_SOH_PACKAGE_LEN-2,nResBuf,YMODEM_SOH_PACKAGE_RES_LEN);

	if(ERR_NONE != nRet)
	{
		//add log
		return nRet;
	}

	//Check whether the response value is ACK.
	if(YMODEM_ACK != nResBuf[0]|| 0x78 != nResBuf[1])
	{
		for(eint i=0;i<2;i++)
		{
			EwayG_WriteLog(LogPriDebug,_S("ERROR! Ymodem_FillSOHPackage Res[%d]=%d "),i,nResBuf[i]);
		}

		return Ymodem_Res_Error;
	}

	return ERR_NONE;
}


/******************************************************************************************************
Function:		    Ymodem_FillSOHLastPackage
Description:	    Send Last SOH packages follow the Ymodem protocol.
					The protocol of the Normal package is below:

	BytesNum	 1   | 1       | 1        |<----------128Bytes Payload--------->| 1     | 1
	defination	 SOH | PackNum | ~PackNum | Image file bytes | Other fill bytes | H-CRC |L-CRC
	Values       1   | 0       | 0xff     | Image file bytes | 0x1a             | H-CRC |L-CRC


					The response of this package is ACK.

Input Paramters:
	@ pImageName       The address of the Image file bytes.
	@ nImageNameSize   The package Number.

Output Parameters:

Return:
	@ ERR_NONE      Successfully
	@ Other values  Unsuccessfully
Author:				Yi Cheng
Others:
*******************************************************************************************************/
eint CMotorCom::Ymodem_FillSOHLastPackage(euint8 *pImage,euint nImageSize ,euint nPackNum)
{
	if(NULL == pImage || nImageSize >YMODEM_SOH_PACKAGE_PAYLOAD_LEN)
	{
		//add log
		return Wrong_Values;
	}

	eint nRet = 0;
	euint8 bSohPackage[YMODEM_SOH_PACKAGE_LEN]={0};
	euint8 nResBuf[YMODEM_SOH_PACKAGE_RES_LEN] = { 0 };

	bSohPackage[0] = YMODEM_SOH;
	bSohPackage[1] = (euint8)(nPackNum%256);// when the packge num is bigger than 255,it will begin from 0 again.
	bSohPackage[2] = (euint8)(~(nPackNum%256));

	memcpy(&bSohPackage[3],pImage,(euint32)nImageSize);
	memset(&bSohPackage[3+nImageSize],YMODEM_FILLLASTPACK,YMODEM_SOH_PACKAGE_PAYLOAD_LEN - (euint32)nImageSize);

	nRet = Ymodem_SendCmd(bSohPackage,YMODEM_SOH_PACKAGE_LEN-2,nResBuf,YMODEM_SOH_PACKAGE_RES_LEN);

	if(ERR_NONE != nRet|| YMODEM_ACK != nResBuf[0])
	{
		for(eint i=0;i<2;i++)
		{
			EwayG_WriteLog(LogPriDebug,_S("ERROR! Ymodem_FillSOHFirstPackage nRet=%d Res[%d]=%d "),nRet,i,nResBuf[i]);
		}

		return Ymodem_Res_Error;
	}

	return ERR_NONE;
}

/******************************************************************************************************
Function:		    Ymodem_FillSOHAllZeroPackage
Description:	    Send All-0s SOH packages follow the Ymodem protocol.
					The protocol of the Normal package is below:

	BytesNum	 1   | 1       | 1        |<----------128Bytes Payload--------->| 1     | 1
	defination	 SOH | PackNum | ~PackNum | 0x00 								| H-CRC |L-CRC
	Values       1   | 0       | 0xff     | 0x00								| H-CRC |L-CRC


					The response of this package is ACK.

Input Paramters:

Output Parameters:

Return:
	@ ERR_NONE      Successfully
	@ Other values  Unsuccessfully
Author:				Yi Cheng
Others:
*******************************************************************************************************/
eint CMotorCom::Ymodem_FillSOHAllZeroPackage()
{
	eint nRet = 0;
	euint8 bSohPackage[YMODEM_SOH_PACKAGE_LEN]={0};
	euint8 nResBuf=0;
	bSohPackage[0] = YMODEM_SOH;
	bSohPackage[1] = 0x00;
	bSohPackage[2] = ~0x00;

	memset(&bSohPackage[3],0x0,YMODEM_SOH_PACKAGE_PAYLOAD_LEN);
	nRet = Ymodem_SendCmd(bSohPackage,YMODEM_SOH_PACKAGE_LEN-2,&nResBuf,1);

	if(ERR_NONE != nRet || YMODEM_ACK != nResBuf)
	{
		EwayG_WriteLog(LogPriDebug,_S("ERROR! Ymodem_FillSOHAllZeroPackage nRet=%d Res=%d "),nRet,nResBuf);
		return Ymodem_Res_Error;
	}
	return ERR_NONE;
}

/******************************************************************************************************
Function:		    Ymodem_TransmitFinish
Description:	    Send Transmit Finished bytes follow the Ymodem protocol.
					The protocol is below:

					1)PC UI --> Motor         EOT
					2)Motor --> PC UI         NAK
					3)PC UI --> Motor         EOT
					4)Motor --> PC UI         ACK

Input Paramters:

Output Parameters:

Return:
	@ ERR_NONE      Successfully
	@ Other values  Unsuccessfully
Author:				Yi Cheng
Others:
*******************************************************************************************************/
eint CMotorCom::Ymodem_TransmitFinish()
{
	eint nRet = 0;
	euint8 bSendBuf[1] = {0};
	euint8 bRecvBuf[1] = {0};
	eint nDataLen = 0;

	nDataLen = 1;
	bSendBuf[0] = YMODEM_EOT;
	while(nDataLen>0)	//send packet with crc
	{
		nRet = m_piSerialPort->Write(bSendBuf, nDataLen);
		if(nRet<=0)
			return WRITE_DATA_ERROR;
		nDataLen-=nRet;
	}

	nDataLen = 1;
	while(nDataLen>0)//receive response
	{
		nRet = m_piSerialPort->Read(bRecvBuf, nDataLen, 50);
		if (nRet <= 0)
		{
			return Read_Data_Error;
		}

		nDataLen-=nRet;
	}

	if(YMODEM_NAK != bRecvBuf[0])
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! %s %s %d"), __EFILE__, __FUNCTION__, __LINE__);
		return Ymodem_Res_Error;
	}



	nDataLen = 1;
	bSendBuf[0] = YMODEM_EOT;
	while(nDataLen>0)	//send packet with crc
	{
		nRet = m_piSerialPort->Write(bSendBuf, nDataLen);
		if(nRet<=0)
			return WRITE_DATA_ERROR;
		nDataLen-=nRet;
	}

	nDataLen = 1;
	while(nDataLen>0)//receive response
	{
		nRet = m_piSerialPort->Read(bRecvBuf, nDataLen, 50);
		if (nRet <= 0)
		{
			return Read_Data_Error;
		}

		nDataLen-=nRet;
	}

	if(YMODEM_ACK != bRecvBuf[0])
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! %s %s %d"), __EFILE__, __EFUNCTION__, __LINE__);
		//add log
		return Ymodem_Res_Error;
	}

	return ERR_NONE;
}

/******************************************************************************************************
Function:		    Ymodem_SerialWrite
Description:	    Serial Write interface in Ymodem module.

Input Paramters:
	@ *pValue		The begin address of the buffer.
	@ nSize         The length of the buffer.

Output Parameters:

Return:
	@ ERR_NONE      Successfully
	@ Other values  Unsuccessfully
Author:				Yi Cheng
Others:
*******************************************************************************************************/
eint CMotorCom::Ymodem_SerialWrite(euint8 *pValue, euint nSize)
{
	if(NULL == pValue || nSize >512)
	{
		//add log
		return Wrong_Values;
	}

	eint nRet,nDataLen = 0;
	euint8 bSendBuf[512] = {0};

	nDataLen =nSize;

	memcpy(bSendBuf, pValue, (euint16)nDataLen);

	while(nDataLen>0)
	{
		nRet = m_piSerialPort->Write(bSendBuf, nDataLen);
		if(nRet<=0)
			return WRITE_DATA_ERROR;
		nDataLen-=nRet;
	}

	return ERR_NONE;
}

/******************************************************************************************************
Function:		    Ymodem_SerialWrite
Description:	    Serial Write interface in Ymodem module.

Input Paramters:
	@ *pValue		The begin address of the buffer.
	@ nSize         The length of the buffer.

Output Parameters:

Return:
	@ ERR_NONE      Successfully
	@ Other values  Unsuccessfully
Author:				Yi Cheng
Others:
*******************************************************************************************************/
eint CMotorCom::Ymodem_SerialRead(euint8 *pValue, eint nSize)
{
	if(NULL == pValue || nSize >512)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! %s %s %d"), __EFILE__, __EFUNCTION__, __LINE__);
		return Wrong_Values;
	}

	eint nRet = 0;
	euint8 bRecvBuf[512] = {0};
	eint nDataLen = 0;

	nDataLen = nSize;
	while(nDataLen>0)//receive response
	{
		nRet = m_piSerialPort->Read(bRecvBuf, nDataLen, 50);
		if (nRet <= 0)
		{
			return Read_Data_Error;
		}

		nDataLen-=nRet;
	}


	memcpy(pValue,bRecvBuf,(euint32)nSize);

	return ERR_NONE;
}

eint CMotorCom::Ymodem_MotorRebootPack(eint nAddr)
{
	eint nRetLen=0;
	euint8 bReqBuf[256] = { 0 };
	euint8 bResBuf[256] = { 0 };

	FillReqHeader(bReqBuf, nAddr, ReqHeaderLength, CtrlCode_RebootSend);
	nRetLen = SendCmd(bReqBuf,ReqHeaderLength,bResBuf,YmodemRebootReplyLen-2);
	if(nRetLen<=0)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRetLen, __EFILE__, __EFUNCTION__, __LINE__);
		return nRetLen;
	}

	if(0xff != bResBuf[0]
	   || 0xff != bResBuf[1]
	   || nAddr != bResBuf[2]
	   || YmodemRebootReplyLen != bResBuf[3]
	   || CtrlCode_RebootSend != bResBuf[4]
	   || MERR_NONE != GetErrCodeByMErrCode(bResBuf[5]))
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRetLen, __EFILE__, __EFUNCTION__, __LINE__);
		return Ymodem_RebootMotor_Error;
	}

	return ERR_NONE;
}


eint CMotorCom::Stop()
{
	m_bStopThread = true;
	m_pRecvThread->WaitForThreadQuit();
	return 0;
}

eint CMotorCom::Run()
{
	m_pRecvThread = CEBaseFactory::GetThreadInstance();
	m_pRecvThread->CreateThread(CMotorCom::WaitRecv, (eint)(this));
	return 0;
}



void* CMotorCom::WaitRecv(void* args)
{
	eint nRet = 0;
	CSerialPort* pSerial = ((CMotorCom*)args)->GetSerialPort();
	euint8 bBuf[128] = { 0 };

	while (!((CMotorCom*)args)->GetThreadStatus())
	{
		nRet = pSerial->Read(bBuf, 9,5000);
		if (nRet > 0)
		{
			EwayG_WriteLog(LogPriDebug, _S("OK !!! bBuf= %d %d %d %d %d %d %d %d %d"), bBuf[0],bBuf[1]
				, bBuf[2], bBuf[3], bBuf[4], bBuf[5], bBuf[6], bBuf[7], bBuf[8]);
			if (9 == nRet
				&& 0xff == bBuf[0]
				&& 0xff == bBuf[1]
				&& YmodemRebootMasterRecvLen == bBuf[3])
			{
				EwayG_WriteLog(LogPriDebug, _S("OK !!! Recieve the motor bring up package successfully!!!"));

				((CMotorCom*)args)->Stop();
			}

		}

	}
	return NULL;
}

eint CMotorCom::Ymodem_MotorBringUpPack(eint nAddr,eint nCtrlCodeRes,eint nCtrlCodeMasterRes)
{
	eint unCnt = 0;
	eint nRet = 0;
	euint8 bRecvBuf[256] = {0};
	euint8 bReqBuf[256] = {0};
	euint16 unBootVer = 0;
	euint8 unCheckFlag = 0;

	FillReqHeader(bReqBuf, nAddr, ReqHeaderLength, nCtrlCodeRes);
	Run();

	while(!m_bStopThread)
	{
		nRet = Ymodem_SendCmd_NoRecv(bReqBuf,ReqHeaderLength);

		if(unCnt>200)
		{
			EwayG_WriteLog(LogPriDebug,_S("ERROR!!! Retry time = 200"));
			Ymodem_SetBootVer(0xffff);
			return Ymodem_RebootMotor_Error;
		}

		ESleep(50);
		unCnt++;
	}

	FillReqHeader(bReqBuf, nAddr, ReqHeaderLength, nCtrlCodeMasterRes);
 	nRet = Ymodem_FirmwareLoad_SendCmd(bReqBuf,ReqHeaderLength,&unCheckFlag,1);
	if(ERR_NONE!=nRet)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR! nRet=%d %s %s %d"), nRet, __EFILE__, __EFUNCTION__, __LINE__);
		return nRet;
	}

	if('C' != unCheckFlag)
	{
		EwayG_WriteLog(LogPriDebug, _S("ERROR!!! Can not Recieve the 'C' from Motor. %s %s %d"), __EFILE__, __EFUNCTION__, __LINE__);

        EwayG_WriteLog(LogPriDebug, _S("ERROR! Ymodem_MotorBringUpPack nRet=%d bCheckFlag=%d "), nRet,unCheckFlag);

		return Ymodem_Check_C_Error;
	}

	return ERR_NONE;
}

eint CMotorCom::Ymodem_SetBootVer(euint16 nBootVer)
{
	CAutoMutex iAutoMutex(m_piMutex);

	m_nBootVer = nBootVer;
	return ERR_NONE;
}

eint CMotorCom::Ymodem_GetBootVer(euint16 &nBootVer)
{
	CAutoMutex iAutoMutex(m_piMutex);

	nBootVer=m_nBootVer;
	return ERR_NONE;
}

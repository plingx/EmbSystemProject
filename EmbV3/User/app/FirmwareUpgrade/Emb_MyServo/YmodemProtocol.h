eint Ymodem_SendCmd(euint8* pbReqBuf,eint nReqLen,euint8* pbResBuf,eint nResLen);
eint Ymodem_SendCmd_NoRecv(euint8* pbReqBuf,eint nReqLen);
eint Ymodem_FirmwareLoad_SendCmd(euint8* pbReqBuf, eint nReqLen, euint8* pbResBuf, eint nResLen);

eint Ymodem_FillSOHFirstPackage(euint8 *pImageName,euint nImageNameSize);
eint Ymodem_FillSOHPackage(euint8 *pImage ,euint nPackNum);
eint Ymodem_FillSOHLastPackage(euint8 *pImage,euint nImageSize ,euint nPackNum);
eint Ymodem_FillSOHAllZeroPackage();
eint Ymodem_TransmitFinish();
eint Ymodem_SerialWrite(euint8 *pValue, euint nSize);
eint Ymodem_SerialRead(euint8 *pValue, eint nSize);
eint Ymodem_MotorRebootPack(eint nAddr);
eint Ymodem_MotorBringUpPack(eint nAddr,eint nCtrlCodeRes,eint nCtrlCodeMasterRes);

eint Ymodem_SetBootVer(euint16 nBootVer);
eint Ymodem_GetBootVer(euint16 &nBootVer);


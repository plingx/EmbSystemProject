/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Emb_PC.h
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2018-05-30
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/05/30 | 0.0.1 | LingPing | Create file
*
*/
#ifndef _EMB_PC_H_
#define _EMB_PC_H_


#define CMD_PC2SHOULDER_INFO_LEN         24   //!< define the PC send shoulder cmd buffer length  ���20Bytes
#define CMD_PC2SHOULDER_QUEU_NUM         18

#define CMD_PC2ARMS_INFO_LEN             84   //!< define the PC send Arms cmd buffer length  ���82Bytes
#define CMD_PC2ARMS_QUEU_NUM             18

#define CMD_PC2HEAD_INFO_LEN             24   //!< define the PC send Head cmd buffer length  ���22Bytes
#define CMD_PC2HEAD_QUEU_NUM             18

#define CMD_PC2WHEEL_INFO_LEN            28   //!< define the PC send Wheel cmd buffer length  ���26Bytes
#define CMD_PC2WHEEL_QUEU_NUM            18

#define CMD_PC2GRIPPER_INFO_LEN          28   //!< define the PC send Gripper cmd buffer length  ���26Bytes
#define CMD_PC2GRIPPER_QUEU_NUM          18

#define CMD_PC2EMBSYS_INFO_LEN           20   //!< define the PC send EmbSystem cmd buffer length  ���18Bytes
#define CMD_PC2EMBSYS_QUEU_NUM           10

#define CMD_EMBWRLOG_INFO_LEN            128 //!< define the PC send shoulder cmd buffer length  ���128Bytes
#define CMD_EMBWRLOG_QUEU_NUM            10

#define CMD_EMBSYS2PSM_INFO_LEN          16   //!< define the EmbSystem send cmd to PSM buffer length  ���16Bytes
#define CMD_EMBSYS2PSM_QUEU_NUM           4


/**
* Define the CMD To PC.
*/
#define  PSU_Status                                                0x0000  //!< ���״̬
#define  PSM_Status                                                0x0001  //!<��Դ���ذ�״̬
#define  IMU_Status                                                0x0002  //!<IMU״̬
#define  TOF_Status                                                0x0003
#define  RDS_Status                                                0x0004
#define  MIC_Status                                                0x0005
#define  TOF_NUM                                                   6

#define  SingleCy_Motor_NUM                                        6  //!< Ŀǰ�����ϴ����6����


//����״̬���˶�״̬�ϲ�
#define  Motor_Status                                               0x0010  //!< ����˶�״̬  ˫��+ͷ
#define  Shoulder_Status                                            0x0011  //!< ����˶�״̬  �粿
#define  Wheel_Status                                               0x0020  //!< �����˶�״̬
#define  Manipulation_Status                                        0x0030  //!< צ���˶�״̬
#define  SingleCycle_JointMotors_Status                             0x0040  //< �����ڶ��ָ����״̬
#define  SingleCycle_WheelMotors_Status                             0x0041  //< �����ڶ��ָ����״̬
#define  EmbSystemSoftwareVersion_Info                              0x00F0  //!< ��λ������汾����Ϣ

#define  EmbSTOP_Preparing                                          0x00FE  //!< �ػ�����
/**
* Define the CMD Len to PC.
*/

#define  PSU_Status_Length                                         0x17  //!< ���״̬���ݳ���
#define  PSM_Status_Length                                         0x2a  //!< ��Դ���ذ�״̬����
#define  IMU_Status_Length                                         0x1E  //!< IMU״̬����
#define  TOF_Status_Length                                         (0x0A+2*(TOF_NUM+1))  //!< TOF״̬����
#define  RDS_Status_Length                                         0x17  //!< RDS״̬����*/
#define  MIC_Status_Length                                         0x10
#define  EmbSysSoftVer_Length                                      0x11  //!< ����汾��Ϣ���ݳ���

//����״̬���˶�״̬�ϲ�
#define  Motor_Status_Length                                (0x0F*15+0xA+2)  //!< ���״̬*����
#define  Shoulder_Status_Lenght                             (0x13+0x0A)     //!< һ�����ĳ���   
#define  Wheel_Status_Length                                (0x11*2+0x0A)  //!< ����״̬����
#define  Manipulation_Status_Length                         (0x0F*2+0x0A)  //!< צ��״̬*����

#define  SingleCycle_Motor_Unit_Length                      0x0E    //!> id(2) + DnPos(4) + DnSpd(2) + UpPos(4) + UpSpd(2)
#define  SingleCycle_Motors_Status_Length                   (SingleCycle_Motor_Unit_Length*SingleCy_Motor_NUM+0x0A) //!< �����ڶ��ָ��&״̬ ������

#define  EmbSTOP_Preparing_Length                                  0x0C  //!< �ػ�����*����


/**
* Define the CMD with PC.
*/
//===============================================================================================//
#define  PC_CMD_FRAME_HEADER                                      0xE1FE   //!< ֡ͷ
#define  MAX_ATE_LENTH                                            500
#define  PC_CMD_HEAD_LENTH                                        10      
#define  PC_CMD_PKT_LEN_MAX                                       128      //!< �޶�ָ������Ȳ�����128(cdm+timeStamp+datalen+Datas....)


#define  PCCmd_ArmPosMoveMode                                        0x0110   //!< �ֱ�λ��ģʽ
#define  PCCmd_ArmSpdMoveMode                                        0x0111   //!< �ֱ��ٶ�ģʽ
#define  PCCmd_ArmPIDMode                                            0x0112   //!< �ֱ�PID����
#define  PCCmd_ArmTorqueSwitch                                       0x0113   //!< �ֱ�ת�ؿ���
#define  PCCmd_ArmMotorLimit                                         0x0114   //!< �ֱ�ת������

#define  PCCmd_HeadPosMove                                           0x0120   //!< ͷλ��ģʽ
#define  PCCmd_HeadSpeedMove                                         0x0121   //!< ͷ�ٶ�ģʽ
#define  PCCmd_HeadPID                                               0x0122   //!< ͷPID����
#define  PCCmd_HeadTorqueSwitch                                      0x0123   //!< ͷת�ؿ���
#define  PCCmd_HeadMotorLimit                                        0x0124   //!< ͷת������
 
#define  PCCmd_WheelPositionMove                                     0x0130   //!< ����λ��ģʽ
#define  PCCmd_WheelSpeedMove                                        0x0131   //!< �����ٶ�ģʽ
#define  PCCmd_WheelPID                                              0x0132   //!< ����PID����
#define  PCCmd_WheelTorqueSwitch                                     0x0133   //!< ����ת�ؿ���
#define  PCCmd_WheelMotorLimit                                       0x0134   //!< ����ת������

#define  PCCmd_GripperMove                                           0x0140   //!< צ���˶�
#define  PCCmd_GripperPID                                            0x0142   //!< צ��PID����
#define  PCCmd_GripperTorqueSwitch                                   0x0143   //!< צ��ת�ؿ���
#define  PCCmd_GripperMotorLimit                                     0x0144   //!< צ��ת������

#define  PCCmd_ShoulderPosMoveMode                                   0x0160   //!< ���λ��ģʽ
#define  PCCmd_ShoulderSpdMoveMode                                   0x0161   //!< ����ٶ�ģʽ
#define  PCCmd_ShoulderPIDMode                                       0x0162   //!< ���PID����
#define  PCCmd_ShoulderTorqueSwitch                                  0x0163   //!< ���ת�ؿ���
#define  PCCmd_ShoulderMotorLimit                                    0x0164   //!< ���ת������

#define  PCCmd_Led                                                   0x0150   //!< ����˸״̬
#define  PCCmd_Clear                                                 0x0100   //!< �������
#define  PCCmd_ObstclAvoid                                           0x0101   //!< ���Ͽ�������
#define  PCCmd_HeartBeat                                             0x0102   //!< ������
#define  PCCmd_SysStop                                               0x01FE   //!< �ػ�ָ��
#define  PCCmd_Laser                                                 0x01FF   //!< ���⿪������

#define  PCCmd_WheelStop                                             (s16)0x8000   //!< �ػ�ָ��
#define  PCCmd_ServoStop                                             0   //!< �ػ�ָ��
#define  PCCmd_Motor_Invilid                                            30001   //!< 

#define  PCCmd_DebugCtrl                                             0xE001   //!< Debug����ָ��


#define PCCmd_ArmPosMoveMode_Unit                                    6
#define PCCmd_ArmSpdMoveMode_Unit                                    6
#define PCCmd_HeadPosMove_Unit                                       6
#define PCCmd_HeadSpeedMove_Unit                                     6
#define PCCmd_ShoulderPosMoveMode_Unit                               10     //!< 8
#define PCCmd_ShoulderSpdMoveMode_Unit                               10     //!< 8
#define PCCmd_ShoulderPosPIDSet_Unit                                 8
#define PCCmd_WheelPositionMove_Unit                                 8
#define PCCmd_WheelSpeedMove_Unit                                    4
#define PCCmd_WheelPosPIDSet_Unit                                    8
#define PCCmd_GripperMove_Unit                                       6
#define PCCmd_GripperTorqueSw_Unit                                   3
#define PCCmd_GripperTorqueLm_Unit                                   4
#define PCCmd_DebugCtrl_Unit                                         8


#define PCCmd_CLEAR_ARM_MASK_BIT                                    0x01
#define PCCmd_CLEAR_SHOULDER_MASK_BIT                               0x04
#define PCCmd_CLEAR_HEAD_MASK_BIT                                   0x08
#define PCCmd_CLEAR_WHEEL_MASK_BIT                                  0x10
#define PCCmd_CLEAR_GRIPPER_MASK_BIT                                0x20

#define Shoulder_Status_CommErr_MaskBit                             0x01




//===============================================================================================//
#define CMD_PC_2_EMB_BUFF_NUMS             16   //<! store cmd pc to emb(shoulder,gripper,wheels and arms) buffer numbers
#define CMD_PC_2_EMB_BUFF_LEN              18   //!< ÿ��ָ�����󳤶�: code(2) + timStamp(4) + cmdContent(10)

//!< ����ÿ��Motor��ָ���
typedef struct{
    u8 dCmd[CMD_PC_2_EMB_BUFF_NUMS][CMD_PC_2_EMB_BUFF_LEN];      //!< 14*16=224 //18*16=288bytes  modified by ling at 0529 
    u8 dCmdWr;
	u8 dCmdRe;
	u8 dCmdCnt;    
    osMutexId* mCmdBuffHandle;
}MotorCMDBuff;


typedef struct{
	u8* pSnd[PRINTF_TX_BUFF_CNT];
	u8* pRev[PRINTF_RX_BUFF_CNT];
	u16 SndCnt[PRINTF_TX_BUFF_CNT];
	u8  bsy[PRINTF_TX_BUFF_CNT];                              //!< bit x,channel x        1:busy,0:idle
	u8 CurBuf;
}TCPSENDCTRL;


typedef enum{
	CMDClearID_Arm = 0x10,
	CMDClearID_Head=0x20,
	CMDClearID_Wheel=0x30,
	CMDClearID_Shoulder=0x40
}CMDClearID_Type;






void sysExtractLegalPktFromTCP(EmbTcpRecvModule* pTcpConn);
s8 sysCmdRecvdFromPCprocess(u8* pMsg,u16 len);
s8 sysGeneralWriteDataToTcpSendBuffer(u8* pdat,u16 len,EmbTcpTransModule* pTrans);
void sysSendEmbInfoToPC(void);
s8 sysGetImuInfo(u8* pdat,u16* plen);
s8 sysGetRDSInfo(u8* pdat,u16* plen);
s8 sysGetMICInfo(u8* pdat,u16* plen);
s8 sysGetShoulderMotorInfo(u8* pdat,u16* plen);


//��������λ�����Э�飬״̬�ϱ��ĺ���

s8 sysGetWheelInfo(u8* pdat,u16* plen);
s8 sysGetMotorInfo(u8* pdat,u16* plen);
s8 sysGetGripperInfo(u8* pdat,u16* plen);
s8 sysGetRadarInfo(u8* pdat,u16* plen);
s8 sysGetPSUInfo(u8* pdat,u16* plen);
s8 sysGetPSMInfo(u8* pdat,u16* plen);
void sysPcCommMaintainWork(void);
s8 sysGetEmbSystemSoftwareVersion(u8* pdat,u16* plen);

//��������λ�����Э�飬�����·��ĺ���
s8 PC_PowerOffCMD(u8 *pUData,u16 unLen);
s8 PC_CmdLedCtrl(u8 *pUData,u16 unLen);
s8 PC_CmdLaserCtrl(u8 *pUData,u16 unLen);
s8 PC_CmdClear(u8 *pUData,u16 unLen);
s8 PC_CmdObstSwitch(u8 *pUData,u16 unLen);
s8 PC_CmdDebugCtrl(u8 *pUData,u16 unLen);
s8 PC_CmdHeartBeat(u8 *pUData,u16 unLen);




#endif

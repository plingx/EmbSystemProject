/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file Arms_Cmd.h
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
* @author 
* @version 0.0.1
* @date 2018-01-30
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/01/30 | 0.0.1 | Ling     | Create file
*
*/
#ifndef _APP_MODULE_ARMS_CMD_H_
#define _APP_MODULE_ARMS_CMD_H_


//s8 sysModuleArmsLeftMovement(EwayArmModule* pModule);
//s8 sysModuleArmsRightMovement(EwayArmModule* pModule);
s8 sysModuleArmsMovement(EwayArmModule* pModule,EwayMotor_Device_Type devType);



#endif

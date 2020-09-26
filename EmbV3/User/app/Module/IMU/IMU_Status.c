/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file IMU_Status.c
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.IMU��д��ͨ��״̬ά��
* @author 
* @version 0.0.1
* @date 2018-03-13
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/03/13 | 0.0.1 | Ling       | Create file
*
*/



#include "includes.h"
#include "bsp.h"





s8 sysIMUCommMaintain(EwayIMUModule* pModule)
{
    if(((pModule->Comm.resIMU)&0x00FF)==0x00FF)//!< ���iic���߶�д״̬������������ζ��������������³�ʼ��iic����
    {
        GPIOB->BSRRL = GPIO_Pin_8 ;        //!< Set SCLK high.
        GPIOB->BSRRL = GPIO_Pin_9 ;        //!< Set SDA high.    

        pModule->Comm.sta += 1;                 //!< delay One Time Cycle to enable IIC Communication
        
        pModule->Comm.resIMU = 0x0000;
    }

    return ERR_NONE;
}




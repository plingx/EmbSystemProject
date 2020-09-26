/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file IMU_Status.c
* @brief 本文件的概要说明.
* @details 本文件的详细说明.IMU读写及通信状态维护
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
    if(((pModule->Comm.resIMU)&0x00FF)==0x00FF)//!< 检查iic总线读写状态，若有连续多次读错误的情况则重新初始化iic总线
    {
        GPIOB->BSRRL = GPIO_Pin_8 ;        //!< Set SCLK high.
        GPIOB->BSRRL = GPIO_Pin_9 ;        //!< Set SDA high.    

        pModule->Comm.sta += 1;                 //!< delay One Time Cycle to enable IIC Communication
        
        pModule->Comm.resIMU = 0x0000;
    }

    return ERR_NONE;
}




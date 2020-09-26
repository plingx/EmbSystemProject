/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp_i2c.h
* @brief 本文件的概要说明.
* @details 本文件的详细说明.
* @author lingping@ewaybot.com
* @version 0.0.1
* @date 2017-12-20
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2017/12/20 | 0.0.1 | lingping | Create file
*
*/
#ifndef _BSP_I2C_H_
#define _BSP_I2C_H_


#define AT24C1024_WRITE_ADDR	0xA0
#define AT24C1024_READ_ADDR	    0xA1

#define IMU_JY901_WRITE_ADDR	0xA0
#define IMU_JY901_READ_ADDR	    0xA1



void BspI2c1_init(void);
void BspI2c2_init(void);
s8 I2C1_Read(u8 regAddr, u8 *data_in, u16 nums);
s8 I2C_Read(u32 address, u8 *data_in, u16 nums);
s8 I2C_Write(u32 address, u8 *data_out, u16 nums);


#endif

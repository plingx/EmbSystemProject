/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file bsp_i2c.h
* @brief ���ļ��ĸ�Ҫ˵��.
* @details ���ļ�����ϸ˵��.
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

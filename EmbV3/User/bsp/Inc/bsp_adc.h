/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp_adc.h
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
* 2017/12/20 | 0.0.1 | LingPing | Create file
*
*/
#ifndef _BSP_ADC_H_
#define _BSP_ADC_H_


#define ADC_AVERAGE_SAMPLE_VALUE (32)

#define LIMIT_SWITCH_LOW_POS          10840U               //!< 拉线传感器测量低限位开关的高度为108.40mm
#define LIMIT_SWITCH_LENGTH_SHORT           39200U               //!< Moro1 高低限位开关距离为400mm 要减去8mm钢板厚度；Moro2为579mm   
//!< #define LIMIT_SWITCH_LENGTH           57900U            //!< Moro2高低限位开关距离为587mm 要减去8mm钢板厚度


void BspAdc3_init(void);
u8 GetADC3RopeDisplacementSensorInfo(u32* rslt);
#endif

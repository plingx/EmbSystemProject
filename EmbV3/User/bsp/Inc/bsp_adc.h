/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file bsp_adc.h
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
* 2017/12/20 | 0.0.1 | LingPing | Create file
*
*/
#ifndef _BSP_ADC_H_
#define _BSP_ADC_H_


#define ADC_AVERAGE_SAMPLE_VALUE (32)

#define LIMIT_SWITCH_LOW_POS          10840U               //!< ���ߴ�������������λ���صĸ߶�Ϊ108.40mm
#define LIMIT_SWITCH_LENGTH_SHORT           39200U               //!< Moro1 �ߵ���λ���ؾ���Ϊ400mm Ҫ��ȥ8mm�ְ��ȣ�Moro2Ϊ579mm   
//!< #define LIMIT_SWITCH_LENGTH           57900U            //!< Moro2�ߵ���λ���ؾ���Ϊ587mm Ҫ��ȥ8mm�ְ���


void BspAdc3_init(void);
u8 GetADC3RopeDisplacementSensorInfo(u32* rslt);
#endif

/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file bsp_adc.c
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

#include "bsp.h"

/* --------------------------------------------------------------------------*/
/**
* @name BspAdc3_init
* @brief 
* @details initialize ADC3 interface:Pins&ADC config
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void BspAdc3_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;	
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructyre;
	__IO uint32_t counter = 0;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);      //!< ADC3_IN14 PF4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,ENABLE);

	//!< �� PF4 ӳ��Ϊ ADC3_IN14 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOF,&GPIO_InitStructure);

	//!< ADCͨ������
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

	//!< ADC3����
    ADC_InitStructyre.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructyre.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructyre.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructyre.ADC_ExternalTrigConvEdge = 0x0000;
    ADC_InitStructyre.ADC_NbrOfConversion = 1;  
    ADC_InitStructyre.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructyre.ADC_ScanConvMode = DISABLE;
    ADC_Init(ADC3,&ADC_InitStructyre);
  
    ADC_Cmd(ADC3,ENABLE);
    ADC_RegularChannelConfig(ADC3,ADC_Channel_14,1,ADC_SampleTime_144Cycles);
	
	ADC_SoftwareStartConv(ADC3);

	counter = (3 * (SystemCoreClock / 1000000));          //!< wait for some cycles
	
    while(counter != 0)
    {
       counter--;
    }    
}


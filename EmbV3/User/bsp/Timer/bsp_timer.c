/*
*********************************************************************************************************
*
*	ģ������ : ��ʱ��ģ��
*	�ļ����� : bsp_timer.c
*	��    �� : V1.2
*	˵    �� : ����systick��ʱ����Ϊϵͳ�δ�ʱ����ȱʡ��ʱ����Ϊ1ms��
*
*				ʵ���˶�������ʱ����������ʹ��(����1ms)�� ����ͨ���޸� TMR_COUNT ������ʱ������
*				ʵ����ms�����ӳٺ���������1ms�� ��us���ӳٺ���
*				ʵ����ϵͳ����ʱ�亯����1ms��λ��
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-02-01 armfly  ��ʽ����
*		V1.1    2013-06-21 armfly  ����us���ӳٺ��� bsp_DelayUS
*		V1.2    2014-09-07 armfly  ����TIM4 Ӳ����ʱ�жϣ�ʵ��us����ʱ.20us - 16��
*
*	Copyright (C), 2014-2015, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"
#include "Led.h"




/* ���������ʱ���ṹ����� */

 
 uint32_t lwip_localtime;//lwip����ʱ�������,��λ:ms
 uint32_t localtime;//lwip����ʱ�������,��λ:ms
/*
	ȫ������ʱ�䣬��λ1ms
	����Ա�ʾ 24.85�죬�����Ĳ�Ʒ��������ʱ�䳬�������������뿼���������
*/
//__IO int32_t g_iRunTime = 0;

//static void bsp_SoftTimerDec(SOFT_TMR *_tmr);


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitTimer
*	����˵��: ����systick�жϣ�����ʼ�������ʱ������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
/*
void bsp_InitTimer(void)
{
	//
	//	����systic�ж�����Ϊ1ms��������systick�жϡ�

    //	SystemCoreClock �ǹ̼��ж����ϵͳ�ں�ʱ�ӣ�����STM32F4XX,һ��Ϊ 168MHz

    //	SysTick_Config() �������βα�ʾ�ں�ʱ�Ӷ��ٸ����ں󴥷�һ��Systick��ʱ�ж�.
	//    	-- SystemCoreClock / 1000  ��ʾ��ʱƵ��Ϊ 1000Hz�� Ҳ���Ƕ�ʱ����Ϊ  1ms
	//    	-- SystemCoreClock / 500   ��ʾ��ʱƵ��Ϊ 500Hz��  Ҳ���Ƕ�ʱ����Ϊ  2ms
	//    	-- SystemCoreClock / 2000  ��ʾ��ʱƵ��Ϊ 2000Hz�� Ҳ���Ƕ�ʱ����Ϊ  500us

    //	���ڳ����Ӧ�ã�����һ��ȡ��ʱ����1ms�����ڵ���CPU���ߵ͹���Ӧ�ã��������ö�ʱ����Ϊ 10ms
    //
	//  SysTick_Config(SystemCoreClock / 1000);   //1ms
#if defined (USE_TIM2) || defined (USE_TIM3)  || defined (USE_TIM4)	|| defined (USE_TIM5)
	//bsp_InitHardTimer(); //StepMotorʹ����TIM2,��StepMotorInit�г�ʼ����TIM2��ؼĴ���
#endif
}
*/

/*
*********************************************************************************************************
*	�� �� ��: SysTick_ISR
*	����˵��: SysTick�жϷ������ÿ��1ms����1��
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitHardTimer
*	����˵��: ���� TIM4������us����Ӳ����ʱ��TIM4���������У�����ֹͣ.
*			TIM4������TIM2 - TIM5 ֮���TIM, ��ЩTIM��4��ͨ��, ���� APB1 �ϣ�����ʱ��=SystemCoreClock / 2
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
/*
void bsp_InitHardTimer(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint16_t usPeriod;
	uint16_t usPrescaler;
	uint32_t uiTIMxCLK;

  	// ʹ��TIMʱ�� //
	RCC_APB1PeriphClockCmd(TIM_HARD_RCC, ENABLE);

    //-----------------------------------------------------------------------
	//	system_stm32f4xx.c �ļ��� void SetSysClock(void) ������ʱ�ӵ��������£�

	//	HCLK = SYSCLK / 1     (AHB1Periph)
	//	PCLK2 = HCLK / 2      (APB2Periph)
	//	PCLK1 = HCLK / 4      (APB1Periph)

	//	��ΪAPB1 prescaler != 1, ���� APB1�ϵ�TIMxCLK = PCLK1 x 2 = SystemCoreClock / 2;
	//	��ΪAPB2 prescaler != 1, ���� APB2�ϵ�TIMxCLK = PCLK2 x 2 = SystemCoreClock;

	//	APB1 ��ʱ���� TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7, TIM12, TIM13,TIM14
	//	APB2 ��ʱ���� TIM1, TIM8 ,TIM9, TIM10, TIM11

	//----------------------------------------------------------------------- //
	uiTIMxCLK = SystemCoreClock / 2;

	usPrescaler = uiTIMxCLK / 1000000 ;	// ��Ƶ������ 1us //
	usPeriod = 0xFFFF;

	// Time base configuration //
	TIM_TimeBaseStructure.TIM_Period = usPeriod;
	TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM_HARD, &TIM_TimeBaseStructure);

	//TIM_ARRPreloadConfig(TIMx, ENABLE);

	// TIMx enable counter //
	TIM_Cmd(TIM_HARD, ENABLE);

	// ����TIM��ʱ�ж� (Update) //
	{
		NVIC_InitTypeDef NVIC_InitStructure;	// �жϽṹ���� misc.h �ж��� //

		NVIC_InitStructure.NVIC_IRQChannel = TIM_HARD_IRQn;

		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;	// �ȴ������ȼ��� //
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
}
*/

/*
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM_HARD, TIM_IT_CC1))
    {
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC1);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC1, DISABLE);	// ����CC1�ж� //

        // �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� //
        s_TIM_CallBack1();
    }

    if (TIM_GetITStatus(TIM_HARD, TIM_IT_CC2))
    {
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC2);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC2, DISABLE);	// ����CC2�ж� //

        // �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� //
        s_TIM_CallBack2();
    }

    if (TIM_GetITStatus(TIM_HARD, TIM_IT_CC3))
    {
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC3);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC3, DISABLE);	// ����CC3�ж� //

        // �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� //
        s_TIM_CallBack3();
    }

    if (TIM_GetITStatus(TIM_HARD, TIM_IT_CC4))
    {
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC4);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC4, DISABLE);	// ����CC4�ж� //

        // �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� //
        s_TIM_CallBack4();
    }
}
*/


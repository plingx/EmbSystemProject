/*
*********************************************************************************************************
*
*	模块名称 : 定时器模块
*	文件名称 : bsp_timer.c
*	版    本 : V1.2
*	说    明 : 配置systick定时器作为系统滴答定时器。缺省定时周期为1ms。
*
*				实现了多个软件定时器供主程序使用(精度1ms)， 可以通过修改 TMR_COUNT 增减定时器个数
*				实现了ms级别延迟函数（精度1ms） 和us级延迟函数
*				实现了系统运行时间函数（1ms单位）
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2013-02-01 armfly  正式发布
*		V1.1    2013-06-21 armfly  增加us级延迟函数 bsp_DelayUS
*		V1.2    2014-09-07 armfly  增加TIM4 硬件定时中断，实现us级别定时.20us - 16秒
*
*	Copyright (C), 2014-2015, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"
#include "Led.h"




/* 定于软件定时器结构体变量 */

 
 uint32_t lwip_localtime;//lwip本地时间计数器,单位:ms
 uint32_t localtime;//lwip本地时间计数器,单位:ms
/*
	全局运行时间，单位1ms
	最长可以表示 24.85天，如果你的产品连续运行时间超过这个数，则必须考虑溢出问题
*/
//__IO int32_t g_iRunTime = 0;

//static void bsp_SoftTimerDec(SOFT_TMR *_tmr);


/*
*********************************************************************************************************
*	函 数 名: bsp_InitTimer
*	功能说明: 配置systick中断，并初始化软件定时器变量
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
/*
void bsp_InitTimer(void)
{
	//
	//	配置systic中断周期为1ms，并启动systick中断。

    //	SystemCoreClock 是固件中定义的系统内核时钟，对于STM32F4XX,一般为 168MHz

    //	SysTick_Config() 函数的形参表示内核时钟多少个周期后触发一次Systick定时中断.
	//    	-- SystemCoreClock / 1000  表示定时频率为 1000Hz， 也就是定时周期为  1ms
	//    	-- SystemCoreClock / 500   表示定时频率为 500Hz，  也就是定时周期为  2ms
	//    	-- SystemCoreClock / 2000  表示定时频率为 2000Hz， 也就是定时周期为  500us

    //	对于常规的应用，我们一般取定时周期1ms。对于低速CPU或者低功耗应用，可以设置定时周期为 10ms
    //
	//  SysTick_Config(SystemCoreClock / 1000);   //1ms
#if defined (USE_TIM2) || defined (USE_TIM3)  || defined (USE_TIM4)	|| defined (USE_TIM5)
	//bsp_InitHardTimer(); //StepMotor使用了TIM2,在StepMotorInit中初始化了TIM2相关寄存器
#endif
}
*/

/*
*********************************************************************************************************
*	函 数 名: SysTick_ISR
*	功能说明: SysTick中断服务程序，每隔1ms进入1次
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*	函 数 名: bsp_InitHardTimer
*	功能说明: 配置 TIM4，用于us级别硬件定时。TIM4将自由运行，永不停止.
*			TIM4可以用TIM2 - TIM5 之间的TIM, 这些TIM有4个通道, 挂在 APB1 上，输入时钟=SystemCoreClock / 2
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
/*
void bsp_InitHardTimer(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint16_t usPeriod;
	uint16_t usPrescaler;
	uint32_t uiTIMxCLK;

  	// 使能TIM时钟 //
	RCC_APB1PeriphClockCmd(TIM_HARD_RCC, ENABLE);

    //-----------------------------------------------------------------------
	//	system_stm32f4xx.c 文件中 void SetSysClock(void) 函数对时钟的配置如下：

	//	HCLK = SYSCLK / 1     (AHB1Periph)
	//	PCLK2 = HCLK / 2      (APB2Periph)
	//	PCLK1 = HCLK / 4      (APB1Periph)

	//	因为APB1 prescaler != 1, 所以 APB1上的TIMxCLK = PCLK1 x 2 = SystemCoreClock / 2;
	//	因为APB2 prescaler != 1, 所以 APB2上的TIMxCLK = PCLK2 x 2 = SystemCoreClock;

	//	APB1 定时器有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7, TIM12, TIM13,TIM14
	//	APB2 定时器有 TIM1, TIM8 ,TIM9, TIM10, TIM11

	//----------------------------------------------------------------------- //
	uiTIMxCLK = SystemCoreClock / 2;

	usPrescaler = uiTIMxCLK / 1000000 ;	// 分频到周期 1us //
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

	// 配置TIM定时中断 (Update) //
	{
		NVIC_InitTypeDef NVIC_InitStructure;	// 中断结构体在 misc.h 中定义 //

		NVIC_InitStructure.NVIC_IRQChannel = TIM_HARD_IRQn;

		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;	// 比串口优先级低 //
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
        TIM_ITConfig(TIM_HARD, TIM_IT_CC1, DISABLE);	// 禁能CC1中断 //

        // 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 //
        s_TIM_CallBack1();
    }

    if (TIM_GetITStatus(TIM_HARD, TIM_IT_CC2))
    {
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC2);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC2, DISABLE);	// 禁能CC2中断 //

        // 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 //
        s_TIM_CallBack2();
    }

    if (TIM_GetITStatus(TIM_HARD, TIM_IT_CC3))
    {
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC3);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC3, DISABLE);	// 禁能CC3中断 //

        // 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 //
        s_TIM_CallBack3();
    }

    if (TIM_GetITStatus(TIM_HARD, TIM_IT_CC4))
    {
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC4);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC4, DISABLE);	// 禁能CC4中断 //

        // 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 //
        s_TIM_CallBack4();
    }
}
*/


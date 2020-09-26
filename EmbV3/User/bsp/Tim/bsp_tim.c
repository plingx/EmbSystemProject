/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp_tim.c
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
#include "bsp.h"
#include "Radar.h"


extern EwayRadarObstAvoidModule EwayRadarObstAvoid;


u8 RadarDist[RADAR_NUMS_MAX] = {0};                    //!<  port A B C D G E F H
//u8 RadarRecvFlag=0;
static u16 radarRxDat=0;                  //!< Radar signal rx data
static u8 radarRxsta = 0;                 //!< Radar signal rx status


void RadarSaveTimData(u8 bitV);

//extern EwayEmbSysModule EwayEmbSys;


/* --------------------------------------------------------------------------*/
/**
* @name BspTim9_Init
* @brief 
* @details Initialize Timer 9,configure the clock of timer 9 and both edge 
* trigging timer 9 interrupt
*
* @param[in] None
*
* @returns None
*
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void BspTim9_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	NVIC_InitTypeDef NVIC_InitStructure;	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
		
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM9_PRE_PRIO_VALUE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIM9_SUB_PRIO_VALUE;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;                   //!< TIM Config
	TIM_TimeBaseStructure.TIM_Prescaler = TIM9_SYS_PRESCALER;    //!< N+1
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_PWMIConfig(TIM9, &TIM_ICInitStructure);
	
	TIM_SelectInputTrigger(TIM9, TIM_TS_TI1FP1);
	
	TIM_Cmd(TIM9, ENABLE);
	
	TIM_ITConfig(TIM9, TIM_IT_CC1, ENABLE);
}

/* --------------------------------------------------------------------------*/
/**
* @name BspTim3_Init
* @brief 
* @details Initialize Timer 3,configure the clock of timer 3 , time base function 
*
* @param[in] None
*
* @returns None
*
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
/*
void BspTim3_Init(void)
{	
	NVIC_InitTypeDef NVIC_InitStructure;	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIM3_SUB_PRIO_VALUE;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	TIM_TimeBaseStructure.TIM_Period = 20000;                    //!< TIM Config 1us*2000 = 2ms							100us*20000 = 2000ms
	TIM_TimeBaseStructure.TIM_Prescaler = TIM3_SYS_PRESCALER;    //!< N+1  84M的APB1 timer clock,分频84,频率为1M(1us)     84M的APB1,分频8400，频率为10K(100us)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
		
	TIM_Cmd(TIM3, ENABLE);
	
}
*/

/*
void TIM3_IRQHandler(void)
{	
	u8 i;
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{
		for(i=0;i<5;i++)
		{
			if((EwayEmbSys.COMSendCtrl[i].tCtr.Cnt>0)&&(EwayEmbSys.COMSendCtrl[i].tCtr.en>0))
			{
				EwayEmbSys.COMSendCtrl[i].tCtr.Cnt -= 1;
			}
		}
	}

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update );
}
*/


/* --------------------------------------------------------------------------*/
/**
* @name TIM1_BRK_TIM9_IRQHandler
* @brief 
* @details Timer 9 IRQ Handler,According to the trigging counter,recognize the 
* flag bit UltraSonic Radar send
*
* @param[in] None
*
* @returns None
*
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void TIM1_BRK_TIM9_IRQHandler(void)
{
    u32 high_time;
	u8 bit_value;
	
	TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);
	
	if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5))
	{		
		TIM_SetCounter(TIM9, 0);                        //!< reset TIM counter
	} 
	else
	{
		high_time = TIM_GetCapture1(TIM9);		
		
		if ( (high_time > 950) && (high_time < 1050) ) //!< frame start - normal 1000
		{
			bit_value = 'S';

            radarRxsta = 0;     radarRxDat = 0;            
		} 
        else if ( (high_time > 70) && (high_time < 110) ) //!< bit 0 - normal 90
		{
			bit_value = 0x30;
		} 
        else if ( (high_time > 220) && (high_time < 260) ) //!< bit 1 - normal 240
		{
			bit_value = 0x31;
		} else // invalid bit
		{
			bit_value = 'E';
		}
		
		RadarSaveTimData(bit_value);
	}
}


/* --------------------------------------------------------------------------*/
/**
* @name RadarSaveTimData
* @brief 
* @details Timer 9 IRQ Handler,According to the trigging counter,recognize the 
* flag bit UltraSonic Radar send
*
* @param[in] None
*
* @returns None
*
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void RadarSaveTimData(u8 bitV)
{
	u8 index;
	
	if(radarRxsta==0)                        //!< 0阶段只有收到'S'后才会进到1阶段
	{
		if(bitV=='S')
		{
			radarRxsta++;
		}		
	}                                        //!< 1-16阶段只接收 0 or 1，接收到其他字符均打回到0阶段
	else if((radarRxsta>0)&&(radarRxsta<17))//!< 1-16阶段收到0 or 1后就记录起来，并自增1
	{
		if((bitV == 0x30)||(bitV == 0x31))
		{
			bitV = bitV-0x30;		    
			radarRxDat = (radarRxDat<<1) + bitV;
			radarRxsta++;
		}
		else
		{
			radarRxsta = 0;			
			radarRxDat = 0;
		}
	}
	else if(radarRxsta==17)                  //!< 17阶段只接收frame tail 0，否则打回到0阶段
	{
		if((bitV == 0x30)&&((radarRxDat&0xF000)==0x4000))        //!< check the data ???	
		{ 				
			index = (radarRxDat>>8)&0x07;   //!< save the data
			
			//RadarDist[index] = (u8)(radarRxDat&0x00FF);
			EwayRadarObstAvoid.RadarCollect.UltraSoundDist[index] = (u8)(radarRxDat&0x00FF);

            //RadarRecvFlag |= (0x01<<index);     //!< 置接收标志 ling-20180427 modify
            EwayRadarObstAvoid.RadarCollect.RecvFlag = (0x01<<index);
            
		}
    else
		{
			radarRxsta = 0; radarRxDat = 0;
		}
	}
	else
	{
		radarRxsta = 0;	radarRxDat = 0;
	}
}


/* --------------------------------------------------------------------------*/
/**
* @name GetTim9RadarInfo
* @brief 
* @details Get the Radar information
*
* @param[in] datNums Data numbers wants to read
* @param[in] pdat Pointer of data to store
*
* @returns ERR_NONE Successful
*          other value Failed
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 GetTim9RadarInfo(u8 datNums,u8* pdat)
{	
	if (datNums > RADAR_NUMS_MAX) 
	{
		return ERR_INPUT_PARAMETERS;
	}
	
	if(datNums==RADAR_NUM)              //!< ling-20180427  modify 将数据按照1.0.2.3.4.7的顺序拷贝给获取信息的函数.
    {
        pdat[0] = EwayRadarObstAvoid.RadarCollect.UltraSoundDist[1];    //RadarDist[1];
        pdat[1] = EwayRadarObstAvoid.RadarCollect.UltraSoundDist[0];    //RadarDist[0];
        pdat[2] = EwayRadarObstAvoid.RadarCollect.UltraSoundDist[2];    //RadarDist[2];
        pdat[3] = EwayRadarObstAvoid.RadarCollect.UltraSoundDist[3];    //RadarDist[3];
        pdat[4] = EwayRadarObstAvoid.RadarCollect.UltraSoundDist[4];    //RadarDist[4];
        pdat[5] = EwayRadarObstAvoid.RadarCollect.UltraSoundDist[7];    //RadarDist[7];
    }
    else
    {
        memcpy(pdat,&EwayRadarObstAvoid.RadarCollect.UltraSoundDist[0],datNums);//memcpy(pdat,RadarDist,datNums);           
    }
			
	return ERR_NONE;
}


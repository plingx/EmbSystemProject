/**
* ��Ȩ����(C) 2017 - 2022.������һά�ҿƼ����޹�˾. �����˱�������Ȩ��.
*
* ���ļ���������Ewaybot Operating System(EwayOS)����,Ϊ��Դ����,������BSD 2-Clause License��ǰ���£�
* �����߿��ٷ����������Դ���뼰��������ʽʹ����������������޸Ļ򲻽����޸ġ�
* ��˾ͨ�ŵ�ַ�������к�������������·48�ű����Ƽ���չ����1B��16L
*
* @file bsp_gpio.c
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
#include "bsp.h"
#include "bsp_gpio.h"
#include "Led.h"

//leds pins config, leds init status config



void LedsGpio_Init(void);
//===================================================================//





//leds pins config, leds init status config
/* --------------------------------------------------------------------------*/
/**
* @name BspGpios_init
* @brief 
* @details Initialize all the gpio pins.
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void BspGpios_init(void)
{
	s8 res;
	
	LedsGpio_Init();
	
	res = LedsStatus_Init();
	if(ERR_NONE != res)
	{
		Bsp_printf("1-1 LedsStatus_Init() failed.");
	}
	
	SysIOsGpio_Init();
}

/* --------------------------------------------------------------------------*/
/**
* @name LedsGpio_Init
* @brief 
* @details Initialize LEDs pins
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void LedsGpio_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 	  
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}



/* --------------------------------------------------------------------------*/
/**
* @name SysIOsGpio_Init
* @brief 
* @details Initialize pins, includes Up and Down limit switch and Input&Output
* pins 
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void SysIOsGpio_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //!< UpDownLimitInit
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
	
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN;		
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);  //!< OUTPUTInit
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Init(GPIOF, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN;          //!< INPUTInit
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
    
#if Emb_LaserCtrl_Enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);   //!< Laser Pin
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOD, &GPIO_InitStructure);	 
    
    GPIO_ResetBits(GPIOD, GPIO_Pin_15);	                    //!< ��ʼ����Pin�ر�
#endif

}

/* --------------------------------------------------------------------------*/
/**
* @name BspGetAt24c1024BWriteProtectPinStatus
* @brief 
* @details Get the Up and Down limit switch level status:0 or 1
*
* @param[in] sw Up or Down limit switch
* @param[in] psta Pointer of storing Pin Level(Up or Down limit)
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully

* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 BspGetAt24c1024BWriteProtectPinStatus(u8* pRes)
{

   *pRes = GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13);

	return ERR_NONE;
}

/* --------------------------------------------------------------------------*/
/**
* @name BspSetE2PROMWriteProtectPinStatus
* @brief 
* @details Get the Up and Down limit switch level status:0 or 1
*
* @param[in] sw Up or Down limit switch
* @param[in] psta Pointer of storing Pin Level(Up or Down limit)
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully

* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 BspSetAt24c1024BWriteProtectPinStatus(u8 pinSta)
{
	if(pinSta!=0)    
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
	}
	else
	{		
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	}

	return ERR_NONE;
}

/* --------------------------------------------------------------------------*/
/**
* @name BspGetLimitSwitchStatus
* @brief 
* @details Get the Up and Down limit switch level status:0 or 1
*
* @param[in] sw Up or Down limit switch
* @param[in] psta Pointer of storing Pin Level(Up or Down limit)
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully

* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 BspGetLimitSwitchStatus(SYSLIMITSWITCH sw,u8* pSta)
{
	if((sw==UpLim)||(sw==DownLim))
	{
		switch(sw)
		{
			case UpLim:
					*pSta = PCin(9);
				break;
			case DownLim:
					*pSta = PAin(8);
				break;
			default:				
				break;
		}
		
		return ERR_NONE;
	}
	else
	{
		return ERR_INPUT_PARAMETERS;
	}
}

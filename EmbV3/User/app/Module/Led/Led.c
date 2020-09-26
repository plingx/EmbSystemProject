#include "includes.h"
#include "Led.h"




//leds状态初始化要放在管脚初始化完成以后
s8 LedsStatus_Init(void)
{
	s8 res=ERR_NONE;
	
	res = LedsStatus_Set(LED0,LIGHT_OFF);
	if(ERR_NONE != res)
	{
		//res = Bsp_printf("LedsStatus_Init->LedsStatus_Set(LED0)failed.\n");
	}
	
	res = LedsStatus_Set(LED1,LIGHT_OFF);
	if(ERR_NONE != res)
	{
		//res = Bsp_printf("LedsStatus_Init->LedsStatus_Set(LED1)failed.\n");
	}
	
	return res;
}


s8 LedsStatus_Set(SYSLEDS ld,u8 sta)
{
	s8 res = ERR_NONE;
	u8 status = sta;

	if(status!=0) status = LIGHT_OFF;
	else status = LIGHT_ON;
	
	switch(ld)
	{
		case LED0:
				PGout(9) = status;
			break;
			
		case LED1:
				PGout(10) = status;
			break;
			
		default:res = ERR_INPUT_PARAMETERS;
			break;
	}

	return res;
}

s8 LedsStatus_Toggle(SYSLEDS ld)
{
	s8 res=ERR_NONE;

	switch(ld)
	{
		case LED0:
				PGout(9) = !PGout(9) ;
			break;
			
		case LED1:
				PGout(10) = !PGout(10) ;
			break;
			
		default:res = ERR_INPUT_PARAMETERS;
			break;
	}

	return res;
}


s8 LedsStatus_Get(SYSLEDS ld,u8* psta)
{
	s8 res = 0;
			
	switch(ld)
	{
		case LED0:
				*psta = PGout(9);
			break;
				
		case LED1:
				*psta = PGout(10);
				break;

		default:res = ERR_INPUT_PARAMETERS;
				break;
	}
	
	return res;
}



s8 LaserStatus_Set(u8 sta)
{
	if(sta!=0)
    {
        PDout(15) = 1;
    }   
	else
    {
        PDout(15) = 0;
    }   
	
	return ERR_NONE;
}



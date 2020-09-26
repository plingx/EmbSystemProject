#ifndef _APP_LED_H_
#define _APP_LED_H_


//define the LEDs index
#define LED_0         0
#define LED_1         1

//define LEDs   status
#define LIGHT_ON      0
#define LIGHT_OFF     1

//define LEDs
typedef enum{
	LED0 = 1,
	LED1
}SYSLEDS;



//usr functions
s8 LedsStatus_Init(void);
s8 LedsStatus_Set(SYSLEDS ld,u8 sta);
s8 LedsStatus_Toggle(SYSLEDS ld);
s8 LedsStatus_Get(SYSLEDS ld,u8* psta);
s8 LaserStatus_Set(u8 sta);

#endif

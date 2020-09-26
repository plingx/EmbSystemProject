/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp_i2c.c
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


void Start(void);
void Stop(void);
void SendACK(void);
s8 Write(u8 data_out);
s8 Read(u8 *data_in, u8 send_ack);
void SetSDATA(u8 state);
void SetSCLK(u8 state);
u8 GetSDATA(void);
void I2C1Start(void);
void I2C1Stop(void);
void I2C1SendACK(void);
s8 I2C1Write(u8 data_out);
s8 I2C1Read(u8 *data_in, u8 send_ack);
void I2C1SetSDATA(u8 state);
void I2C1SetSCLK(u8 state);
u8 I2C1GetSDATA(void);
s8 SysE2PromWriteProtectFuncSet(u8 sta);


/* --------------------------------------------------------------------------*/
/**
* @name BspI2c2_init
* @brief 
* @details Initializes AT24C1024b I2c interface pins .
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void BspI2c2_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);   //!< PF0(i2c2_sda),PF1(i2c2_scl)

    GPIO_InitStructure.GPIO_Mode =	GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    GPIOF->BSRRL = GPIO_Pin_0|GPIO_Pin_1 ;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				//!< EEPROM_WP PC13 :GND-normal write,VCC-write operations are inhibited,悬空则内部接地.
	GPIO_Init(GPIOC, &GPIO_InitStructure);

    //!< 使能写保护管脚
    SysE2PromWriteProtectFuncSet(ENABLE);
}


/* --------------------------------------------------------------------------*/
/**
* @name BspI2c2_init
* @brief 
* @details Initializes IMU Device I2c interface pins i.
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void BspI2c1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);   //!<PB8,PB9

    GPIO_InitStructure.GPIO_Mode =	GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIOB->BSRRL = GPIO_Pin_8|GPIO_Pin_9 ;
}


/* --------------------------------------------------------------------------*/
/**
* @name I2C_Write
* @brief Write data to AT24Cxxxx
* @details 
*
* @param[in] address Write address
* @param[in] data_out Pointer of Data to be written into AT24Cxxxx
* @param[in] nums Number of data be written
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 I2C_Write(u32 address, u8 *data_out, u16 nums)
{
    u8 addr0;
	u16 i,j;

	if((address&0x00010000)!=0)                            //!< 检查address的bit16是否为1,bit16为Page addr
	{
        addr0 = AT24C1024_WRITE_ADDR|0x02;                 //!< 向i2c写入即将要读的起始地址
	}		
	else
	{
		addr0 = AT24C1024_WRITE_ADDR;
	}
	
	Start();							                   //!<  Send start signal
	if (Write(addr0))				
	{
		Stop();							
		return ERR_PROCESS_FAILED;
	}
	if (Write((u8)(address>>8)))	                       //!<  Send address to device
	{
		Stop();
		return ERR_PROCESS_FAILED;
	}
	if (Write((u8)address))	                               //!<  Send address to device
	{
		Stop();
		return ERR_PROCESS_FAILED;
	}

	for(i=0;i<nums;i++)
	{
        if (Write(data_out[i]))			                   //!<  Send byte to device
	    {
		    Stop();
		    return ERR_PROCESS_FAILED;
	    }
	}	
	
	Stop();

	//need to delay some time
	for(j=0;j<5;j++)
	{
		for (i = 0; i < 0x4000; i++) 
		{ 
			__asm("nop;"); 
		}	//!<  Delay
	}	

	return ERR_NONE;
}



/* --------------------------------------------------------------------------*/
/**
* @name I2C_Read
* @brief 
* @details Read operation from AT24Cxxxx
*
* @param[in] address Address ready to read
* @param[in] data_in Data pointer to store data read from AT24Cxxxx
* @param[in] nums Number of data read from AT24Cxxxx
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 I2C_Read(u32 address, u8 *data_in, u16 nums)
{
    u8 addr0,addr1;
	u16 i;

	if((address&0x00010000)!=0)                    //!< 检查address的bit16是否为1,bit16为Page addr
	{
        addr0 = AT24C1024_WRITE_ADDR|0x02;         //!< 向i2c写入即将要读的起始地址
        addr1 = AT24C1024_READ_ADDR|0x02;
	}		
	else
	{
		addr0 = AT24C1024_WRITE_ADDR;
		addr1 = AT24C1024_READ_ADDR;
	}

	Start();							           //!<  Send start signal
	if(Write(addr0))				               //!<  Send device address
	{
		Stop();							           //!<  Send I2C Stop Transfer
		return ERR_PROCESS_FAILED;                                  //!<  return false
	}
	if(Write((u8)(address>>8)))				   //!<  Send address high to device
	{
		Stop();							           
		return ERR_PROCESS_FAILED;
	}

	if(Write((u8)address))				       //!<  Send address low to device
	{
		Stop();							           
		return ERR_PROCESS_FAILED;
	}          
	
	Start();							      //!<  Send I2C Start Transer
	if (Write(addr1))			              //!<  Send  I2C read address
	{
		Stop();							      //!<  Send I2C Stop Transfer
		return ERR_PROCESS_FAILED;
	}

	for(i=0;i<(nums-1);i++)
	{
        if (Read(&data_in[i], 1))			  //!<  Read byte
	    {
		    Stop();							  //!<  Send I2C Stop Transfer
		    return ERR_PROCESS_FAILED;
	    }
	}

	if (Read(&data_in[(nums-1)], 0))		 //!<  Read last byte,does't send 
	{
		Stop();
		return ERR_PROCESS_FAILED;
	}
		
	Stop();								//!<  Send I2C Stop Transfer

	return ERR_NONE;
}


/* --------------------------------------------------------------------------*/
/**
* @name Start
* @brief 
* @details Sends I2C Start Trasfer - "S" to AT24Cxxxx
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void Start(void)
{
	SetSCLK(1);					//!<  Set SCLK high
	SetSDATA(0);	  	        //!<  Set SDATA output/low
	SetSCLK(0);					//!<  Set SCLK low
}



/* --------------------------------------------------------------------------*/
/**
* @name Stop
* @brief 
* @details Sends I2C Stop Trasfer - "P" to AT24Cxxxx
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void Stop(void)
{
	SetSDATA(0);				//!<  Set SDATA output/low
	SetSCLK(1);				    //!<  Set SCLK high
	SetSDATA(1);				//!<  Set SDATA as input/high
}


/* --------------------------------------------------------------------------*/
/**
* @name SendACK
* @brief 
* @details Sends I2C ACK Trasfer - "A" To AT24Cxxxx
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void SendACK(void)
{
	SetSDATA(0);			
	SetSCLK(1);				  
	SetSCLK(0);				
}


/* --------------------------------------------------------------------------*/
/**
* @name Write
* @brief 
* @details Writes data over the I2C bus and return status.
*
* @param[in] data_out
*
* @returns 0 Receive ACK
*          1 Receive NACK
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 Write(u8 data_out)
{
	u8 index;

	for(index = 0; index < 8; index++)
	{
		//!<  Output the data bit to the device
		SetSDATA(((data_out & 0x80) ? 1 : 0));

		data_out <<= 1;						//!<  Shift the byte by one bit
		SetSCLK(1);							//!<  Set SCLK high
		SetSCLK(0);							//!<  Set SCLK low
	}

	SetSDATA(1);							//!<  Set SDATA input/high
	SetSCLK(1);								//!<  Set SCLK high

	if (!GetSDATA())
	{
		SetSCLK(0);							//!<  Set SCLK low
		return ERR_NONE;						    //!<  ACK from slave
	} else
	{
		SetSCLK(0);							//!<  Set SCLK low
		return ERR_PROCESS_FAILED;						    //!<  NACK from slave
	}
}




/* --------------------------------------------------------------------------*/
/**
* @name Read
* @brief 
* @details Reads data from the I2C bus and return it in data_in Returns status.
*
* @param[in] data_in
* @param[in] send_ack If true send the ACK signal else send NACK
*
* @returns ERR_NONE successful
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 Read(u8 *data_in, u8 send_ack)
{
	u8 index;

	*data_in = 0x00;

	SetSDATA(1);							//!<  Set SDATA input/high
	SetSCLK(0);								//!<  Set SCLK low

	//!<  Get 8 bits from the device
	for(index = 0; index < 8; index++)
	{
		*data_in <<= 1;						//!<  Shift the data right 1 bit
		SetSCLK(1);							//!<  Set SCLK high
							//!<  Set SCLK low
		*data_in |= GetSDATA();				//!<  Read the data bit

		SetSCLK(0);							//!<  Set SCLK low
	}

	if (send_ack)
		SetSDATA(0);		//!<  Set data pin to output/low to ACK the read
	else
		SetSDATA(1);		//!<  Set data pin to input/high to NACK the read

	SetSCLK(1);								//!<  Set SCLK high
	SetSCLK(0);								//!<  Set SCLK low
	SetSDATA(0);							//!<  Set SDATA output/low
	SetSDATA(1);							//!<  Set SDATA input/high

	return ERR_NONE;
}



/* --------------------------------------------------------------------------*/
/**
* @name SetSDATA
* @brief 
* @details Set the I2C port SDATA pin to <state>.
*
* @param[in] state
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void SetSDATA(u8 state)
{
	u32 i;	
	if (state)
	{
		GPIOF->BSRRL = GPIO_Pin_0;				//!<  Set SDATA.
	} else
	{
		GPIOF->BSRRH = GPIO_Pin_0 ;             //!<  reset SDA		
	}	
	for (i = 0; i < 0x90; i++) { __asm("nop;"); }   //!<  Delay
}



/* --------------------------------------------------------------------------*/
/**
* @name SetSCLK
* @brief 
* @details Set the I2C port SCLK pin to <state>.
*
* @param[in] state
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void SetSCLK(u8 state)
{
    u32 i;
	
	if (state)
	{
		GPIOF->BSRRL = GPIO_Pin_1 ;    	//!<  Set SCLK high.

	} else
	{
		GPIOF->BSRRH = GPIO_Pin_1 ;    //!<  Set SCLK low.
	}
	for (i = 0; i < 0x90; i++) 
	{ 		
		__asm("nop;"); 
	}
}


/* --------------------------------------------------------------------------*/
/**
* @name GetSDATA
* @brief 
* @details Get the I2C port SDATA pin state.
*
* @param[in] state
*
* @returns state of I2C pin SDA
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u8 GetSDATA(void)
{	
	return (( GPIOF->IDR & GPIO_Pin_0) ? 1 : 0);
}

//===============================================================================


/* --------------------------------------------------------------------------*/
/**
* @name I2C1_Write
* @brief Write data to IMU
* @details Writes bytes to the given address and return status.
*
* @param[in] address Write address
* @param[in] data_out Pointer of Data to be written into IMU Module
* @param[in] nums Number of data be written
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 I2C1_Write(u8 regAddr, u8 *data_out, u8 nums)
{
    u8 num = (nums<<1);
	u16 i;
	
	I2C1Start();							   //!<  Send start signal
	if (I2C1Write(IMU_JY901_WRITE_ADDR))				
	{
		I2C1Stop();							
		return ERR_PROCESS_FAILED;
	}

	if (I2C1Write(regAddr))	         //!<  Send address to device
	{
		I2C1Stop();
		return ERR_PROCESS_FAILED;
	}

	for(i=0;i<num;i++)
	{
        if (I2C1Write(data_out[i]))			//!<  Send byte to device
	    {
		    I2C1Stop();
		    return ERR_PROCESS_FAILED;
	    }
	}	
	
	I2C1Stop();

	return ERR_NONE;
}



/* --------------------------------------------------------------------------*/
/**
* @name I2C1_Read
* @brief 
* @details Read operation from AT24Cxxxx
*
* @param[in] regAddr Register address ready to read
* @param[in] data_in Data pointer to store data read from IMU Module
* @param[in] nums Number of data read from IMU Module
*
* @returns 0 successful
*          1 failed
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 I2C1_Read(u8 regAddr, u8 *data_in, u16 nums)
{
	u16 i;
	u8 num=(nums<<1);

	I2C1Start();							           //!<  Send start signal
	if(I2C1Write(IMU_JY901_WRITE_ADDR))				               //!<  Send device address
	{
		I2C1Stop();							           //!<  Send I2C Stop Transfer
		return ERR_PROCESS_FAILED;                                      //!<  return false
	}

	if(I2C1Write(regAddr))				               //!<  Send address low to device
	{
		I2C1Stop();							           
		return ERR_PROCESS_FAILED;
	}          
	
	I2C1Start();							      //!<  Send I2C Start Transer
	if (I2C1Write(IMU_JY901_READ_ADDR))			              //!<  Send  I2C read address
	{
		I2C1Stop();							      //!<  Send I2C Stop Transfer
		return ERR_PROCESS_FAILED;
	}

	for(i=0;i<(num-1);i++)
	{
        if (I2C1Read(&data_in[i], 1))			  //!<  Read byte
	    {
		    I2C1Stop();							  //!<  Send I2C Stop Transfer
		    return ERR_PROCESS_FAILED;
	    }
	}

	if (I2C1Read(&data_in[(num-1)], 0))		 //!<  Read last byte,does't send 
	{
		I2C1Stop();
		return ERR_PROCESS_FAILED;
	}
		
	I2C1Stop();								//!<  Send I2C Stop Transfer

	return ERR_NONE;
}


/* --------------------------------------------------------------------------*/
/**
* @name Start
* @brief 
* @details Sends I2C Start Trasfer - "S" to IMU
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void I2C1Start(void)
{
	I2C1SetSCLK(1);					//!<  Set SCLK high
	I2C1SetSDATA(0);	  	        //!<  Set SDATA output/low
	I2C1SetSCLK(0);					//!<  Set SCLK low
}


/* --------------------------------------------------------------------------*/
/**
* @name Stop
* @brief 
* @details Sends I2C Stop Trasfer - "P" to IMU
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void I2C1Stop(void)
{
	I2C1SetSDATA(0);				//!<  Set SDATA output/low
	I2C1SetSCLK(1);				    //!<  Set SCLK high
	I2C1SetSDATA(1);				//!<  Set SDATA as input/high
}


/* --------------------------------------------------------------------------*/
/**
* @name SendACK
* @brief 
* @details Sends I2C ACK Trasfer - "A" To IMU Module
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void I2C1SendACK(void)
{
	I2C1SetSDATA(0);			
	I2C1SetSCLK(1);				  
	I2C1SetSCLK(0);				
}


/* --------------------------------------------------------------------------*/
/**
* @name Write
* @brief 
* @details Writes data over the I2C bus and return status.
*
* @param[in] data_out
*
* @returns 0 Receive ACK
*          1 Receive NACK
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 I2C1Write(u8 data_out)
{
	u8 index;

	for(index = 0; index < 8; index++)
	{
		//!<  Output the data bit to the device
		I2C1SetSDATA(((data_out & 0x80) ? 1 : 0));

		data_out <<= 1;						//!<  Shift the byte by one bit
		I2C1SetSCLK(1);							//!<  Set SCLK high
		I2C1SetSCLK(0);							//!<  Set SCLK low
	}

	I2C1SetSDATA(1);							//!<  Set SDATA input/high
	I2C1SetSCLK(1);								//!<  Set SCLK high

	if (!I2C1GetSDATA())
	{
		I2C1SetSCLK(0);							//!<  Set SCLK low
		return ERR_NONE;						//!<  ACK from slave
	} else
	{
		I2C1SetSCLK(0);							//!<  Set SCLK low
		return ERR_PROCESS_FAILED;						//!<  NACK from slave
	}
}



/* --------------------------------------------------------------------------*/
/**
* @name Read
* @brief 
* @details Reads data from the I2C bus and return it in data_in Returns status.
*
* @param[in] data_in
* @param[in] send_ack If true send the ACK signal else send NACK
*
* @returns ERR_NONE successful
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 I2C1Read(u8 *data_in, u8 send_ack)
{
	u8 index;

	*data_in = 0x00;

	I2C1SetSDATA(1);							//!<  Set SDATA input/high
	I2C1SetSCLK(0);								//!<  Set SCLK low

	//!<  Get 8 bits from the device
	for(index = 0; index < 8; index++)
	{
		*data_in <<= 1;						//!<  Shift the data right 1 bit
		I2C1SetSCLK(1);							//!<  Set SCLK high
							//!<  Set SCLK low
		*data_in |= I2C1GetSDATA();				//!<  Read the data bit

		I2C1SetSCLK(0);							//!<  Set SCLK low
	}

	if (send_ack)
		I2C1SetSDATA(0);		//!<  Set data pin to output/low to ACK the read
	else
		I2C1SetSDATA(1);		//!<  Set data pin to input/high to NACK the read

	I2C1SetSCLK(1);								//!<  Set SCLK high
	I2C1SetSCLK(0);								//!<  Set SCLK low
	I2C1SetSDATA(0);							//!<  Set SDATA output/low
	I2C1SetSDATA(1);							//!<  Set SDATA input/high

	return ERR_NONE;
}



/* --------------------------------------------------------------------------*/
/**
* @name SetSDATA
* @brief 
* @details Set the I2C port SDATA pin to <state>.
*
* @param[in] state
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void I2C1SetSDATA(u8 state)
{
	u32 i;	
	if (state)
	{
		GPIOB->BSRRL = GPIO_Pin_9;				//!<  Set SDATA.
	} else
	{
		GPIOB->BSRRH = GPIO_Pin_9;             //!<  reset SDA		
	}	
	for (i = 0; i < 0x30; i++) { __asm("nop;"); }   //!<  Delay
}


/* --------------------------------------------------------------------------*/
/**
* @name SetSCLK
* @brief 
* @details Set the I2C port SCLK pin to <state>.
*
* @param[in] state
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void I2C1SetSCLK(u8 state)
{
    u32 i;
	
	if (state)
	{
		GPIOB->BSRRL = GPIO_Pin_8 ;    	//!<  Set SCLK high.

	} else
	{
		GPIOB->BSRRH = GPIO_Pin_8 ;     //!<  Set SCLK low.
	}
	for (i = 0; i < 0x30; i++) 
	{ 		
		__asm("nop;"); 
	}	

}



/* --------------------------------------------------------------------------*/
/**
* @name I2C1GetSDATA
* @brief 
* @details Get the I2C port SDATA pin state.
*
* @param[in] state
*
* @returns state of I2C pin SDA
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u8 I2C1GetSDATA(void)
{	
	return (( GPIOB->IDR & GPIO_Pin_9) ? 1 : 0);
}


/* --------------------------------------------------------------------------*/
/**
* @name delay
* @brief 
* @details 
*
* @param[in] num Number of delay
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void delay(u32 num)
{
    u32 i;
    for(i=0;i<(num);i++)
    { 
    	__asm("nop;"); 
    }
}


/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file bsp_spi.c
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



/* Private variables ---------------------------------------------------------*/
MSD_CARDINFO SD0_CardInfo;



void MSD0_SPIHighSpeed(u8 b_high);
u8 SDCardCmd_9_10_Send(u8 cmd, u8 *buff, u16 len);
u16 MSD0_send_command(u8 cmd, u32 arg, u8 crc);
u16 MSD0_spi_read_write(u8 data);
u8 MSD0_read_buffer(u8 *buff, u16 len, u8 release);
u16 MSD0_send_command_hold(u8 cmd, u32 arg, u8 crc);





/* --------------------------------------------------------------------------*/
/**
* @name BSPSpi2_Init
* @brief 
* @details SD Card SPI Configuration.
*
* @param[in] None
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void BspSpi2_Init(void)
{		
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;          //!< SPI MOSI Pin config
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;         //!< SPI MISO Pin config
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
    
  MSD0_card_disable();                               //!< 关闭片选  
  MSD0_SPIHighSpeed(1);                              //!< 设置SPI接口
  
  SPI_Cmd(SPI2, ENABLE);                             //!< 使能SPI模块
}


/* --------------------------------------------------------------------------*/
/**
* @name MSD0_SPIHighSpeed
* @brief 
* @details SD Card Speed Set.
*
* @param[in] b_hith  1 = 18MHz, 0 = 281.25Hz
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
void MSD0_SPIHighSpeed(u8 b_high)
{
    SPI_InitTypeDef SPI_InitStructure;

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    if(b_high == 0)                                        //!< Speed select 
    {
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    }
    else
    {
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    }

    SPI_Init(SPI2, &SPI_InitStructure);
}

/* --------------------------------------------------------------------------*/
/**
* @name SDCardCmd_9_10_Send
* @brief 
* @details Send CMD9 or CMD10 to SD Card.
*
* @param[in] cmd CMD9 or CMD10
* @param[in] buff Pointer to store CSD or CID
* @param[in] len Sending data length
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u8 SDCardCmd_9_10_Send(u8 cmd, u8 *buff, u16 len)
{
	u8 tmp=DUMMY_BYTE; 
	u8 retry;
	u8 Dat[6]={0};
	
	Dat[0] = cmd|0x40;
	Dat[1] = 0;
	Dat[2] = 0;
	Dat[3] = 0;
	Dat[4] = 0;
	Dat[5] = 0;

	MSD0_spi_read_write(DUMMY_BYTE);

	MSD0_card_enable();                                    //!< pin reset,chip select		
	
    MSD0_spi_read_write(Dat[0]);                           //!< ==== Command, argument and crc====
    MSD0_spi_read_write(Dat[1]);
    MSD0_spi_read_write(Dat[2]);
    MSD0_spi_read_write(Dat[3]);
    MSD0_spi_read_write(Dat[4]);
    MSD0_spi_read_write(Dat[5]);	
	
	for(retry=0; retry<200; retry++)                       //!< ====Wait response, quit till timeout====
	{
		tmp = MSD0_spi_read_write(DUMMY_BYTE);		
		if(tmp==0x00)
		{				 
			break;
		}
	}

	tmp = DUMMY_BYTE;    
	for(retry=0; retry<200; retry++)                       //!<  Wait start-token 0xFE 
	{
        tmp = MSD0_spi_read_write(DUMMY_BYTE);
        if(tmp == SD_START_BLOCK_TOKEN_1)
        {
            retry = 0;
            break;
		 }
    }
	    
    if(retry == 200)                                       //!< ====Timeout return====
    {
        MSD0_card_disable();                               //!< pin set        
        return 1;
    }	
	
    for(retry=0; retry<len; retry++)                       //!< ====Start reading====
    {
        *(buff+retry) = MSD0_spi_read_write(DUMMY_BYTE);
    }	
	
	MSD0_spi_read_write(DUMMY_BYTE);                       //!< ====2bytes dummy CRC====
    MSD0_spi_read_write(DUMMY_BYTE);
		
	MSD0_card_disable();                                   //!< pin set 
		
	MSD0_spi_read_write(DUMMY_BYTE);                       //!< send dummy byte
    MSD0_spi_read_write(DUMMY_BYTE);                       //!< send dummy byte
	MSD0_spi_read_write(DUMMY_BYTE);                       //!< send dummy byte
    MSD0_spi_read_write(DUMMY_BYTE);                       //!< send dummy byte
	
	return 0;
}

/* --------------------------------------------------------------------------*/
/**
* @name MSD0_spi_read_write
* @brief 
* @details Write data to SPI interface and Read data from SPI interface.
*
* @param[in] data Written to spi line
*
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u16 MSD0_spi_read_write(u8 data)
{    
  //!< Loop while DR register in not emplty  
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  //!< Send byte through the SPI1 peripheral  
  SPI_I2S_SendData(SPI2, data);

  //!< Wait to receive a byte  
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  //! < Return the byte read from the SPI bus  
  return SPI_I2S_ReceiveData(SPI2);
  
}

/* --------------------------------------------------------------------------*/
/**
* @name MSD0_read_buffer
* @brief 
* @details Read data form spi interface.
*
* @param[in] buff Pointer of data read from spi line
* @param[in] len Length of data to read
* @param[in] release After read data,release the chip select or not
*                    1:release,CS Pin is Set 0: not release,CS Pin is Reset
* @returns 0 successful
*          1 failed
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u8 MSD0_read_buffer(u8 *buff, u16 len, u8 release)
{
  u8 r1;
  register u16 retry;
  
  MSD0_card_enable();                 //!< Card enable, Prepare to read	
  
  for(retry=0; retry<2000; retry++)   //!< Wait start-token 0xFE 
  {
	 r1 = MSD0_spi_read_write(DUMMY_BYTE);
	 
	 if(r1 == 0xFE)
	 {
		 retry = 0;
		 break;
	 }
  }
  
  if(retry == 2000)                  //!< Timeout return	
  {
	 MSD0_card_disable();
	 
	 return 1;
  }
  
  for(retry=0; retry<len; retry++)  //!< Start reading 
  {
     *(buff+retry) = MSD0_spi_read_write(DUMMY_BYTE);
  }
  
  MSD0_spi_read_write(DUMMY_BYTE); //!< 2bytes dummy CRC 
  MSD0_spi_read_write(DUMMY_BYTE);
  
  if(release)                      //!< chip disable and dummy byte 
  {
	 MSD0_card_disable();
	 
	 MSD0_spi_read_write(DUMMY_BYTE);
	 MSD0_spi_read_write(DUMMY_BYTE);
	 MSD0_spi_read_write(DUMMY_BYTE);
	 MSD0_spi_read_write(DUMMY_BYTE);
  }

  return 0;
}


/* --------------------------------------------------------------------------*/
/**
* @name MSD0_send_command
* @brief 
* @details Send command to sd card according to communication protocol.
*
* @param[in] cmd Command sending to sd card
* @param[in] arg Argument of Command sending to sd card
* @param[in] crc CRC of Command
* 
* @returns R1 value, response from card
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u16 MSD0_send_command(u8 cmd, u32 arg, u8 crc)
{
  u8 r1;
  u8 retry;
  
  MSD0_spi_read_write(DUMMY_BYTE);     //!< Dummy byte and chip enable 
  MSD0_card_enable();
  
  MSD0_spi_read_write(cmd | 0x40);     //!< Command, argument and crc 
  MSD0_spi_read_write(arg >> 24);
  MSD0_spi_read_write(arg >> 16);
  MSD0_spi_read_write(arg >> 8);
  MSD0_spi_read_write(arg);
  MSD0_spi_read_write(crc);
    
  for(retry=0; retry<200; retry++)    //!< Wait response, quit till timeout 
  {
	 r1 = MSD0_spi_read_write(DUMMY_BYTE);
	 if(r1 != 0xFF)
	 {
		 break;
	 }
  }
  
  MSD0_card_disable();               //!< Chip disable and dummy byte 
  MSD0_spi_read_write(DUMMY_BYTE);

  return r1;
}	

/* --------------------------------------------------------------------------*/
/**
* @name MSD0_send_command_hold
* @brief 
* @details Sending Command to SD Card and don't release the CS Line
*
* @param[in] cmd Command sending to sd card
* @param[in] arg Argument of Command sending to sd card
* @param[in] crc CRC of Command
* 
* @returns R1 value, response from card
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u16 MSD0_send_command_hold(u8 cmd, u32 arg, u8 crc)
{
  u8 r1;
  u8 retry;
  
  MSD0_spi_read_write(DUMMY_BYTE);          //!< Dummy byte and chip enable 
  MSD0_card_enable();
  
  MSD0_spi_read_write(cmd | 0x40);          //!< Command, argument and crc 
  MSD0_spi_read_write(arg >> 24);
  MSD0_spi_read_write(arg >> 16);
  MSD0_spi_read_write(arg >> 8);
  MSD0_spi_read_write(arg);
  MSD0_spi_read_write(crc);
    
  for(retry=0; retry<200; retry++)          //!< Wait response, quit till timeout 
  {
	 r1 = MSD0_spi_read_write(DUMMY_BYTE);
	 if(r1 != 0xFF)
	 {
		 break;
	 }
  }

  return r1;
}

/* --------------------------------------------------------------------------*/
/**
* @name SDCard_Init
* @brief 
* @details Initialize the SD Card interface
*
* @param[in] None
* 
* @returns None
* 
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u8 SDCard_Init(void)
{
    u8 r1;	
    u8 buff[6] = {0};
    u16 retry; 
    
    for(retry=0; retry<20; retry++)                     //!< Satrt send 74 clocks at least 
    {
        MSD0_spi_read_write(DUMMY_BYTE);
    }	
	
    MSD0_card_enable();
    
    for(retry=0; retry<0xFFF; retry++)                 //!< Start send CMD0 till return 0x01 means in IDLE state 
    {
        r1 = MSD0_send_command(CMD0, 0, 0x95);
        if(r1 == 0x01)
        {
            retry = 0;
            break;
        }
    }
    
    if(retry == 0xFFF)                                //!< Timeout return 
    {
#if PRINT_SD_DEBUG_INFO 
        Bsp_printf("Reset card into IDLE state failed!\r\n");
#endif
        return 1;
    }	
    
    r1 = MSD0_send_command_hold(CMD8, 0x1AA, 0x87);  //!< Get the card type, version 
    
    if(r1 == 0x05)                                   //!< r1=0x05 -> V1.0 
    {
        SD0_CardInfo.CardType = CARDTYPE_SDV1;
	
		MSD0_card_disable();                         //!< End of CMD8, chip disable and dummy byte 
		MSD0_spi_read_write(DUMMY_BYTE);
		
	    //!< SD1.0/MMC start initialize
	    //!< Send CMD55+ACMD41, No-response is a MMC card, otherwise is a SD1.0 card 
		for(retry=0; retry<0xFFF; retry++)
        {
           r1 = MSD0_send_command(CMD55, 0, 0);		 //!< should be return 0x01
           if(r1 != 0x01)
           {
#if PRINT_SD_DEBUG_INFO 
                Bsp_printf("Send CMD55 should return 0x01, response=0x%02x\r\n", r1);
#endif
                return r1;
           }
		   
           r1 = MSD0_send_command(ACMD41, 0, 0);	 //!< should be return 0x00 
           
           if(r1 == 0x00)
           {
                retry = 0;
                break;
           }
        }
	
		if(retry == 0xFFF)                          //!< MMC card initialize start 
		{
            for(retry=0; retry<0xFFF; retry++)
	    	{
                r1 = MSD0_send_command(CMD1, 0, 0);	//!< should be return 0x00 
                
                if(r1 == 0x00)
                {
                    retry = 0;
                    break;
                }
            }
             
            if(retry == 0xFFF)                     //!< Timeout return
            {
#if PRINT_SD_DEBUG_INFO
                Bsp_printf("Send CMD1 should return 0x00, response=0x%02x\r\n", r1);
#endif
                return 2;
            }	

            SD0_CardInfo.CardType = CARDTYPE_MMC;	
			
#if PRINT_SD_DEBUG_INFO 
            Bsp_printf("Card Type                     : MMC\r\n");
#endif 				
	    }        
#if PRINT_SD_DEBUG_INFO                         //!< SD1.0 card detected, print information 
        else
        {
            Bsp_printf("Card Type                     : SD V1\r\n");
        }
#endif 		
        
        MSD0_SPIHighSpeed(1);		              //!< Set spi speed high               
        
        r1 = MSD0_send_command(CMD59, 0, 0x01);   //!< CRC disable 
        if(r1 != 0x00)
		{
#if PRINT_SD_DEBUG_INFO 
        	Bsp_printf("Send CMD59 should return 0x00, response=0x%02x\r\n", r1);
#endif
              return r1;		                 //!< response error, return r1 
		}
        
        r1 = MSD0_send_command(CMD16, MSD_BLOCKSIZE, 0xFF);//!< Set the block size 
        
        if(r1 != 0x00)
        {
#if PRINT_SD_DEBUG_INFO
            Bsp_printf("Send CMD16 should return 0x00, response=0x%02x\r\n", r1);
#endif
            return r1;		                    //!< response error, return r1 
		}
   }	   
   else if(r1 == 0x01)                          //!< r1=0x01 -> V2.x, read OCR register, check version 
   {    //!< 4Bytes returned after CMD8 sent	
		buff[0] = MSD0_spi_read_write(DUMMY_BYTE);				//!< should be 0x00 
		buff[1] = MSD0_spi_read_write(DUMMY_BYTE);				//!< should be 0x00 
		buff[2] = MSD0_spi_read_write(DUMMY_BYTE);				//!< should be 0x01 
		buff[3] = MSD0_spi_read_write(DUMMY_BYTE);				//!< should be 0xAA 
		
		//!< End of CMD8, chip disable and dummy byte 
		MSD0_card_disable();
		MSD0_spi_read_write(DUMMY_BYTE);
		
		if(buff[2]==0x01 && buff[3]==0xAA)     //!< Check voltage range be 2.7-3.6V
		{
            for(retry=0; retry<0xFFF; retry++)
            {
                r1 = MSD0_send_command(CMD55, 0, 0);			//!< should be return 0x01 

				if(r1!=0x01)
                {
#if PRINT_SD_DEBUG_INFO 
                    Bsp_printf("Send CMD55 should return 0x01, response=0x%02x\r\n", r1);
#endif
                    return r1;
                }				

                r1 = MSD0_send_command(ACMD41, 0x40000000, 0);	//!< should be return 0x00 

				if(r1 == 0x00)
                {
                	retry = 0;
                    break;
                }
            } 		 	
            
            if(retry == 0xFFF)                                 //!< Timeout return 
            {
#if PRINT_SD_DEBUG_INFO
                Bsp_printf("Send ACMD41 should return 0x00, response=0x%02x\r\n", r1);
#endif
                return 3;
            }
            
			r1 = MSD0_send_command_hold(CMD58, 0, 0);         //!< Read OCR by CMD58 

			if(r1!=0x00)
	    	{
#if PRINT_SD_DEBUG_INFO
				Bsp_printf("Send CMD58 should return 0x00, response=0x%02x\r\n", r1);
#endif
                return r1;		                             //!< response error, return r1 
			}

			buff[0] = MSD0_spi_read_write(DUMMY_BYTE);					
            buff[1] = MSD0_spi_read_write(DUMMY_BYTE);					
            buff[2] = MSD0_spi_read_write(DUMMY_BYTE);					
            buff[3] = MSD0_spi_read_write(DUMMY_BYTE);
            
            MSD0_card_disable();                            //!< End of CMD58, chip disable and dummy byte 
            MSD0_spi_read_write(DUMMY_BYTE);
	    
			if(buff[0] & 0x40)                              //!< OCR -> CCS(bit30)  1: SDV2HC	 0: SDV2 
			{
                SD0_CardInfo.CardType = CARDTYPE_SDV2HC;
#if PRINT_SD_DEBUG_INFO 
				Bsp_printf("Card Type                     : SD V2HC\r\n");
#endif 	
			}
	    	else
	    	{
                SD0_CardInfo.CardType = CARDTYPE_SDV2;
#if PRINT_SD_DEBUG_INFO
				Bsp_printf("Card Type                     : SD V2\r\n");
#endif 	
			}
            
            MSD0_SPIHighSpeed(1);                         //!< Set spi speed high 
        }	
   }
   return 0;
}


/* --------------------------------------------------------------------------*/
/**
* @name SDCard_GetCardInfo
* @brief 
* @details Sending CMD9&CMD10 to SD Card to get the Card Infomation
*
* @param[in] SD0_CardInfo Infomation struct of SD Card
* 
* @returns 0 successful
*          1 failed
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u8 SDCard_GetCardInfo(PMSD_CARDINFO SD0_CardInfo)
{
	u8 CSD_Tab[16]={0};
    u8 CID_Tab[16]={0};
	u8 rt,Res;

    rt = SDCardCmd_9_10_Send(CMD9, CSD_Tab, 16);

	if(rt!=0)
	{
#if PRINT_SD_DEBUG_INFO
		Bsp_printf("Send CMD9 return timeout!\r\n");
#endif
        return 1;
	}

	if(Res!=0x00)
	{
#if PRINT_SD_DEBUG_INFO
		Bsp_printf("Send CMD9 should return 0x00, response=0x%02x\r\n",Res);
#endif
        return 1;
	}
	
    rt = SDCardCmd_9_10_Send(CMD10, CID_Tab, 16);          //!< ====Send CMD10, Read CID ====
    if(rt != 0x00)
    {
#if PRINT_SD_DEBUG_INFO
        Bsp_printf("Send CMD10 return timeout!\r\n");
#endif
        return rt;
    }
	if(Res!=0x00)                                          //!< CMD10 return R1=0x00
		{
#if PRINT_SD_DEBUG_INFO
		Bsp_printf("Send CMD10 should return 0x00, response=0x%02x\r\n",Res);
#endif
		return 1;
	}
	
    SD0_CardInfo->CSD.CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;             //!<  Byte 0 
    SD0_CardInfo->CSD.SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
    SD0_CardInfo->CSD.Reserved1 = CSD_Tab[0] & 0x03;
    
    SD0_CardInfo->CSD.TAAC = CSD_Tab[1] ;                               //!<  Byte 1 
    
    SD0_CardInfo->CSD.NSAC = CSD_Tab[2];                                //!<  Byte 2 
  
    SD0_CardInfo->CSD.MaxBusClkFrec = CSD_Tab[3];                       //!<  Byte 3 
  
    SD0_CardInfo->CSD.CardComdClasses = CSD_Tab[4] << 4;                //!<  Byte 4 
  
    SD0_CardInfo->CSD.CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;      //!<  Byte 5 
    SD0_CardInfo->CSD.RdBlockLen = CSD_Tab[5] & 0x0F;
  
    SD0_CardInfo->CSD.PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;         //!<  Byte 6 
    SD0_CardInfo->CSD.WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
    SD0_CardInfo->CSD.RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
    SD0_CardInfo->CSD.DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
    SD0_CardInfo->CSD.Reserved2 = 0;                                    //!<  Reserved 
    SD0_CardInfo->CSD.DeviceSize = (CSD_Tab[6] & 0x03) << 10;
  
    SD0_CardInfo->CSD.DeviceSize |= (CSD_Tab[7]) << 2;                  //!<  Byte 7 
  
    SD0_CardInfo->CSD.DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;           //!<  Byte 8 
    SD0_CardInfo->CSD.MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
    SD0_CardInfo->CSD.MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);
  
    SD0_CardInfo->CSD.MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;    //!<  Byte 9 
    SD0_CardInfo->CSD.MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
    SD0_CardInfo->CSD.DeviceSizeMul = (CSD_Tab[9] & 0x03) << 1;
  
    SD0_CardInfo->CSD.DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;       //!<  Byte 10 
    SD0_CardInfo->CSD.EraseGrSize = (CSD_Tab[10] & 0x7C) >> 2;
    SD0_CardInfo->CSD.EraseGrMul = (CSD_Tab[10] & 0x03) << 3;
  
    SD0_CardInfo->CSD.EraseGrMul |= (CSD_Tab[11] & 0xE0) >> 5;          //!<  Byte 11 
    SD0_CardInfo->CSD.WrProtectGrSize = (CSD_Tab[11] & 0x1F);
  
    SD0_CardInfo->CSD.WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;    //!<  Byte 12 
    SD0_CardInfo->CSD.ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
    SD0_CardInfo->CSD.WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
    SD0_CardInfo->CSD.MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;
  
    SD0_CardInfo->CSD.MaxWrBlockLen |= (CSD_Tab[13] & 0xc0) >> 6;       //!<  Byte 13 
    SD0_CardInfo->CSD.WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
    SD0_CardInfo->CSD.Reserved3 = 0;
    SD0_CardInfo->CSD.ContentProtectAppli = (CSD_Tab[13] & 0x01);
  
    SD0_CardInfo->CSD.FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;     //!<  Byte 14 
    SD0_CardInfo->CSD.CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
    SD0_CardInfo->CSD.PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
    SD0_CardInfo->CSD.TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
    SD0_CardInfo->CSD.FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
    SD0_CardInfo->CSD.ECC = (CSD_Tab[14] & 0x03);
  
    SD0_CardInfo->CSD.CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;              //!<  Byte 15 
    SD0_CardInfo->CSD.Reserved4 = 1;

    if(SD0_CardInfo->CardType == CARDTYPE_SDV2HC)
    {                                                                     //!<  Byte 7-8-9
        SD0_CardInfo->CSD.DeviceSize = ((CSD_Tab[7]&0x3F)<<16)|(CSD_Tab[8]<<8)|(CSD_Tab[9]);		
    }

    SD0_CardInfo->Capacity = (SD0_CardInfo->CSD.DeviceSize+1) * SD_BLOCKSIZE * 1024;
    SD0_CardInfo->BlockSize = SD_BLOCKSIZE;
  
    SD0_CardInfo->CID.ManufacturerID = CID_Tab[0];                      //!<  Byte 0 
  
    SD0_CardInfo->CID.OEM_AppliID = CID_Tab[1] << 8;                    //!<  Byte 1 
  
    SD0_CardInfo->CID.OEM_AppliID |= CID_Tab[2];                        //!<  Byte 2 
  
    SD0_CardInfo->CID.ProdName1 = CID_Tab[3] << 24;                     //!<  Byte 3 
  
    SD0_CardInfo->CID.ProdName1 |= CID_Tab[4] << 16;                    //!<  Byte 4 
  
    SD0_CardInfo->CID.ProdName1 |= CID_Tab[5] << 8;                     //!<  Byte 5 
  
    SD0_CardInfo->CID.ProdName1 |= CID_Tab[6];                          //!<  Byte 6 
  
    SD0_CardInfo->CID.ProdName2 = CID_Tab[7];                           //!<  Byte 7 
  
    SD0_CardInfo->CID.ProdRev = CID_Tab[8];                             //!<  Byte 8 
  
    SD0_CardInfo->CID.ProdSN = CID_Tab[9] << 24;                        //!<  Byte 9 
  
    SD0_CardInfo->CID.ProdSN |= CID_Tab[10] << 16;                      //!<  Byte 10 
  
    SD0_CardInfo->CID.ProdSN |= CID_Tab[11] << 8;                       //!<  Byte 11 
  
    SD0_CardInfo->CID.ProdSN |= CID_Tab[12];                            //!<  Byte 12 
  
    SD0_CardInfo->CID.Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;           //!<  Byte 13 
  
    SD0_CardInfo->CID.ManufactDate = (CID_Tab[13] & 0x0F) << 8;         //!<  Byte 14 
  
    SD0_CardInfo->CID.ManufactDate |= CID_Tab[14];                      //!<  Byte 15 
  
    SD0_CardInfo->CID.CID_CRC = (CID_Tab[15] & 0xFE) >> 1;              //!<  Byte 16 
  
    SD0_CardInfo->CID.Reserved2 = 1;

    return 0;  
}


/* --------------------------------------------------------------------------*/
/**
* @name SDCard_ReadSingleBlock
* @brief 
* @details Launch timing to read a single block from SD Card
*
* @param[in] sector No.of sector to read
* @param[in] buffer Pointer to store read data
*
* @returns 0 successful
*          1 failed
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u8 SDCard_ReadSingleBlock(u32 sector, u8 *buffer)
{
  u8 r1;

  //!< if ver = SD2.0 HC, sector need <<9 
  if(SD0_CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	 sector = sector<<9;
  }
	
  //!< Send CMD17 : Read single block command 
  r1 = MSD0_send_command(CMD17, sector, 0);
	
  if(r1 != 0x00)
  {
	 return 1;
  }
	
  //!< Start read and return the result 
  r1 = MSD0_read_buffer(buffer, MSD_BLOCKSIZE, RELEASE);

  //!< Send stop data transmit command - CMD12 
  MSD0_send_command(CMD12, 0, 0);

  return r1;
}

/* --------------------------------------------------------------------------*/
/**
* @name SDCard_ReadMultiBlock
* @brief 
* @details Launch timing to read multiple blocks from SD Card
*
* @param[in] sector No. of sector to read
* @param[in] buffer Pointer to store read data
* @param[in] NbrOfSector No. of sector to read
*
* @returns 0 successful
*          1 failed
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u8 SDCard_ReadMultiBlock(u32 sector, u8 *buffer, u32 NbrOfSector)
{
  u8 r1;
  register u32 i;
  
  if(SD0_CardInfo.CardType != CARDTYPE_SDV2HC)   //!< if ver = SD2.0 HC, sector need <<9 
  {
	 sector = sector<<9;
  }
  
  r1 = MSD0_send_command(CMD18, sector, 0);      //!< Send CMD18 : Read multi block command 
  
  if(r1 != 0x00)
  {
     return 1;
  }

  for(i=0; i<NbrOfSector; i++)                   //!< Start read	
  {
     if(MSD0_read_buffer(buffer+i*MSD_BLOCKSIZE, MSD_BLOCKSIZE, HOLD))
     {		 
		 MSD0_send_command(CMD12, 0, 0);        //!< Send stop data transmit command - CMD12	
		 
		 MSD0_card_disable();                   //!< chip disable and dummy byte 
		 
		 return 2;
     }
  }

  MSD0_send_command(CMD12, 0, 0);//!< Send stop data transmit command - CMD12 
 
  MSD0_card_disable();           //!< chip disable and dummy byte 
  
  MSD0_spi_read_write(DUMMY_BYTE);
	
  return 0;
}

/* --------------------------------------------------------------------------*/
/**
* @name SDCard_WriteSingleBlock
* @brief 
* @details Launch timing to write single sector data to SD Card
*
* @param[in] sector No. of sector to be written
* @param[in] buffer Pointer to the data to be written
*
* @returns 0 successful
*          1 failed
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
u8 SDCard_WriteSingleBlock(u32 sector, uc8 *buffer)
{
  u8 r1;
  register u16 i;
  u32 retry;
  
  if(SD0_CardInfo.CardType != CARDTYPE_SDV2HC)   //!< if ver = SD2.0 HC, sector need <<9 
  {
	 sector = sector<<9;
  }
  
  r1 = MSD0_send_command(CMD24, sector, 0);      //!< Send CMD24 : Write single block command 
	
  if(r1 != 0x00)
  {
	 return 1;
  }
  
  MSD0_card_enable();                            //!< Card enable, Prepare to write 
  MSD0_spi_read_write(DUMMY_BYTE);
  MSD0_spi_read_write(DUMMY_BYTE);
  MSD0_spi_read_write(DUMMY_BYTE);
  
  MSD0_spi_read_write(0xFE);                     //!< Start data write token: 0xFE 
  
  for(i=0; i<MSD_BLOCKSIZE; i++)                 //!< Start single block write the data buffer 
  {
    MSD0_spi_read_write(*buffer++);
  }
  
  MSD0_spi_read_write(DUMMY_BYTE);               //!< 2Bytes dummy CRC 
  MSD0_spi_read_write(DUMMY_BYTE);
  
  r1 = MSD0_spi_read_write(DUMMY_BYTE);          //!< MSD card accept the data 
  
  if((r1&0x1F) != 0x05)
  {
    MSD0_card_disable();
    return 2;
  }
  
  retry = 0;                                     //!< Wait all the data programm finished
  while(MSD0_spi_read_write(DUMMY_BYTE) == 0x00)
  {		 
	 if(retry++ == 0x40000)                      //!< Timeout return 
	 {
	    MSD0_card_disable();
		
	    return 3;
	 }
  }
  
  MSD0_card_disable();                           //!< chip disable and dummy byte 
  
  MSD0_spi_read_write(DUMMY_BYTE);
	
  return 0;
}

/* --------------------------------------------------------------------------*/
/**
* @name SDCard_WriteMultiBlock
* @brief 
* @details Launch timing to write multiple blocks to SD Card
*
* @param[in] sector No. of sector to be written
* @param[in] buffer Pointer of data
* @param[in] NbrOfSector No. of sector to be written
*
* @returns 0 successful
*          1 failed
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
/*******************************************************************************
* Function Name  : MSD0_WriteMultiBlock
* Description    : None
* Input          : - sector:
*				   - buffer:
*                  - NbrOfSector:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
u8 SDCard_WriteMultiBlock(u32 sector, uc8 *buffer, u32 NbrOfSector)
{
  u8 r1;
  register u16 i;
  register u32 n;
  u32 retry;
  
  if(SD0_CardInfo.CardType != CARDTYPE_SDV2HC)   //!< if ver = SD2.0 HC, sector need <<9 
  {
	  sector = sector<<9;
  }
  
  if(SD0_CardInfo.CardType != CARDTYPE_MMC)      //!< Send command ACMD23 berfore multi write if is not a MMC card 
  {
      MSD0_send_command(CMD55,0,0);
	  MSD0_send_command(ACMD23, NbrOfSector, 0x00);	  
  }

  r1 = MSD0_send_command(CMD25, sector, 0);      //!< Send CMD25 : Write nulti block command	
	
  if(r1 != 0x00)
  {
	  return 1;
  }
  
  MSD0_card_enable();                            //!< Card enable, Prepare to write 
  MSD0_spi_read_write(DUMMY_BYTE);

  for(n=0; n<NbrOfSector; n++)
  {		 
	 MSD0_spi_read_write(0xFC);                  //!< Start multi block write token: 0xFC 

	 for(i=0; i<MSD_BLOCKSIZE; i++)
	 {
		MSD0_spi_read_write(*buffer++);
	 }	
	 
	 MSD0_spi_read_write(DUMMY_BYTE);            //!< 2Bytes dummy CRC 
	 MSD0_spi_read_write(DUMMY_BYTE);
	 
	 r1 = MSD0_spi_read_write(DUMMY_BYTE);       //!< MSD card accept the data 
	 
	 if((r1&0x1F) != 0x05)
	 {
	    MSD0_card_disable();
		
	    return 2;
	 }

	 retry = 0;                                  //!< Wait all the data programm finished	
	 while(MSD0_spi_read_write(DUMMY_BYTE) != 0xFF)
	 {		
		if(retry++ == 0x40000)                  //!< Timeout return 
		{
		   MSD0_card_disable();
		   
		   return 3;
		}
	 }
  }
  
  r1 = MSD0_spi_read_write(0xFD);                //!< Send end of transmit token: 0xFD 
  
  if(r1 == 0x00)
  {
	 return 4;
  }
  
  retry = 0;                                     //!< Wait all the data programm finished 
  
  while(MSD0_spi_read_write(DUMMY_BYTE) != 0xFF)
  {		 
	 if(retry++ == 0x40000)                     //!< Timeout return 
	 {
	     MSD0_card_disable();
		 
	     return 5;
	 }
  }
  
  MSD0_card_disable();                           //!< chip disable and dummy byte 
  
  MSD0_spi_read_write(DUMMY_BYTE);

  return 0;
}



/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "stm32f4xx.h"
#include "bsp_spi.h"


/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */

extern MSD_CARDINFO SD0_CardInfo;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	switch (pdrv) {
	case DEV_RAM :		

		return RES_OK;

	case DEV_MMC :
		
		return RES_OK;

	case DEV_USB :
		
		return RES_OK;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;

	switch (pdrv) {
	case DEV_RAM :		
		           stat = SDCard_Init();

				   if(stat==0) { return RES_OK; }
                   else{ return STA_NOINIT; }

	case DEV_MMC :		
		return STA_NOINIT;

	case DEV_USB :

		return STA_NOINIT;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	int result;

	if (pdrv || !count) return RES_PARERR;		/* Check parameter */

	switch (pdrv) {
	case DEV_RAM :
                if(count==1)            /* 1个sector的读操作 */      
                {   
                    result =  SDCard_ReadSingleBlock( sector ,buff );
                    if(result == 0) 
					{ 
						return RES_OK; 
					}
                    else
					{ 
						return RES_ERROR; 
					}    
                }                                                
                else                    /* 多个sector的读操作 */     
                {  
                    result = SDCard_ReadMultiBlock( sector , buff ,count);
                    if(result == 0)
					{ 
						return RES_OK; 
					}
                    else
					{ 
						return RES_ERROR; 
					} 
                }      

		//return RES_ERROR;

	case DEV_MMC :
		
		return RES_OK;

	case DEV_USB :
		
		return RES_OK;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	int result;

	switch (pdrv) {
	case DEV_RAM :
                if(count==1)            /* 1个sector的写操作 */      
                {   
                    result = SDCard_WriteSingleBlock( sector , (uint8_t *)(&buff[0]) ); 
                    if(result == 0)
					{ 
						return RES_OK; 
					}
                    else
					{ 
						return RES_ERROR;
					} 
                }                                                
                else                    /* 多个sector的写操作 */    
                {  
                    result = SDCard_WriteMultiBlock( sector , (uint8_t *)(&buff[0]) , count );
                    if(result == 0)
					{ 
						return RES_OK; 
					}
                    else
					{ 
						return RES_ERROR; 
					}   
                }   

	case DEV_MMC :		

		return RES_OK;

	case DEV_USB :
		
		return RES_OK;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	switch (pdrv) {
	case DEV_RAM :
                SDCard_GetCardInfo(&SD0_CardInfo);

				switch(cmd)
				{
					case CTRL_SYNC : 
						            return RES_OK;
					case GET_SECTOR_COUNT : 
									*(DWORD*)buff = SD0_CardInfo.Capacity/SD0_CardInfo.BlockSize;
									return RES_OK;
					case GET_BLOCK_SIZE :
									*(WORD*)buff = SD0_CardInfo.BlockSize;
									return RES_OK;	
					case CTRL_POWER :
									break;
					case CTRL_LOCK :
									break;
					case CTRL_EJECT :
									break;
					/* MMC/SDC command */
					case MMC_GET_TYPE :
									break;
					case MMC_GET_CSD :
									break;
					case MMC_GET_CID :
									break;
					case MMC_GET_OCR :
									break;
					case MMC_GET_SDSTAT :
									break;
					default:
						break;
				}		

		return RES_OK;

	case DEV_MMC :		

		return RES_PARERR;

	case DEV_USB :

		return RES_PARERR;
	}

	return RES_PARERR;
}


/* 得到文件Calendar格式的建立日期,是DWORD get_fattime (void) 逆变换 */							
/*-----------------------------------------------------------------------*/
/* User defined function to give a current time to fatfs module          */
/* 31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
/* 15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */    
DWORD get_fattime (void)
{
   
    return 0;
}



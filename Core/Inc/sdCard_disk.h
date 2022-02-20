/*
 * sdCard_disk.h
 *
 *  Created on: Feb 10, 2022
 *      Author: Maxiufeng
 */

#ifndef INC_SDCARD_DISK_H_
#define INC_SDCARD_DISK_H_


/*
 * @note If using the default form HAL_SD_Init()
 * generated by cubeMx, add the following definitions to the file.
 * 		// #define DISABLE_SD_INIT
 * -----------------------------------------------------------
 * If you delete the following code in HAL_SD_Init()
 * do not add the above definition.
 * 	if (HAL_SD_Init(&hsd) != HAL_OK)
 * 	{
 * 		Error_Handler();
 * 	}
 * 	if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
 * 	{
 *	  	Error_Handler();
 * 	}
 * 	----------------------------------------------------------
 * If the SD insertion detection pin is used, uncomment the definition below.
 * 		// #define ENABLE_SD_DETECT
 */


/* external header file */
#include "stm32f4xx_hal.h"
#include "diskio.h"


/* Extern variables ---------------------------------------------------------*/
extern SD_HandleTypeDef hsd;


/**
  * @brief  Disk IO Driver structure definition
  */
typedef struct
{
  DSTATUS (*disk_initialize) (void);                     /*!< Initialize Disk Drive                     */
  DSTATUS (*disk_status)     (void);                     /*!< Get Disk Status                           */
  DRESULT (*disk_read)       (BYTE*, DWORD, UINT);       /*!< Read Sector(s)                            */
  DRESULT (*disk_write)      (const BYTE*, DWORD, UINT); /*!< Write Sector(s) when _USE_WRITE = 0       */
  DRESULT (*disk_ioctl)      (BYTE, void*);              /*!< I/O control operation when _USE_IOCTL = 1 */

}SDCardDiskio_drvTypeDef;


/* Externally declare SD card io driver structure variables */
extern const SDCardDiskio_drvTypeDef  SD_Driver;


/**
  * @brief	SD Card Whether to initialize
  */
//#define DISABLE_SD_INIT

/**
  * @brief	SD Card detect GPIO-Pin define, A low level has a card
  * 	It needs to be modified according to the actual pin.
  */
//#define ENABLE_SD_DETECT

#if defined(ENABLE_SD_DETECT)
#define SD_DETECT_Pin 	GPIO_PIN_13
#define SD_DETECT_Port 	GPIOB
#endif

#define UNUSED_SD_ERASE 1

/**
  * @brief  SD status structure definition
  */
#define   MSD_OK                        ((uint8_t)0x00)
#define   MSD_ERROR                     ((uint8_t)0x01)

/**
  * @brief  SD transfer state definition
  */
#define   SD_TRANSFER_OK                ((uint8_t)0x00)
#define   SD_TRANSFER_BUSY              ((uint8_t)0x01)

#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)
#define SD_DATATIMEOUT           ((uint32_t)100000000)


#endif /* INC_SDCARD_DISK_H_ */
/*
 * sdCard_disk.c
 *
 *  Created on: Feb 10, 2022
 *      Author: Maxiufeng
 */


#include "sdCard_disk.h"


#define SD_TIMEOUT 30 * 1000

#define SD_DEFAULT_BLOCK_SIZE 512

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

static volatile  UINT  WriteStatus = 0, ReadStatus = 0;

/* function declaration -----------------------------------------------------*/
/* static function */
static uint8_t SD_IsDetected(void);
static uint8_t SD_Init(void);
static DSTATUS SD_CheckStatus(void);
#if !defined(UNUSED_SD_ERASE)
static uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);
#endif /* UNUSED_SD_ERASE */
static int SD_CheckStatusWithTimeout(uint32_t timeout);
static uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
static uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
static void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo);

/* sd card IO driver */
DSTATUS SD_initialize(void);
DSTATUS SD_status (void);
DRESULT SD_read(BYTE *buff, DWORD sector, UINT count);
DRESULT SD_write(const BYTE *buff, DWORD sector, UINT count);
DRESULT SD_ioctl(BYTE cmd, void *buff);



/* Define the SD card io driver structure -----------------------------------*/
const SDCardDiskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
  SD_write,
  SD_ioctl,
};


/* function definition ------------------------------------------------------*/

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param  None
 * @retval Returns if SD is detected or not
 */
static uint8_t SD_IsDetected(void)
{
	__IO uint8_t status = SD_PRESENT;

#if defined(ENABLE_SD_DETECT)
	if (HAL_GPIO_ReadPin(SD_DETECT_Port, SD_DETECT_Pin) != GPIO_PIN_RESET)
		status = SD_NOT_PRESENT;
#endif /* ENABLE_SD_DETECT */

  return status;
}

/**
  * @brief  Initializes the SD card device use HAL library.
  * @retval SD status
  */
static uint8_t SD_Init(void)
{
  uint8_t sd_state = MSD_OK;
  /* Check if the SD card is plugged in the slot */
  if (SD_IsDetected() != SD_PRESENT)
  {
    return MSD_ERROR;
  }

  /* HAL SD initialization */
  sd_state = HAL_SD_Init(&hsd);
  /* Configure SD Bus width (4 bits mode selected) */
  if (sd_state == MSD_OK)
  {
    /* Enable wide operation */
    if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != MSD_OK)
    {
      sd_state = MSD_ERROR;
    }
  }
  else
  {
	  sd_state = MSD_ERROR;
  }

  return sd_state;
}


/**
  * @brief check sd card status use HAL_SD_GetCardState()
  * @retval DSTATUS: Operation status
  */
static DSTATUS SD_CheckStatus(void)
{

	Stat = STA_NOINIT;

	uint8_t cardState = ((HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER ) ?
														SD_TRANSFER_OK : SD_TRANSFER_BUSY);

	if(cardState == MSD_OK)
	{
		Stat &= ~STA_NOINIT;
	}

	return Stat;
}

/**
  * @brief  Erases the specified memory area of the given SD card.
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval SD status
  */
#if !defined(UNUSED_SD_ERASE)
static uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
  uint8_t sd_state = MSD_OK;

  if (HAL_SD_Erase(&hsd, StartAddr, EndAddr) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  return sd_state;
}
#endif /* UNUSED_SD_ERASE */

/**
  * @brief check status and wait time,used DMA read and write
  * @param timeout: max wait time value
  * @retval int 0:ok /-1:error
  */
static int SD_CheckStatusWithTimeout(uint32_t timeout)
{
	uint32_t timer = HAL_GetTick();
	/* block until SDIO IP is ready again or a timeout occur */
	while(HAL_GetTick() - timer < timeout)
	{
	  uint8_t cardState = ((HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER ) ?
	  														SD_TRANSFER_OK : SD_TRANSFER_BUSY);
	  if (cardState == SD_TRANSFER_OK)
	  {
		  return 0;
	  }
	}

	return -1;
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @retval SD status
  */
static uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;

  /* Read block(s) in DMA transfer mode */
  if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  return sd_state;
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @retval SD status
  */
static uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;

  /* Write block(s) in DMA transfer mode */
  if (HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  return sd_state;
}


/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None
  */
static void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo)
{
  /* Get SD card Information */
  HAL_SD_GetCardInfo(&hsd, CardInfo);
}

/* The function of operating SD Card is provided to diskio.c ************************ */
/**
  * @brief  Initializes a Drive
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize(void)
{
#if !defined(DISABLE_SD_INIT)

	if(SD_Init() == MSD_OK)
	{
		Stat = SD_CheckStatus();
	}

#else
	Stat = SD_CheckStatus();
#endif

	return Stat;
}


/**
  * @brief  Gets Disk Status
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(void)
{
	return SD_CheckStatus();
}


/**
  * @brief  Reads Sector(s)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */

DRESULT SD_read(BYTE *buff, DWORD sector, UINT count)
{
	DRESULT res = RES_ERROR;
	uint32_t timeout;

	/*
	 * ensure the SDCard is ready for a new operation
	 */

	if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
	{
		return res;
	}
    if(BSP_SD_ReadBlocks_DMA((uint32_t*)buff,
                             (uint32_t) (sector),
                             count) == MSD_OK)
    {
      ReadStatus = 0;
      /* Wait that the reading process is completed or a timeout occurs */
      timeout = HAL_GetTick();
      while((ReadStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
      {
      }
      /* in case of a timeout return error */
      if (ReadStatus == 0)
      {
        res = RES_ERROR;
      }
      else
      {
        ReadStatus = 0;
        timeout = HAL_GetTick();

        while((HAL_GetTick() - timeout) < SD_TIMEOUT)
        {
        	uint8_t cardState = ((HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER ) ?
        		  														SD_TRANSFER_OK : SD_TRANSFER_BUSY);
          if (cardState == SD_TRANSFER_OK)
          {
            res = RES_OK;
            break;
          }
        }
      }
    }

  return res;
}


/**
  * @brief  Writes Sector(s)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT SD_write(const BYTE *buff, DWORD sector, UINT count)
{
	DRESULT res = RES_ERROR;
	uint32_t timeout;

	WriteStatus = 0;

	if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
	{
		return res;
	}


    if(BSP_SD_WriteBlocks_DMA((uint32_t*)buff,
                              (uint32_t)(sector),
                              count) == MSD_OK)
    {
      /* Wait that writing process is completed or a timeout occurs */

      timeout = HAL_GetTick();
      while((WriteStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
      {
      }
      /* in case of a timeout return error */
      if (WriteStatus == 0)
      {
        res = RES_ERROR;
      }
      else
      {
        WriteStatus = 0;
        timeout = HAL_GetTick();

        while((HAL_GetTick() - timeout) < SD_TIMEOUT)
        {
        	uint8_t cardState = ((HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER ) ?
        	        		  						SD_TRANSFER_OK : SD_TRANSFER_BUSY);
          if (cardState == SD_TRANSFER_OK)
          {
            res = RES_OK;
            break;
          }
        }
      }
    }
  return res;
}


/**
  * @brief  I/O control operation
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
DRESULT SD_ioctl(BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  HAL_SD_CardInfoTypeDef CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}


/* DMA and IT callback function section. ***********************/
/**
  * @brief Rx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
	ReadStatus = 1;
}


/**
  * @brief Tx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
	WriteStatus = 1;
}


/**
  * @brief SD Abort callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
  /* user code BEGIN */

  /* user code END */
}


/**
  * @brief SD error callbacks
  * @param hsd: Pointer SD handle
  * @retval None
  */
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
	/* user code BEGIN */

	/* user code END */
}

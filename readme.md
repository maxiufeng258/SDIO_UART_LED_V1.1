# 1.SDIO driver based on HAL lib

**[sdCard_disk.h/c] provides low-level read and write operations in SDIO 4bit mode.**

**It is stored in the form of a structure of function pointers.**

```c
/**
  * @brief  Disk IO Driver structure definition
  */
typedef struct
{
  DSTATUS (*disk_initialize) (void);                     /*!< Initialize Disk Drive */
  DSTATUS (*disk_status)     (void);                     /*!< Get Disk Status */
  DRESULT (*disk_read)       (BYTE*, DWORD, UINT);       /*!< Read Sector(s) */
  DRESULT (*disk_write)      (const BYTE*, DWORD, UINT); /*!< Write Sector(s) when _USE_WRITE = 0       */
  DRESULT (*disk_ioctl)      (BYTE, void*);   /*!< I/O control operation when _USE_IOCTL = 1 */
}SDCardDiskio_drvTypeDef;
```

# 2. CubeMx Configuration

- SDIO 4bit Mode
- DMA: SDIO_Rx   SDIO_Tx
  - Note  DMA Request Setting -> Memort  [Byte]  [Single]

- NVIC: global interrupt   Enable

# 3. [sdCard_disk.h] setting

```c
/**
  * @brief	SD Card detect GPIO-Pin define, A low level has a card
  * 	It needs to be modified according to the actual pin.
  */
//#define ENABLE_SD_DETECT

#if defined(ENABLE_SD_DETECT)
#define SD_DETECT_Pin 	GPIO_PIN_13
#define SD_DETECT_Port 	GPIOB
#endif
```

# 3. [main.c] test example

```c
  /* USER CODE BEGIN 2 */

  HAL_Delay(5000);
  FRESULT fRes;

  FATFS sdFat;
  fRes = f_mount(&sdFat, "0:", 1);

  if (fRes == FR_OK)
  {
	  printf("sd card disk mount ok...\r\n");
	  HAL_GPIO_TogglePin(LD2_Green_GPIO_Port, LD2_Green_Pin);

	  DWORD free_byte[1], total_byte[1];
	  fRes = ff_getCapacity("0:", free_byte, total_byte);
	  if (fRes == FR_OK)
	  {
		  printf("get capacity is succeed..\r\n");
		  printf("total Capacity : %ld KByte\r\nfree  Capacity : %ld KByte\r\n",total_byte[0], free_byte[0]);

		  TCHAR path[100] = "0:";
		  fRes = ff_scanDisk(path);
		  if (fRes == FR_OK)
			  printf("-----scan over-----\r\n");
	  }

	  fRes = f_open(&fp, "0:FIRST.TXT", FA_OPEN_ALWAYS|FA_WRITE|FA_READ);

	  fRes = f_lseek(&fp, 0);

	  UINT br[1];
	  uint8_t buff[100];
	  fRes = f_read(&fp, buff, f_size(&fp), br);

	  UINT bw[1];
	  fRes = f_lseek(&fp, f_size(&fp));
	  fRes = f_write(&fp, "123456", 7, bw);

	  (fRes == FR_OK)? printf("scan sd disk ok...\r\n"): printf("scan sd disk error...\r\n");

	  fRes = f_close(&fp);

	  f_mount(NULL, "0:", 1);
  }
  else
	  printf("error code : %d\r\n", fRes);

  /* USER CODE END 2 */
```


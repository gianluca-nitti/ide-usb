/**
  ******************************************************************************
  * @file    usbd_msc_storage_template.c
  * @author  MCD Application Team
  * @brief   Memory management layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}{adafruit}_sd.c"
EndBSPDependencies */

#include "ide.h"
#include "usbd_msc_storage_template.h"

int8_t STORAGE_Init(uint8_t lun);

int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num,
                           uint16_t *block_size);

int8_t  STORAGE_IsReady(uint8_t lun);

int8_t  STORAGE_IsWriteProtected(uint8_t lun);

int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                    uint16_t blk_len);

int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                     uint16_t blk_len);

int8_t STORAGE_GetMaxLun(void);

/* USB Mass storage Standard Inquiry Data */
int8_t  STORAGE_Inquirydata[] =  /* 36 */ {

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0', '1',                     /* Version      : 4 Bytes */
};

USBD_StorageTypeDef USBD_MSC_Template_fops = {
  STORAGE_Init,
  STORAGE_GetCapacity,
  STORAGE_IsReady,
  STORAGE_IsWriteProtected,
  STORAGE_Read,
  STORAGE_Write,
  STORAGE_GetMaxLun,
  STORAGE_Inquirydata
};

int8_t STORAGE_Init(uint8_t lun) {
	return 0;
}

int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size) {
	*block_num  = ide_get_num_sectors();
	*block_size = 512;
	return 0;
}

int8_t  STORAGE_IsReady(uint8_t lun) {
	return ide_ready() ? 0 : 1;
}

int8_t  STORAGE_IsWriteProtected(uint8_t lun) {
	return 0;
}

int8_t STORAGE_Read(uint8_t lun, uint8_t *buf,
                    uint32_t blk_addr, uint16_t blk_len) {
	ide_read_sectors(blk_addr, buf, blk_len);
	return 0;
}

int8_t STORAGE_Write(uint8_t lun, uint8_t *buf,
                     uint32_t blk_addr, uint16_t blk_len) {
	return 0;
}

int8_t STORAGE_GetMaxLun(void) {
	return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

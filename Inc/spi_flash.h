/**
  ******************************************************************************
  * @file    SPI/SPI_FLASH/spi_flash.h
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   This file contains all the functions prototypes for the spi_flash
  *          firmware driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* M25P SPI Flash supported commands */
#define sFLASH_CMD_WRITE          0x02  /* Write to Memory Page Program */
#define sFLASH_CMD_WRSR           0x01  /* Write Register WRR */
#define sFLASH_CMD_WREN           0x06  /* Write enable WREN */
#define sFLASH_CMD_READ           0x03  /* Read from Memory instruction */
#define sFLASH_CMD_RDSR           0x05  /* Read Status Register instruction  */
#define sFLASH_CMD_RDID           0x9F  /* Read identification */
#define sFLASH_CMD_SE             0xD8  /* Sector Erase instruction */
#define sFLASH_CMD_BE             0xC7  /* Bulk Erase instruction */

#define sFLASH_WIP_FLAG           0x01  /* Write In Progress (WIP) flag */

#define sFLASH_DUMMY_BYTE         0xA5
#define sFLASH_SPI_PAGESIZE       0x100

/* M25P FLASH SPI Interface pins  */
#define sFLASH_SPI                           SPI2
#define sFLASH_SPI_SCK_PIN                   GPIO_PIN_1
#define sFLASH_SPI_SCK_GPIO_PORT             GPIOC
#define sFLASH_SPI_MISO_PIN                  GPIO_PIN_2
#define sFLASH_SPI_MISO_GPIO_PORT            GPIOC
#define sFLASH_SPI_MOSI_PIN                  GPIO_PIN_3
#define sFLASH_SPI_MOSI_GPIO_PORT            GPIOC
#define sFLASH_CS_PIN                        GPIO_PIN_8
#define sFLASH_CS_GPIO_PORT                  GPIOC

/* Exported macro ------------------------------------------------------------*/
/* Select sFLASH: Chip Select pin low */
#define sFLASH_CS_LOW()       HAL_GPIO_WritePin(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, GPIO_PIN_RESET)
/* Deselect sFLASH: Chip Select pin high */
#define sFLASH_CS_HIGH()      HAL_GPIO_WritePin(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, GPIO_PIN_SET)

/* Exported functions ------------------------------------------------------- */

/* High layer functions  */
void sFLASH_DeInit();
void sFLASH_Init();
void sFLASH_EraseSector(uint32_t SectorAddr);
void sFLASH_EraseBulk(void);
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t sFLASH_ReadID(void);
void sFLASH_StartReadSequence(uint32_t ReadAddr);

/* Low layer functions */
uint8_t sFLASH_ReadByte(void);
uint8_t sFLASH_SendByte(uint8_t byte);
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord);
void sFLASH_WriteEnable(void);
void sFLASH_WaitForWriteEnd(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_FLASH_H */

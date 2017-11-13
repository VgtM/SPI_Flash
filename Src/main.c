/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_tim.h"

/* USER CODE BEGIN Includes */
#include "spi_flash.h"
#include "flash.h"
#include "lsm6ds3.h"
#include "buffer.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;
/* USER CODE BEGIN PV */
/* Private define */
#define Fail 0x00
#define Success 0x01
#define FLASH_WRITE_ADDRESS 0x04000000
#define FLASH_READ_ADDRESS FLASH_WRITE_ADDRESS
#define FLASH_ERASE_ADDRESS FLASH_WRITE_ADDRESS
#define BufferSize 256
/* Private variables ---------------------------------------------------------*/
uint8_t TBuffer[BufferSize] = "this is a test";
uint8_t RBuffer[BufferSize];
__IO uint8_t Index = 0x0;
__IO uint32_t FlashID = 0;
triplet acc_reading;
triplet gyro_reading1;
triplet gyro_reading2;
triple_ring_buffer angular_velocity_buffer_sternum;
triple_ring_buffer angular_velocity_buffer_waist;
uint8_t write_buff[PAGE_SIZE];
//float write_buffx[PAGE_SIZE];
//float write_buffy[PAGE_SIZE];
//float write_buffz[PAGE_SIZE];
uint8_t read_buff[PAGE_SIZE];
//float read_buffx[PAGE_SIZE];
//float read_buffy[PAGE_SIZE];
//float read_buffz[PAGE_SIZE];
triplet angle_i;
triplet angular_velocity_i;
/*Accelerometer initialization routine*/
LSM6DS3_StatusTypedef acc_init_status, gyro1_init_status, gyro2_init_status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
//int Buffercmp(float* PBuffer1, float* pBuffer2, uint16_t BufferLength);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  init_buffer(&angular_velocity_buffer_sternum);
  init_buffer(&angular_velocity_buffer_waist);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  /*Start timer interrupt*/
   HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */
   triplet acc_reading;
   triplet gyro_reading1;
   triplet gyro_reading2;
   angular_velocity_i.x = 0;
   angular_velocity_i.y = 0;
   angular_velocity_i.z = 0;
   triplet angle_f;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   gyro1_init_status = init_gyroscope(&hi2c1,SENSOR_1,dps_250,rate416hz);
   gyro2_init_status = init_gyroscope(&hi2c1,SENSOR_2,dps_250,rate416hz);
  while (1)
  {
  /* USER CODE END WHILE */
	  if(peek(&angular_velocity_buffer_sternum) == BUFFER_AVAILABLE)
	  	  {
	  		  fetch(&angular_velocity_buffer_sternum, &gyro_reading1);
	  		  //fetchBuffer(&angular_velocity_buffer_sternum, &write_buffx, &write_buffy, &write_buffz);
	  	  }
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
  //verify_flash_memory(&hspi1);

  	/*triple_ring_buffer write_buff[PAGE_SIZE];
  	triple_ring_buffer read_buff[PAGE_SIZE];*/
  	/*while(write_buff[i] != NULL){
  		write_buff[i] = angular_velocity_buffer_sternum->next;
  	}*/
  	for(uint16_t p0=0;p0<PAGE_SIZE;p0++)
    {
  	  //write_buff[p0]=p0;
  	  read_buff[p0]  = 0;
    }

    Master_WriteToFlash_Page(&hspi1, FLASH_WRITE_ADDRESS, write_buff, 1);
    Master_ReadFromFlash( &hspi1, FLASH_WRITE_ADDRESS, read_buff, PAGE_SIZE);

    for(int i = 0; i < PAGE_SIZE; i++){
    	  if(read_buff[i] != i){
    		  Error_Handler();
    	  }
      }
      int BufferStatus = Buffercmp(write_buff, read_buff, PAGE_SIZE);

      if(BufferStatus != Success){
    	  Error_Handler();
      }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

int Buffercmp(uint8_t* TBuffer, uint8_t* RBuffer, uint16_t BufferLength){
	for(int i = 0; i < BufferLength; i++){
		if(TBuffer[i] != RBuffer[i]){
			return Fail;
		}
	}
	return Success;
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t xHandleTaskSimulator = NULL;
static TaskHandle_t xHandleTaskKeyboard = NULL;
static TaskHandle_t xHandleTaskDacTest = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void vTaskSimulator(void *pvParameters)
{
  uint8_t res;
  while(1)
  {
    vTaskDelay(200);
    LCD_Clear(RED);
    vTaskDelay(50);
    LCD_Clear(GREEN);
    vTaskDelay(50);
    LCD_Clear(BLUE);
    vTaskDelay(50);
    
		res = nes_load("0:/my.nes");	//开始nes游戏
    printf("error %d\r\n", res);
    vTaskDelay(100);
  }
}

static void vTaskKeyboard(void *pvParameters)
{
  uint16_t count = 0;
  
  OLED_BLK_Clr();
  vTaskDelay(2000);
  HAL_GPIO_WritePin(POWER_SET_GPIO_Port, POWER_SET_Pin, GPIO_PIN_SET);
	OLED_BLK_Set();
  xTaskCreate( vTaskSimulator, /* 任务函数 */
              "vTaskSimulator", /* 任务名 */
              1*1024/4, /* 任务栈大小，单位 word，也就是 4 字节 */
              NULL, /* 任务参数 */
              2, /* 任务优先级*/
              &xHandleTaskSimulator ); /* 任务句柄 */
  while(HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_RESET)
  {
    vTaskDelay(10);
  }
  while(1)
  {
    if(HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_RESET)
    {
      while(1)
      {
        vTaskDelay(10);
        if(HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_RESET)
        {
          count++;
          if(count >= 200)
          {
            HAL_GPIO_WritePin(POWER_SET_GPIO_Port, POWER_SET_Pin, GPIO_PIN_RESET);
            OLED_BLK_Clr();
            //LCD_Clear(BLACK);
            //while(1);
          }
        }
        else
        {
          count = 0;
          break;
        }
      }
    }
    vTaskDelay(10);
  }
}

static void vTaskDacTest(void *pvParameters)
{
  uint16_t dacval = 0;
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
  //HAL_TIM_Base_Start_IT(&htim13);
  while(1)
  {
    /*if(dacval<0xFFF)
      dacval+=1;
    else
      dacval = 0;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacval);*/
    vTaskDelay(10);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //SCB->VTOR = FLASH_BASE | 0xC000;
  uint8_t res;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM13_Init();
  FLASH_If_Init();
  /* USER CODE BEGIN 2 */

  Lcd_Init();			//初始化OLED  
  if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
  {
    /* FatFs Initialization Error */
    Error_Handler();
  }
  printf("system is ready,by Application\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(POWER_SET_GPIO_Port, POWER_SET_Pin, GPIO_PIN_SET);
  xTaskCreate( vTaskKeyboard, /* 任务函数 */
              "vTaskKeyboard", /* 任务名 */
              512/4, /* 任务栈大小，单位 word，也就是 4 字节 */
              NULL, /* 任务参数 */
              3, /* 任务优先级*/
              &xHandleTaskKeyboard ); /* 任务句柄 */
  xTaskCreate( vTaskDacTest, /* 任务函数 */
              "vTaskDacTest", /* 任务名 */
              512/4, /* 任务栈大小，单位 word，也就是 4 字节 */
              NULL, /* 任务参数 */
              3, /* 任务优先级*/
              &xHandleTaskDacTest ); /* 任务句柄 */
  vTaskStartScheduler();
  while (1)
  {
    /*HAL_Delay(200);
    LCD_Clear(RED);
    HAL_Delay(200);
    LCD_Clear(GREEN);
    HAL_GPIO_WritePin(POWER_SET_GPIO_Port, POWER_SET_Pin, GPIO_PIN_SET);
    HAL_Delay(200);
    LCD_Clear(BLUE);
    
		res = nes_load("0:/my.nes");	//开始nes游戏
    printf("error %d\r\n", res);*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  uint16_t buff;
  extern uint16_t apu_count;
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if(htim->Instance == TIM13)
  {
    if( apu_count < 367)
  	{
  	  extern uint16_t *wave_buffers;
  		buff = wave_buffers[apu_count];
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, buff);
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, buff);
  		apu_count++;
  	}
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  printf("system is error\r\n");
  while(1);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

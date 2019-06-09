/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PH_STA_Pin GPIO_PIN_13
#define PH_STA_GPIO_Port GPIOC
#define PH_MS_Pin GPIO_PIN_14
#define PH_MS_GPIO_Port GPIOC
#define LCD_BL_Pin GPIO_PIN_15
#define LCD_BL_GPIO_Port GPIOC
#define POWER_SET_Pin GPIO_PIN_0
#define POWER_SET_GPIO_Port GPIOC
#define PM_STDBY_Pin GPIO_PIN_1
#define PM_STDBY_GPIO_Port GPIOC
#define PM_CHRG_Pin GPIO_PIN_2
#define PM_CHRG_GPIO_Port GPIOC
#define KEY_SELECT_Pin GPIO_PIN_3
#define KEY_SELECT_GPIO_Port GPIOC
#define DEBUG_TXD_Pin GPIO_PIN_2
#define DEBUG_TXD_GPIO_Port GPIOA
#define DEBUG_TXDA3_Pin GPIO_PIN_3
#define DEBUG_TXDA3_GPIO_Port GPIOA
#define PH_L_Pin GPIO_PIN_4
#define PH_L_GPIO_Port GPIOA
#define PH_R_Pin GPIO_PIN_5
#define PH_R_GPIO_Port GPIOA
#define KEY_UP_Pin GPIO_PIN_6
#define KEY_UP_GPIO_Port GPIOA
#define KEY_RIGHT_Pin GPIO_PIN_7
#define KEY_RIGHT_GPIO_Port GPIOA
#define KEY_LEFT_Pin GPIO_PIN_4
#define KEY_LEFT_GPIO_Port GPIOC
#define KEY_DOWN_Pin GPIO_PIN_5
#define KEY_DOWN_GPIO_Port GPIOC
#define POWER_VIN_Pin GPIO_PIN_0
#define POWER_VIN_GPIO_Port GPIOB
#define PH_MIC_Pin GPIO_PIN_1
#define PH_MIC_GPIO_Port GPIOB
#define KEY_B_Pin GPIO_PIN_12
#define KEY_B_GPIO_Port GPIOB
#define KEY_Y_Pin GPIO_PIN_13
#define KEY_Y_GPIO_Port GPIOB
#define KEY_X_Pin GPIO_PIN_14
#define KEY_X_GPIO_Port GPIOB
#define KEY_A_Pin GPIO_PIN_15
#define KEY_A_GPIO_Port GPIOB
#define SDIO_STA_Pin GPIO_PIN_15
#define SDIO_STA_GPIO_Port GPIOA
#define LCD_CLK_Pin GPIO_PIN_3
#define LCD_CLK_GPIO_Port GPIOB
#define LCD_DAT_Pin GPIO_PIN_5
#define LCD_DAT_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_6
#define LCD_CS_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_7
#define LCD_RST_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_8
#define LCD_DC_GPIO_Port GPIOB
#define KEY_START_Pin GPIO_PIN_9
#define KEY_START_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

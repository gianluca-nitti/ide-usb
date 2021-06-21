/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define IDE_DD4_Pin GPIO_PIN_4
#define IDE_DD4_GPIO_Port GPIOE
#define IDE_DD5_Pin GPIO_PIN_5
#define IDE_DD5_GPIO_Port GPIOE
#define IDE_DD6_Pin GPIO_PIN_6
#define IDE_DD6_GPIO_Port GPIOE
#define IDE_DD7_Pin GPIO_PIN_7
#define IDE_DD7_GPIO_Port GPIOE
#define IDE_DD8_Pin GPIO_PIN_8
#define IDE_DD8_GPIO_Port GPIOE
#define IDE_DD9_Pin GPIO_PIN_9
#define IDE_DD9_GPIO_Port GPIOE
#define IDE_DD10_Pin GPIO_PIN_10
#define IDE_DD10_GPIO_Port GPIOE
#define IDE_DD11_Pin GPIO_PIN_11
#define IDE_DD11_GPIO_Port GPIOE
#define IDE_DD12_Pin GPIO_PIN_12
#define IDE_DD12_GPIO_Port GPIOE
#define IDE_DD13_Pin GPIO_PIN_13
#define IDE_DD13_GPIO_Port GPIOE
#define IDE_DD14_Pin GPIO_PIN_14
#define IDE_DD14_GPIO_Port GPIOE
#define IDE_DD15_Pin GPIO_PIN_15
#define IDE_DD15_GPIO_Port GPIOE
#define IDE_RESET_Pin GPIO_PIN_15
#define IDE_RESET_GPIO_Port GPIOB
#define IDE_DA0_Pin GPIO_PIN_8
#define IDE_DA0_GPIO_Port GPIOD
#define IDE_DA1_Pin GPIO_PIN_9
#define IDE_DA1_GPIO_Port GPIOD
#define IDE_DA2_Pin GPIO_PIN_10
#define IDE_DA2_GPIO_Port GPIOD
#define IDE_CS0_Pin GPIO_PIN_8
#define IDE_CS0_GPIO_Port GPIOC
#define IDE_CS1_Pin GPIO_PIN_9
#define IDE_CS1_GPIO_Port GPIOC
#define IDE_DIOW_Pin GPIO_PIN_11
#define IDE_DIOW_GPIO_Port GPIOC
#define IDE_DD0_Pin GPIO_PIN_0
#define IDE_DD0_GPIO_Port GPIOD
#define IDE_DD1_Pin GPIO_PIN_1
#define IDE_DD1_GPIO_Port GPIOD
#define IDE_DD2_Pin GPIO_PIN_2
#define IDE_DD2_GPIO_Port GPIOD
#define IDE_DD3_Pin GPIO_PIN_3
#define IDE_DD3_GPIO_Port GPIOD
#define IDE_DIOR_Pin GPIO_PIN_4
#define IDE_DIOR_GPIO_Port GPIOB
#define TXS0108E_OE_Pin GPIO_PIN_8
#define TXS0108E_OE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

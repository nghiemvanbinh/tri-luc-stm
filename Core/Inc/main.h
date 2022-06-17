/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f2xx_hal.h"

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
#define tiem_can1_Pin GPIO_PIN_0
#define tiem_can1_GPIO_Port GPIOC
#define tiem_can2_Pin GPIO_PIN_1
#define tiem_can2_GPIO_Port GPIOC
#define tiem_can3_Pin GPIO_PIN_2
#define tiem_can3_GPIO_Port GPIOC
#define cam_bien_Pin GPIO_PIN_0
#define cam_bien_GPIO_Port GPIOA
#define VAL1_Pin GPIO_PIN_1
#define VAL1_GPIO_Port GPIOA
#define VAL2_Pin GPIO_PIN_2
#define VAL2_GPIO_Port GPIOA
#define led1_Pin GPIO_PIN_3
#define led1_GPIO_Port GPIOA
#define led2_Pin GPIO_PIN_4
#define led2_GPIO_Port GPIOA
#define led3_Pin GPIO_PIN_5
#define led3_GPIO_Port GPIOA
#define led4_Pin GPIO_PIN_6
#define led4_GPIO_Port GPIOA
#define may_bom_Pin GPIO_PIN_7
#define may_bom_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define MUX_SEL_Pin GPIO_PIN_13
#define MUX_SEL_GPIO_Port GPIOC
#define DOOR_Pin GPIO_PIN_15
#define DOOR_GPIO_Port GPIOC
#define DOOR_EXTI_IRQn EXTI15_10_IRQn
#define Optron1_2_Pin GPIO_PIN_0
#define Optron1_2_GPIO_Port GPIOA
#define Optron1_1_Pin GPIO_PIN_1
#define Optron1_1_GPIO_Port GPIOA
#define Sv_kont_p_Pin GPIO_PIN_7
#define Sv_kont_p_GPIO_Port GPIOA
#define On_BKK_k1_Pin GPIO_PIN_0
#define On_BKK_k1_GPIO_Port GPIOB
#define On_BKK_k1_EXTI_IRQn EXTI0_IRQn
#define On_BKK_k2_Pin GPIO_PIN_1
#define On_BKK_k2_GPIO_Port GPIOB
#define On_BKK_k2_EXTI_IRQn EXTI1_IRQn
#define Break_K_p_Pin GPIO_PIN_2
#define Break_K_p_GPIO_Port GPIOB
#define error_sv_Pin GPIO_PIN_11
#define error_sv_GPIO_Port GPIOB
#define error_sv_EXTI_IRQn EXTI15_10_IRQn
#define Optron2_2_Pin GPIO_PIN_12
#define Optron2_2_GPIO_Port GPIOB
#define Optron2_1_Pin GPIO_PIN_13
#define Optron2_1_GPIO_Port GPIOB
#define Rele_1_Pin GPIO_PIN_14
#define Rele_1_GPIO_Port GPIOB
#define Rele_5_Pin GPIO_PIN_15
#define Rele_5_GPIO_Port GPIOB
#define Rele_2_Pin GPIO_PIN_8
#define Rele_2_GPIO_Port GPIOA
#define Rele_3_Pin GPIO_PIN_9
#define Rele_3_GPIO_Port GPIOA
#define Rele_4_Pin GPIO_PIN_10
#define Rele_4_GPIO_Port GPIOA
#define OE_RELE_Pin GPIO_PIN_11
#define OE_RELE_GPIO_Port GPIOA
#define KTV_ADR_Pin GPIO_PIN_15
#define KTV_ADR_GPIO_Port GPIOA
#define DISP_LIGHT_BUF_Pin GPIO_PIN_3
#define DISP_LIGHT_BUF_GPIO_Port GPIOB
#define PWR_KTV_BUF_Pin GPIO_PIN_4
#define PWR_KTV_BUF_GPIO_Port GPIOB
#define PCF_INT_Pin GPIO_PIN_5
#define PCF_INT_GPIO_Port GPIOB
#define SD_SW_Pin GPIO_PIN_8
#define SD_SW_GPIO_Port GPIOB
#define SD_SW_EXTI_IRQn EXTI9_5_IRQn
#define ON_3_3V_Pin GPIO_PIN_9
#define ON_3_3V_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

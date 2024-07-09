/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define ntc_input_Pin GPIO_PIN_4
#define ntc_input_GPIO_Port GPIOA
#define reed_role_input_Pin GPIO_PIN_5
#define reed_role_input_GPIO_Port GPIOA
#define flowmetre_Pin GPIO_PIN_6
#define flowmetre_GPIO_Port GPIOA
#define flowmetre_EXTI_IRQn EXTI9_5_IRQn
#define su_microswitch_Pin GPIO_PIN_7
#define su_microswitch_GPIO_Port GPIOA
#define led_boyut_kucuk_Pin GPIO_PIN_0
#define led_boyut_kucuk_GPIO_Port GPIOB
#define led_boyut_orta_Pin GPIO_PIN_1
#define led_boyut_orta_GPIO_Port GPIOB
#define led_boyut_buyuk_Pin GPIO_PIN_2
#define led_boyut_buyuk_GPIO_Port GPIOB
#define buton_fincan_boyut_Pin GPIO_PIN_10
#define buton_fincan_boyut_GPIO_Port GPIOB
#define buton_fincan_boyut_EXTI_IRQn EXTI15_10_IRQn
#define led_uyari_Pin GPIO_PIN_11
#define led_uyari_GPIO_Port GPIOB
#define start_Pin GPIO_PIN_12
#define start_GPIO_Port GPIOB
#define start_EXTI_IRQn EXTI15_10_IRQn
#define led_kahve_normal_Pin GPIO_PIN_13
#define led_kahve_normal_GPIO_Port GPIOB
#define led_kahve_kozde_Pin GPIO_PIN_14
#define led_kahve_kozde_GPIO_Port GPIOB
#define buton_mod_Pin GPIO_PIN_15
#define buton_mod_GPIO_Port GPIOB
#define buton_mod_EXTI_IRQn EXTI15_10_IRQn
#define pompa_Pin GPIO_PIN_8
#define pompa_GPIO_Port GPIOA
#define rezistans_Pin GPIO_PIN_9
#define rezistans_GPIO_Port GPIOA
#define buzzer_Pin GPIO_PIN_12
#define buzzer_GPIO_Port GPIOA
#define buton_fincan_sayi_Pin GPIO_PIN_3
#define buton_fincan_sayi_GPIO_Port GPIOB
#define buton_fincan_sayi_EXTI_IRQn EXTI3_IRQn
#define led_fincan_4_Pin GPIO_PIN_4
#define led_fincan_4_GPIO_Port GPIOB
#define led_fincan_3_Pin GPIO_PIN_5
#define led_fincan_3_GPIO_Port GPIOB
#define led_fincan_2_Pin GPIO_PIN_6
#define led_fincan_2_GPIO_Port GPIOB
#define led_fincan_1_Pin GPIO_PIN_7
#define led_fincan_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum khvFincanBoyut {KUCUK, ORTA, BUYUK} fincanBoyutu;
typedef enum stMach {IDLE,SUDOLDUR,PISIR,HATA} stateMachine;
typedef enum pisirMod {NORMAL,KOZDE} psrMod;
typedef enum mybool {FALSE,TRUE} ptBool;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FLOWMETRE_PULSE_BASINA_MILILITRE 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
uint8_t fincanSayisi = 1;
uint8_t kahveMililitre = 100;
uint8_t totalAlinanSu = 0;
stateMachine machineState = IDLE;
fincanBoyutu kahveFincanBoyutu = KUCUK;
psrMod pisirmeModu = NORMAL;
ptBool startFlag = FALSE;
ptBool sicaklikFlag = FALSE;
uint8_t prevState = 0;
float sicaklik = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void fincanSayiGostergeKontrol(void);
void fincanBoyutGostergeKontrol(void);
void pompaCalistir(void);
void pompaDurdur(void);
void rezistansCalistir(void);
void rezistansDurdur(void);
void alarmLedveBuzzerAc(void);
void alarmLedveBuzzerKapa(void);
void SicaklikOkuveKontrolEt(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_ADC_Start(&hadc1);
	//TRANSACTIONS
	if(machineState==IDLE && HAL_GPIO_ReadPin(reed_role_input_GPIO_Port,reed_role_input_Pin)==1 && HAL_GPIO_ReadPin(su_microswitch_GPIO_Port,su_microswitch_Pin)==1 && startFlag == TRUE)
	{
		machineState = SUDOLDUR;
	}

	if(machineState == SUDOLDUR && totalAlinanSu > fincanSayisi * kahveMililitre)
	{
		machineState = PISIR;
	}

	if(machineState == PISIR && sicaklikFlag )
	{
		machineState = IDLE;
	}

	if(machineState == SUDOLDUR && HAL_GPIO_ReadPin(reed_role_input_GPIO_Port,reed_role_input_Pin)==0)
	{
		prevState = SUDOLDUR;
		machineState = HATA;
	}

	if(machineState == PISIR && HAL_GPIO_ReadPin(reed_role_input_GPIO_Port,reed_role_input_Pin)==0)
	{
		prevState = PISIR;
		machineState = HATA;
	}

	if(machineState == HATA && HAL_GPIO_ReadPin(reed_role_input_GPIO_Port,reed_role_input_Pin)==1)
	{
		machineState = prevState;
	}

	//ACTIONS

	if(machineState==IDLE)
	{
		fincanBoyutGostergeKontrol();
		fincanSayiGostergeKontrol();
		pompaDurdur();
		rezistansDurdur();
		sicaklikFlag = FALSE;
	}

	if(machineState==SUDOLDUR)
	{
		alarmLedveBuzzerKapa();
		startFlag = FALSE;
		pompaCalistir();
	}

	if(machineState==PISIR)
	{
		alarmLedveBuzzerKapa();
		totalAlinanSu = 0;
		pompaDurdur();
		rezistansCalistir();
		SicaklikOkuveKontrolEt();
	}

	if(machineState==HATA)
	{
		alarmLedveBuzzerAc();
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led_boyut_kucuk_Pin|led_boyut_orta_Pin|led_boyut_buyuk_Pin|led_uyari_Pin
                          |led_kahve_normal_Pin|led_kahve_kozde_Pin|led_fincan_4_Pin|led_fincan_3_Pin
                          |led_fincan_2_Pin|led_fincan_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, pompa_Pin|rezistans_Pin|buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : reed_role_input_Pin su_microswitch_Pin */
  GPIO_InitStruct.Pin = reed_role_input_Pin|su_microswitch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : flowmetre_Pin */
  GPIO_InitStruct.Pin = flowmetre_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(flowmetre_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led_boyut_kucuk_Pin led_boyut_orta_Pin led_boyut_buyuk_Pin led_uyari_Pin
                           led_kahve_normal_Pin led_kahve_kozde_Pin led_fincan_4_Pin led_fincan_3_Pin
                           led_fincan_2_Pin led_fincan_1_Pin */
  GPIO_InitStruct.Pin = led_boyut_kucuk_Pin|led_boyut_orta_Pin|led_boyut_buyuk_Pin|led_uyari_Pin
                          |led_kahve_normal_Pin|led_kahve_kozde_Pin|led_fincan_4_Pin|led_fincan_3_Pin
                          |led_fincan_2_Pin|led_fincan_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : buton_fincan_boyut_Pin start_Pin buton_mod_Pin buton_fincan_sayi_Pin */
  GPIO_InitStruct.Pin = buton_fincan_boyut_Pin|start_Pin|buton_mod_Pin|buton_fincan_sayi_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : pompa_Pin rezistans_Pin buzzer_Pin */
  GPIO_InitStruct.Pin = pompa_Pin|rezistans_Pin|buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == buton_fincan_sayi_Pin)
	{
		fincanSayisi++;
		if(fincanSayisi == 5)
			fincanSayisi = 1;
	}

	else if(GPIO_Pin == buton_mod_Pin)
	{
		pisirmeModu++;
		if(pisirmeModu == 2)
			pisirmeModu = 0;
	}
	else if(GPIO_Pin == buton_fincan_boyut_Pin)
	{
		kahveFincanBoyutu++;
		if(kahveFincanBoyutu == 3)
			kahveFincanBoyutu = 0;
	}

	else if(GPIO_Pin == start_Pin)
	{
		startFlag = TRUE;
	}

	else if(GPIO_Pin == flowmetre_Pin)
	{
		totalAlinanSu = totalAlinanSu + FLOWMETRE_PULSE_BASINA_MILILITRE;
	}



}

void fincanBoyutGostergeKontrol(void)
{
	switch(kahveFincanBoyutu)
	{
	case KUCUK:
		HAL_GPIO_WritePin(led_boyut_kucuk_GPIO_Port,led_boyut_kucuk_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(led_boyut_orta_GPIO_Port,led_boyut_orta_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_boyut_buyuk_GPIO_Port,led_boyut_buyuk_Pin,GPIO_PIN_RESET);
		kahveMililitre = 100;
	case ORTA:
		HAL_GPIO_WritePin(led_boyut_kucuk_GPIO_Port,led_boyut_kucuk_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_boyut_orta_GPIO_Port,led_boyut_orta_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(led_boyut_buyuk_GPIO_Port,led_boyut_buyuk_Pin,GPIO_PIN_RESET);
		kahveMililitre = 150;
	case BUYUK:
		HAL_GPIO_WritePin(led_boyut_kucuk_GPIO_Port,led_boyut_kucuk_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_boyut_orta_GPIO_Port,led_boyut_orta_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_boyut_buyuk_GPIO_Port,led_boyut_buyuk_Pin,GPIO_PIN_SET);
		kahveMililitre = 200;
	}
}
void fincanSayiGostergeKontrol(void)
{
	switch(fincanSayisi)
	{
	case 1:
		HAL_GPIO_WritePin(led_fincan_1_GPIO_Port,led_fincan_1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(led_fincan_2_GPIO_Port,led_fincan_2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_fincan_3_GPIO_Port,led_fincan_3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_fincan_4_GPIO_Port,led_fincan_4_Pin,GPIO_PIN_RESET);
	case 2:
		HAL_GPIO_WritePin(led_fincan_1_GPIO_Port,led_fincan_1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_fincan_2_GPIO_Port,led_fincan_2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(led_fincan_3_GPIO_Port,led_fincan_3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_fincan_4_GPIO_Port,led_fincan_4_Pin,GPIO_PIN_RESET);
	case 3:
		HAL_GPIO_WritePin(led_fincan_1_GPIO_Port,led_fincan_1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_fincan_2_GPIO_Port,led_fincan_2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_fincan_3_GPIO_Port,led_fincan_3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(led_fincan_4_GPIO_Port,led_fincan_4_Pin,GPIO_PIN_RESET);
	case 4:
		HAL_GPIO_WritePin(led_fincan_1_GPIO_Port,led_fincan_1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_fincan_2_GPIO_Port,led_fincan_2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_fincan_3_GPIO_Port,led_fincan_3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_fincan_4_GPIO_Port,led_fincan_4_Pin,GPIO_PIN_SET);
	}
}

void pompaCalistir(void)
{
	HAL_GPIO_WritePin(pompa_GPIO_Port,pompa_Pin,GPIO_PIN_SET);
}

void pompaDurdur(void)
{
	HAL_GPIO_WritePin(pompa_GPIO_Port,pompa_Pin,GPIO_PIN_RESET);
}

void rezistansCalistir(void)
{
	HAL_GPIO_WritePin(rezistans_GPIO_Port,rezistans_Pin,GPIO_PIN_SET);
}

void rezistansDurdur(void)
{
	HAL_GPIO_WritePin(rezistans_GPIO_Port,rezistans_Pin,GPIO_PIN_RESET);
}

void alarmLedveBuzzerAc(void)
{
	HAL_GPIO_WritePin(led_uyari_GPIO_Port,led_uyari_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);

}

void alarmLedveBuzzerKapa(void)
{
	HAL_GPIO_WritePin(led_uyari_GPIO_Port,led_uyari_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_RESET);
}
void SicaklikOkuveKontrolEt(void)
{
	 if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
	    {
	      uint16_t adcValue = HAL_ADC_GetValue(&hadc1);

	      // NTC'nin direncini hesaplama
	      float ntcResistance = (float)adcValue / 4095.0 * 3300.0;

	      // NTC'nin sıcaklık değerini hesaplama
	      float sicaklik = (ntcResistance - 10000.0) / 3380.0;

	      if (sicaklik > 95)
	    	  sicaklikFlag = TRUE;
	    }

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

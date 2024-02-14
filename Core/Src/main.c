/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "main.h"
#include <stdint.h>

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void GPIO_set_mode(int option);
uint8_t init_DHT11(void);
uint8_t check_response(void);
uint8_t read_dht11(void);

#define GPIO_INPUT_MODE		0
#define GPIO_OUTPUT_MODE	1

uint8_t data = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  init_DHT11();
  check_response();

  while (1) {

	  data = read_dht11();

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t init_DHT11(void) {

	/*Pulls down signal*/
	GPIO_set_mode(GPIO_OUTPUT_MODE);
	HAL_GPIO_WritePin(GPIO_DHT11_GPIO_Port, GPIO_DHT11_Pin, 0);
	/*Wait 18 ms*/
	HAL_Delay(18);
	/*Pull up signal*/
	//DHT11 is sending response (so now i have to put as input)
	GPIO_set_mode(GPIO_INPUT_MODE);

}

uint8_t check_response(void){

	uint8_t response = 0;

	if(!(HAL_GPIO_ReadPin(GPIO_DHT11_GPIO_Port, GPIO_DHT11_Pin))){
		HAL_Delay(0.08);
		if((HAL_GPIO_ReadPin(GPIO_DHT11_GPIO_Port, GPIO_DHT11_Pin))) response = 1; //dht11 is present
		else response = -1;
	}

	while ((HAL_GPIO_ReadPin(GPIO_DHT11_GPIO_Port, GPIO_DHT11_Pin)));

	return response;
}



uint8_t read_dht11(void){

	uint8_t i, j;

	response = 1;

	for (j = 0; j < 8; j++) {
		response = 2;
		/*Wait for the pin to go high*/
		while (!(HAL_GPIO_ReadPin(GPIO_DHT11_GPIO_Port, GPIO_DHT11_Pin))) {

			HAL_Delay(0.04);
			response = 3;

			if (!(HAL_GPIO_ReadPin(GPIO_DHT11_GPIO_Port, GPIO_DHT11_Pin))) // if the pin is low
			{
				i &= ~(1 << (7 - j));   // write 0
				response = 4;
			}
			else{
				i |= (1 << (7 - j));  // if the pin is high, write 1
				response = 5;
			}
			while ((HAL_GPIO_ReadPin(GPIO_DHT11_GPIO_Port, GPIO_DHT11_Pin))){
			}
		}
		return i;
	}

}

/**
  * @brief GPIO mode selection
  * @param option - mode to select (input, output...)
  * @retval None
  */
void GPIO_set_mode(int option) {

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_DHT11_Pin;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if (option == GPIO_INPUT_MODE) {
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	}

	if (option == GPIO_OUTPUT_MODE) {
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	}

	HAL_GPIO_Init(GPIO_DHT11_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : GPIO_DHT11_Pin */
  GPIO_set_mode(GPIO_OUTPUT_MODE);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

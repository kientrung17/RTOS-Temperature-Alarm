/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include <stdbool.h>
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "dwt_stm32_delay.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */


/* USER CODE BEGIN PV */
uint32_t tempCode;
uint8_t bitIndex;
uint8_t cmd;
uint8_t cmdli;
uint32_t code;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_5
uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;

int a;



void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < delay);
}

uint8_t DHT22_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT22_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
  microDelay (1300);   // wait for 1300us
  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT22_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

uint8_t DHT_Temp(){
	uint8_t temp;

	  if(DHT22_Start())
	  {
		RH1 = DHT22_Read(); // First 8bits of humidity
		RH2 = DHT22_Read(); // Second 8bits of Relative humidity
		TC1 = DHT22_Read(); // First 8bits of Celsius
		TC2 = DHT22_Read(); // Second 8bits of Celsius
		SUM = DHT22_Read(); // Check sum
		CHECK = RH1 + RH2 + TC1 + TC2;
		if (CHECK == SUM)
		{
		  if (TC1>127) // If TC1=10000000, negative temperature
		  {
			tCelsius = (float)TC2/10*(-1);
		  }
		  else
		  {
			tCelsius = (float)((TC1<<8)|TC2)/10;
		  }
		  tFahrenheit = tCelsius * 9/5 + 32;
		  RH = (float) ((RH1<<8)|RH2)/10;
		}
	  }
	 temp = (int)tCelsius;
	 return temp;
}

char temperature[3] = {48,48,'\0'};
void UART_SendTemp(uint8_t a)
{
	temperature[0] = a/10 + '0';
	temperature[1] = a%10 + '0';
	HAL_UART_Transmit(&huart2,(uint8_t*)temperature, 2, 1000);
	HAL_UART_Transmit(&huart2,(uint8_t*)'\n', 1, 1000);
}

typedef uint32_t TaskProfiler;

TaskHandle_t buzzer_handler,uart_handler,dht_handler;


TaskProfiler buzzerTaskProfiler, UARTTaskProfiler;

void vBuzzerTask(void *pvParameters);

void vUARTTask(void *pvParameters);

void vDHTTask(void *pvParametes);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  SSD1306_Init();
  SSD1306_GotoXY(10,10);

  HAL_TIM_Base_Start(&htim1);
  __HAL_TIM_SET_COUNTER(&htim1, 0);

  HAL_TIM_Base_Start(&htim2);

  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
  xTaskCreate(vDHTTask,
   		  	  "DHT Task",
   			  100,
   			  NULL,
   			  2,
			  &dht_handler);

  xTaskCreate(vBuzzerTask,
		  	  "Buzzer Task",
			  100,
			  NULL,
			  1,
			  &buzzer_handler);
  xTaskCreate(vUARTTask,
 		  	  "UART Task",
 			  100,
 			  NULL,
 			  2,
			  &uart_handler);

  vTaskStartScheduler();

  while (1)
  {


  }

}

uint8_t temp;
uint8_t alert = false;
uint8_t threshold = 100;
uint16_t pass;
uint8_t digit;
int biendemso = 0;

uint8_t bienxacthuc = 0;

char password[4]={'1','0','0','\0'};

TickType_t _500ms = pdMS_TO_TICKS(500);

void vBuzzerTask(void *pvParameters)
{
	while(1)
	{
		HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

void vUARTTask(void *pvParameters)
{
	while(1)
	{

		HAL_UART_Transmit(&huart2, (uint8_t*)"Temperature is ", 15, 1000);
		UART_SendTemp(temp);
		if(alert == true)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)" SOS\n", 5, 1000);
		}
		else
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)" Normal\n", 8, 1000);
		}
		vTaskDelay(_500ms);
	}
}
void vDHTTask(void *pvParametes)
{
	while(1)
	{

		temp = DHT_Temp();
		HAL_UART_Transmit(&huart2, (uint8_t*)"DHT Task\n", 9, 1000);
		if(temp > threshold)
		{
			alert = true;
			vTaskPrioritySet(buzzer_handler, 3);
			vTaskResume(buzzer_handler);


		}
		else
		{
			alert = false;
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
			vTaskSuspend(buzzer_handler);

		}
		vTaskDelay(_500ms);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
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
  HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DHT_Pin RS485_DE_Pin */
  GPIO_InitStruct.Pin = DHT_Pin|RS485_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_Pin */
  GPIO_InitStruct.Pin = IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_1)
  {
    if (__HAL_TIM_GET_COUNTER(&htim1) > 8000)
    {
      tempCode = 0;
      bitIndex = 0;
    }
    else if (__HAL_TIM_GET_COUNTER(&htim1) > 1700)
    {
      tempCode |= (1UL << (31-bitIndex));   // write 1
      bitIndex++;
    }
    else if (__HAL_TIM_GET_COUNTER(&htim1) > 1000)
    {
      tempCode &= ~(1UL << (31-bitIndex));  // write 0
      bitIndex++;
    }
    if(bitIndex == 32)
    {
      cmdli = ~tempCode; // Logical inverted last 8 bits
      cmd = tempCode >> 8; // Second last 8 bits
      if(cmdli == cmd) // Check for errors
      {
        code = tempCode; // If no bit errors
        // Do your main work HERE

        SSD1306_Clear();
        SSD1306_GotoXY (0,10);
        SSD1306_Puts("Temp : ", &Font_7x10,1);
        SSD1306_GotoXY (42,10);
        SSD1306_Puts((char*)temperature, &Font_7x10,1);
        SSD1306_GotoXY (0,20);
        SSD1306_Puts((char*)"Threshold : ", &Font_7x10,1);

        SSD1306_GotoXY(80, 20);
        SSD1306_Puts((char*)password, &Font_7x10, 1);
        SSD1306_GotoXY (0,50);

        switch (code)
        {
          case (16753245):
//            SSD1306_Puts ("CH-", &Font_7x10, 1);
          break;

          case (16736925):
//            SSD1306_Puts ("CH", &Font_7x10, 1);
		  SSD1306_GotoXY (0,40);
          SSD1306_Puts((char*)"New Threshold", &Font_7x10,1);
//          char password[4] = {'*','*','*','\0'};
          SSD1306_GotoXY (80,20);
          SSD1306_Puts((char*) password, &Font_7x10,1);
          bienxacthuc = 1;

           break;

          case (16769565):
//            SSD1306_Puts ("CH+", &Font_7x10, 1);
            break;

          case (16720605):
//            SSD1306_Puts ("|<<", &Font_7x10, 1);
            break;

          case (16712445):
//            SSD1306_Puts (">>|", &Font_7x10, 1);
            break;

          case (16761405):
//            SSD1306_Puts (">||", &Font_7x10, 1);
          break;

          case (16769055):
 //           SSD1306_Puts ("-", &Font_7x10, 1);
 //          break;

          case (16754775):
  //          SSD1306_Puts ("+", &Font_7x10, 1);
         break;

          case (16748655):
        //    SSD1306_Puts ("EQ", &Font_7x10, 1);
		  SSD1306_GotoXY (0,40);
          SSD1306_Puts((char*)"Threshold Success ", &Font_7x10,1);

          if(bienxacthuc == 1){
        	  int result;
        	  threshold = 0;
//          for (int i = 0; i < 3; i++) {
//
//           // Chuyển đổi ký tự sang số
//            digit = password[i] - '0';
//           // Nhân với hệ số mũ
//           threshold += digit * pow(10,2-i);
//            }
        	  threshold = ( password[0] - '0')*100 + ( password[1] - '0')*10 + ( password[2] - '0');
          result =1;
          }
          bienxacthuc = 0;
          biendemso = 0;

            break;

          case (16738455):
//		  SSD1306_GotoXY (0,40);
            SSD1306_Puts ("0", &Font_7x10, 1);
          if (bienxacthuc == 1 && biendemso <=2)
          {
        	  password[biendemso] = '0';
        	  biendemso++;
        	  SSD1306_GotoXY (80,20);
      	      SSD1306_Puts((char*) password, &Font_7x10,1);
          }
            break;

          case (16750695):
//            SSD1306_Puts ("100+", &Font_7x10, 1);
            break;

          case (16756815):
//            SSD1306_Puts ("200+", &Font_7x10, 1);
            break;

          case (16724175):
//		SSD1306_GotoXY (0,40);
            SSD1306_Puts ("1", &Font_7x10, 1);
          if (bienxacthuc == 1 && biendemso <=2)
          {
        	  password[biendemso] = '1';
        	  biendemso++;
        	  SSD1306_GotoXY (80,20);
      	      SSD1306_Puts((char*) password, &Font_7x10,1);
          }
            break;

          case (16718055):
//		SSD1306_GotoXY (0,40);
            SSD1306_Puts ("2", &Font_7x10, 1);
          if (bienxacthuc == 1 && biendemso <=2)
          {
        	  password[biendemso] = '2';
        	  biendemso++;
        	  SSD1306_GotoXY (80,20);
      	      SSD1306_Puts((char*) password, &Font_7x10,1);
          }
            break;

          case (16743045):
//		SSD1306_GotoXY (0,40);
            SSD1306_Puts ("3", &Font_7x10, 1);
          if (bienxacthuc == 1 && biendemso <=2)
          {
        	  password[biendemso] = '3';
        	  biendemso++;
        	  SSD1306_GotoXY (80,20);
      	      SSD1306_Puts((char*) password, &Font_7x10,1);
          }
            break;

          case (16716015):
//		SSD1306_GotoXY (0,40);
            SSD1306_Puts ("4", &Font_7x10, 1);
          if (bienxacthuc == 1 && biendemso <=2)
          {
        	  password[biendemso] = '4';
        	  biendemso++;
        	  SSD1306_GotoXY (80,20);
      	      SSD1306_Puts((char*) password, &Font_7x10,1);
          }
            break;

          case (16726215):
//		SSD1306_GotoXY (0,40);
            SSD1306_Puts ("5", &Font_7x10, 1);
          if (bienxacthuc == 1 && biendemso <=2)
          {
        	  password[biendemso] = '5';
        	  biendemso++;
        	  SSD1306_GotoXY (80,20);
      	      SSD1306_Puts((char*) password, &Font_7x10,1);
          }
            break;

          case (16734885):
//		SSD1306_GotoXY (0,40);
            SSD1306_Puts ("6", &Font_7x10, 1);
          if (bienxacthuc == 1 && biendemso <=2)
          {
        	  password[biendemso] = '6';
        	  biendemso++;
        	  SSD1306_GotoXY (80,20);
      	      SSD1306_Puts((char*) password, &Font_7x10,1);
          }
            break;

          case (16728765):
//		SSD1306_GotoXY (0,40);
            SSD1306_Puts ("7", &Font_7x10, 1);
          if (bienxacthuc == 1 && biendemso <=2)
          {
        	  password[biendemso] = '7';
        	  biendemso++;
        	  SSD1306_GotoXY (80,20);
      	      SSD1306_Puts((char*) password, &Font_7x10,1);
          }
            break;

          case (16730805):
//		SSD1306_GotoXY (0,40);
            SSD1306_Puts ("8", &Font_7x10, 1);
          if (bienxacthuc == 1 && biendemso <=2)
          {
        	  password[biendemso] = '8';
        	  biendemso++;
        	  SSD1306_GotoXY (80,20);
      	      SSD1306_Puts((char*) password, &Font_7x10,1);
          }
            break;

          case (16732845):
//		SSD1306_GotoXY (0,40);
            SSD1306_Puts ("9", &Font_7x10, 1);
          if (bienxacthuc == 1 && biendemso <=2)
          {
        	  password[biendemso] = '9';
        	  biendemso++;
        	  SSD1306_GotoXY (80,20);
      	      SSD1306_Puts((char*) password, &Font_7x10,1);
          }
            break;

          default :
            break;
        }
      SSD1306_UpdateScreen();
      }
    bitIndex = 0;
    }
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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

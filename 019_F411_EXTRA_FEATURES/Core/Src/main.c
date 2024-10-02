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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SLAVEID 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

uint8_t rxbuffer[100];
uint8_t txbuffer[100];

uint16_t registers[10]={1,255,0,4,3,0,5,2500,3524,55};

uint16_t Modbus_CRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < length; pos++)
    {
        crc ^= (uint16_t)data[pos];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
                crc >>= 1;
        }
    }
    return crc;
}
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 HAL_UART_Receive(&huart2, rxbuffer, sizeof(rxbuffer), 100);
	 if(rxbuffer[0]==1 && rxbuffer[1]== 0x03){
		 uint16_t start_Reg = rxbuffer[2]<<8 | rxbuffer[3];
		 uint16_t num_Reg = rxbuffer[4]<<8 | rxbuffer[5];

		 uint8_t response [5+2*num_Reg+2];
		 response[0] = SLAVEID;
		 response[1] = 0x03;
		 response[2] = 2*num_Reg;
		 for(int i=0; i<num_Reg;i++){
			 response[3+i*2] = (registers[start_Reg+i]>>8) & 0xFF ;
			 response[4+i*2] =	registers[start_Reg+i] & 0xFF ;
		 }

		 uint16_t CRC2 = Modbus_CRC16(response, 3+2*num_Reg);
		 response[3 + 2*num_Reg] = CRC2 & 0xFF;
		 response[4 + 2*num_Reg] = (CRC2>>8) & 0xFF;


		 int response_size = (3+2*num_Reg+2);
		 HAL_UART_Transmit(&huart2, response, response_size, 100);

	 }
	 else if(rxbuffer[0]==1 && rxbuffer[1]==0x10){
		 uint16_t start_Reg =(rxbuffer[2]<<8) | rxbuffer[3];
		 uint16_t num_Reg = (rxbuffer[4]<<8) | rxbuffer[5];
		 uint8_t num_of_databytes = rxbuffer[6];

		 for(int i=0;i<num_Reg;i++){
			 registers[start_Reg+i] = (rxbuffer[7+2*i])<<8 | rxbuffer[8+2*i];
		 }

		 //Preapering response
		 txbuffer[0]=SLAVEID;
		 txbuffer[1]=0x10;
		 txbuffer[2]=rxbuffer[2];
		 txbuffer[3]=rxbuffer[3];
		 txbuffer[4]=rxbuffer[4];
		 txbuffer[5]=rxbuffer[5];

		 uint16_t CRC2 = Modbus_CRC16(txbuffer, 6);
		 txbuffer[6] = CRC2 & 0xFF;
		 txbuffer[7] = (CRC2>>8) & 0xFF;

		 HAL_UART_Transmit(&huart2, txbuffer, 8, 100);
	 }
	  else if ((rxbuffer[0]==SLAVEID) && (rxbuffer[1]==0x06)){  	// EĞER SLAVE ADRESİ VE FONKSİYON 0x06 İSE
	 	  	  		{

	 	  	  		uint16_t address = (rxbuffer[2]<<8) | rxbuffer[3];
	 	  	  		uint16_t value = ((rxbuffer[4]<<8) | rxbuffer[5]);


	 	  	  		registers[address] = value;										// Register'a yazma


	 	  	  		 uint8_t response[8];
	 	  	  		for (int i = 0; i < 6; i++) {								// Cevabı hazırlama
	 	  	  		response[i] = rxbuffer[i];
	 	  	  		}


	 	  	  		uint16_t crc = Modbus_CRC16(response, 6);					// CRC hesaplama
	 	  	  		response[6] = crc & 0xFF;
	 	  	  		response[7] = (crc >> 8) & 0xFF;

	 	  	  		 // Cevabı gönder
	 	  	  		HAL_UART_Transmit(&huart2, response, 8, 1000);
	 	  	  			  }

	 	  	  	  }
	  else if (rxbuffer[0] == SLAVEID && rxbuffer[1] == 0x04) { // Slave ID ve Fonksiyon kodu kontrolü
	         uint16_t startAddress = (rxbuffer[2] << 8) | rxbuffer[3]; // Başlangıç adresi
	         uint16_t numRegisters = (rxbuffer[4] << 8) | rxbuffer[5]; // Okunacak register sayısı

	         uint8_t response[5 + 2 * numRegisters + 2]; // Cevap mesajı için buffer

	         // Cevap mesajının başlık kısmı
	         response[0] = SLAVEID;
	         response[1] = 0x04; // Function code (Read Input Registers)
	         response[2] = 2 * numRegisters; // Byte sayısı (her register 2 byte)

	         // Register verilerini ekle
	         for (int i = 0; i < numRegisters; i++) {
	             uint16_t value = registers[startAddress + i]; // Register değerini al
	             response[3 + i * 2] = (value >> 8) & 0xFF; // Üst byte
	             response[4 + i * 2] = value & 0xFF; // Alt byte
	         }

	         // CRC hesapla ve ekle
	         uint16_t crc = Modbus_CRC16(response, 3 + 2 * numRegisters);
	         response[3 + 2 * numRegisters] = crc & 0xFF; // CRC düşük byte
	         response[4 + 2 * numRegisters] = (crc >> 8) & 0xFF; // CRC yüksek byte

	         // Cevabı gönder
	         HAL_UART_Transmit(&huart2, response, 5 + 2 * numRegisters + 2, 1000);



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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8400;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

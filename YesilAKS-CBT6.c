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
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define NUMBER_OF_CELLS 12

const uint8_t REGISTER_ADDRESS[NUMBER_OF_CELLS] = {0x0C, 0x0E, 0x10,  0x16, 0x14, 0x18,  0x1A, 0x1E, 0x20,  0x22, 0x24, 0x28};
int voltage[NUMBER_OF_CELLS];
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
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
void synchronization();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rawData[2];
char buffer[50];
int combined;
int totalVoltage = 0.0;

float originalVoltage[12];


CAN_TxHeaderTypeDef   TxHeader;
uint8_t TotalData[24];
uint8_t CanTxData[8] = {11, 22, 33, 44,  55, 66, 77, 88};
uint32_t TxMailbox;
uint8_t cellFlag = 0;

void read_voltage(void) {
    uint8_t i;

    memset(buffer, 0, sizeof(buffer));

    sprintf(buffer, "*VOLTAJ BİLGİLERİ\r\n");

    for (i = 0; i < NUMBER_OF_CELLS; i++) {
    	cellFlag = i + 1;

        HAL_I2C_Mem_Read(&hi2c1, 0x08 << 1, REGISTER_ADDRESS[i], I2C_MEMADD_SIZE_8BIT, rawData, 2, HAL_MAX_DELAY);
//        combined = (rawData[0] << 8) | rawData[1];
//        voltage[i] = combined * 0.380 + 30;

		TotalData[cellFlag * 2] = rawData[0];
		TotalData[(cellFlag * 2) + 1] = rawData[1];


//        totalVoltage += voltage[i];
        sprintf(buffer, "Hücre %d Voltaj=%d\r\n", i + 1, voltage[i]);
        HAL_Delay(10);
    }

    sprintf(buffer, "Toplam Voltaj: %d\r\n", totalVoltage);
}

uint8_t TotalData2[24] = {0,0,1,2,3,4, 5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};


uint8_t counter2 = 2;
void syncPacket(uint8_t _packetNumber) {

	counter2 = 2;
	for(int i = (_packetNumber - 1) * 6; i < (_packetNumber) * 6 ; i++) {
		CanTxData[counter2] = TotalData[i];
		counter2++;
	}
}





// ************************************** RING BUFFER CODES ********************************************

#define BufferSize 750

typedef struct
{
    int buffer[BufferSize];
    int readNumber;
    int writeIndex;
    int readIndex;
} RingBuffer;


void initRingBuffer(RingBuffer *newBuffer)
{
    newBuffer->writeIndex = 0;
    newBuffer->readIndex = 0;
}

int counter = 10;
void write(RingBuffer *newBuffer, int readNumberParam)
{
    if ((newBuffer->writeIndex + 1) % BufferSize  != newBuffer->readIndex)
    {
        newBuffer->buffer[newBuffer->writeIndex] = readNumberParam;
        newBuffer->writeIndex = (newBuffer->writeIndex + 1) % BufferSize;
    }
    else
    {
        //printf("Buffer is FULL!\n");
    }
}

int read(RingBuffer *newBuffer)
{
    newBuffer->readNumber = -1;

    if (newBuffer->readIndex != newBuffer->writeIndex)
    {
        newBuffer->readNumber = newBuffer->buffer[newBuffer->readIndex];
        newBuffer->readIndex = (newBuffer->readIndex + 1) % BufferSize;
    }
    else
    {
        //printf("Buffer is EMPTY!\n");
    }

    return newBuffer->readNumber;
}
int deger;
uint8_t gecici[8] = {0, 3, 18, 0,  0, 0, 0, 0};
uint32_t getClock;


// ************************************** RING BUFFER CODES ********************************************

uint8_t sendData[8] = {0,3,18,0,55,66,77,88};
uint8_t receiveFeedback[4];
bool flag = 0;
bool synchronizationFlag = 0;
uint32_t sonGonderilmeSaati;



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	flag = 1;
	getClock = HAL_GetTick();
	synchronizationFlag = 1;

	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveFeedback, sizeof(receiveFeedback));
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}
// ######################### LORA ILE VERI GONDERME KISMI SON #####################################################








/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	RingBuffer newBuffer1;
		initRingBuffer(&newBuffer1);


		void synchronization() {
			__disable_irq();  // Kesmeleri devre dışı bırak
			for(int i = 0; i < 8; i++) {
				deger = read(&newBuffer1);
				if(deger != -1) {
					gecici[i] = deger;
				}
			}
			__enable_irq();
		}

		sendData[1] = sendData[1];
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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan);

  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0X406;




  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveFeedback, sizeof(receiveFeedback));
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  getClock = HAL_GetTick();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	read_voltage();
//
//	syncPacket(1);
//	HAL_CAN_AddTxMessage(&hcan, &TxHeader, CanTxData, &TxMailbox);
//	HAL_Delay(200);
//
//	syncPacket(2);
//	HAL_CAN_AddTxMessage(&hcan, &TxHeader, CanTxData, &TxMailbox);
//	HAL_Delay(200);
//
//	syncPacket(3);
//	HAL_CAN_AddTxMessage(&hcan, &TxHeader, CanTxData, &TxMailbox);
//	HAL_Delay(200);
//
//	syncPacket(4);
//	HAL_CAN_AddTxMessage(&hcan, &TxHeader, CanTxData, &TxMailbox);
//	HAL_Delay(200);
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);




	for(int i = 0; i < sizeof(sendData); i++) {
		write(&newBuffer1, sendData[i]);
	}

	sendData[3]++;

	  if(HAL_GetTick() - sonGonderilmeSaati > 5000 && flag == 0) {
		  // Yeniden bağlantı sağlandı mı diye kontrol et.
		  HAL_UART_Transmit(&huart3, gecici, sizeof(gecici), 1000);
		  sonGonderilmeSaati = HAL_GetTick();
		  flag = 0;
	  }


	  // Birinci Mesaj Gönderme
	  if(synchronizationFlag == 1) {
		  synchronization();
		  HAL_UART_Transmit(&huart3, gecici, sizeof(gecici), 1000);
		  sonGonderilmeSaati = HAL_GetTick();

		  flag = 0;
		  synchronizationFlag = 0;
		  HAL_Delay(450);
			  // İkinci Mesaj Gönderme
		  if(synchronizationFlag == 1) {
			  synchronization();
			  HAL_UART_Transmit(&huart3, gecici, sizeof(gecici), 1000);
			  sonGonderilmeSaati = HAL_GetTick();

			  flag = 0;
			  synchronizationFlag = 0;
			  HAL_Delay(450);
				  // Üçüncü Mesaj Gönderme
			  if(synchronizationFlag == 1) {
				  synchronization();
				  HAL_UART_Transmit(&huart3, gecici, sizeof(gecici), 1000);
				  sonGonderilmeSaati = HAL_GetTick();

				  flag = 0;
				  synchronizationFlag = 0;
				  HAL_Delay(450);
			  }

		  }
	  }
	  else {
		  HAL_Delay(450);
	  }





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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  huart1.Init.BaudRate = 9600;
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

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
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

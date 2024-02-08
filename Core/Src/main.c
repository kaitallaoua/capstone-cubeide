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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void select_adc_channel(int channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    //sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    switch (channel)
    {
        case 0:
            sConfig.Channel = ADC_CHANNEL_0;
              sConfig.Rank = 1;

              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;

        case 1:
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
              sConfig.Channel = ADC_CHANNEL_1;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 2:
              sConfig.Channel = ADC_CHANNEL_2;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 3:
              sConfig.Channel = ADC_CHANNEL_3;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 4:
              sConfig.Channel = ADC_CHANNEL_4;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 5:
              sConfig.Channel = ADC_CHANNEL_5;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 6:
              sConfig.Channel = ADC_CHANNEL_6;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 7:
              sConfig.Channel = ADC_CHANNEL_7;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 8:
              sConfig.Channel = ADC_CHANNEL_8;
              sConfig.Rank = 9;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 9:
              sConfig.Channel = ADC_CHANNEL_9;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 10:
              sConfig.Channel = ADC_CHANNEL_10;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 11:
              sConfig.Channel = ADC_CHANNEL_11;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 12:
              sConfig.Channel = ADC_CHANNEL_12;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 13:
              sConfig.Channel = ADC_CHANNEL_13;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
              /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
              */
        case 14:
              sConfig.Channel = ADC_CHANNEL_14;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
        case 15:
              sConfig.Channel = ADC_CHANNEL_15;
              sConfig.Rank = 1;
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                Error_Handler();
              }
              break;
        default:
            break;
    }
}
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n~ SD card demo by kiwih ~\r\n\r\n");

    //osDelay(1000);
  HAL_Delay(1000);
//a short delay is important to let the SD card settle

  //some variables for FatFs
  FATFS FatFs; 	//Fatfs handle
  FIL fil; 		//File handle
  FRESULT fres; //Result after operations

  //Open the file system
  fres = f_mount(&FatFs, "", 1); //1=mount now

  printf("just after mount");
  if (fres != FR_OK) {
	printf("f_mount error (%i)\r\n", fres);
	while(1);
  }

  printf("mounted");

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
	printf("f_getfree error (%i)\r\n", fres);
	while(1);
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

  //Now let's try to open file "test.txt"
  fres = f_open(&fil, "test.txt", FA_READ);
  if (fres != FR_OK) {
	printf("f_open error (%i)\r\n");
	while(1);
  }
  printf("I was able to open 'test.txt' for reading!\r\n");

  //Read 30 bytes from "test.txt" on the SD card
  BYTE readBuf[30];

  //We can either use f_read OR f_gets to get data out of files
  //f_gets is a wrapper on f_read that does some string formatting for us
  TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
  if(rres != 0) {
	printf("Read string from 'test.txt' contents: %s\r\n", readBuf);
  } else {
	printf("f_gets error (%i)\r\n", fres);
  }

  //Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);

  //Now let's try and write a file "write.txt"
  fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if(fres == FR_OK) {
	printf("I was able to open 'write.txt' for writing\r\n");
  } else {
	printf("f_open error (%i)\r\n", fres);
  }

  //Copy in a string
  strncpy((char*)readBuf, "a new file is made!", 19);
  UINT bytesWrote;
  fres = f_write(&fil, readBuf, 19, &bytesWrote);
  if(fres == FR_OK) {
	printf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
  } else {
	printf("f_write error (%i)\r\n");
  }

  //Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);

  //We're done, so de-mount the drive
  f_mount(NULL, "", 0);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  //uint32_t channel_val[16];




/*
	uint32_t channel_val;
  for(;;)
  {


	  // ch 2,3 dont work

	  // ch 7 works
	   // ch8 coupled to 7, reads nothing when shorted on its own


	  printf("-------------------------------\r\n");
	  for (uint16_t i = 0; i < 16; i++) {

		    select_adc_channel(i);
		  // Get each ADC value from the group (2 channels in this case)
		  HAL_ADC_Start(&hadc1);
		  // Wait for regular group conversion to be completed
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		  channel_val = HAL_ADC_GetValue(&hadc1);

		  printf("ADC channel [%u] value: %u\r\n", (uint16_t) i, (uint16_t) channel_val);
		  //HAL_ADC_Stop(&hadc1);
	  }



    osDelay(1000);
  }

  */
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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

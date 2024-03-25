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
#include "sdcard.h"
#include <string.h>
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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

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
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
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

// https://controllerstech.com/create-1-microsecond-delay-stm32/
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

uint16_t max(uint16_t arr[], int len)
{
	uint16_t i;

    // Initialize maximum element
    uint16_t max = arr[0];

    // Traverse array elements
    // from second and compare
    // every element with current max
    for (i = 1; i < len; i++)
        if (arr[i] > max)
            max = arr[i];

    return max;
}

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


void turn_on_5v_plane(void) {
	// turn on +5v plane
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
}

void turn_off_5v_plane(void) {

	// turn off +5v plane
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
}


void preform_opamp_measurement_log_to_sd(void) {
	char adc_buf[15];

	// dont need to convert to voltages here, since we divide at the end for gain

	const uint16_t NUM_OF_SAMPLES = 80;

	uint16_t source_sine_samples[NUM_OF_SAMPLES];
	uint16_t peak_source_sine;

	uint16_t test_sine_samples[NUM_OF_SAMPLES];
	uint16_t peak_test_sine;

	uint16_t noshd_gain; // percentage
	uint16_t mel_gain;
	uint16_t al_gain;

	for (uint16_t i = 0; i <= 3; i++) {

		 for (uint8_t j = 0; j < NUM_OF_SAMPLES; j++) {

			  select_adc_channel(i);
			  // Get each ADC value from the group (2 channels in this case)
			  HAL_ADC_Start(&hadc1);
			  // Wait for regular group conversion to be completed
			  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			  if (i == 0) {
				  source_sine_samples[j] = HAL_ADC_GetValue(&hadc1);
			  } else {
				  test_sine_samples[j] = HAL_ADC_GetValue(&hadc1);
			  }

			  delay_us(80); // ~160 Hz sine wave, try to sample it evenly with 80 samples as 1/(160 Hz * 80) ~ 80us

		 }

		 switch (i) {
		 	 case 0:
		 		 peak_source_sine = max(source_sine_samples, NUM_OF_SAMPLES);
		 		 printf("SINE source peak quantized: %u\r\n", peak_source_sine);
		 		 break;
		 	 case 1: // noshd
		 		 peak_test_sine = max(test_sine_samples, NUM_OF_SAMPLES);

		 		noshd_gain = (uint16_t) (( ((float) peak_test_sine) / ((float) peak_source_sine) ) * 100);
		 		 printf("SINE noshd gain %%: %u\r\n", noshd_gain);


		 		break;
		 	 case 2:
		 		 peak_test_sine = max(test_sine_samples, NUM_OF_SAMPLES);

			 		mel_gain = (uint16_t) (( ((float) peak_test_sine) / ((float) peak_source_sine) ) * 100);
			 		 printf("SINE mel gain %%: %u\r\n", mel_gain);
		 		break;

		 	 case 3:
		 		 peak_test_sine = max(test_sine_samples, NUM_OF_SAMPLES);

			 		al_gain = (uint16_t) (( ((float) peak_test_sine) / ((float) peak_source_sine) ) * 100);
			 		 printf("SINE al gain %%: %u\r\n", al_gain);
		 		break;

		 }

	}
  snprintf(adc_buf, 15, "%u,%u,%u,", noshd_gain, mel_gain, al_gain);
  write_sdcard_file(adc_buf);
}


void preform_vref_measurement_log_to_sd(void) {
	char adc_buf[40];
	uint16_t quantized_vref_val;

	for (uint16_t i = 4; i <= 6; i++) {

		  select_adc_channel(i);
		  // Get each ADC value from the group (2 channels in this case)
		  HAL_ADC_Start(&hadc1);
		  // Wait for regular group conversion to be completed
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		  quantized_vref_val = HAL_ADC_GetValue(&hadc1);

		  if (i == 4) { printf("VREF NOSHD quantized val: %u\r\n", quantized_vref_val);}
		  if (i == 5) { printf("VREF MEL quantized val: %u\r\n", quantized_vref_val);}
		  if (i == 6) { printf("VREF AL quantized val: %u\r\n", quantized_vref_val);}


		  snprintf(adc_buf, 40, "%u,", quantized_vref_val);
		  write_sdcard_file(adc_buf);

	}

}

void read_lm35(void) {
		char adc_buf[40];

		uint16_t quantized_lm35_v;

	  select_adc_channel(9);
	  // Get each ADC value from the group (2 channels in this case)
	  HAL_ADC_Start(&hadc1);
	  // Wait for regular group conversion to be completed
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  quantized_lm35_v = HAL_ADC_GetValue(&hadc1);

	  printf("LM35 quantized volt: %u\r\n", quantized_lm35_v);
	  snprintf(adc_buf, 40, "%u,", quantized_lm35_v);
	  write_sdcard_file(adc_buf);
}

void preform_opto_measurement_log_to_sd(void) {
	char adc_buf[15];

	// dont need to convert to voltages here, since we divide at the end for gain

	const uint16_t NUM_OF_SAMPLES = 80;

	uint16_t forward_opto_current[NUM_OF_SAMPLES];
	uint16_t peak_forward_opto_current;

	uint16_t emitter_opto_current[NUM_OF_SAMPLES];
	uint16_t peak_emitter_opto_current;

	uint16_t noshd_ctr; // percentage
	uint16_t mel_ctr;
	uint16_t al_ctr;

	for (uint16_t i = 10; i <= 15; i++) {

		 for (uint8_t j = 0; j < NUM_OF_SAMPLES; j++) {

			  select_adc_channel(i);
			  // Get each ADC value from the group (2 channels in this case)
			  HAL_ADC_Start(&hadc1);
			  // Wait for regular group conversion to be completed
			  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

			  // if adc channel number even
			  // if moving channels around have to change that here
			  if (i % 2 == 0) {
				  // its forward
				  forward_opto_current[j] = HAL_ADC_GetValue(&hadc1);
			  } else {
				  // else its emitter
				  emitter_opto_current[j] = HAL_ADC_GetValue(&hadc1);
			  }

			  delay_us(80); // ~160 Hz sine wave, try to sample it evenly with 80 samples as 1/(160 Hz * 80) ~ 80us

		 }

		 switch (i) {
		 	 case 10:
		 		peak_forward_opto_current = max(forward_opto_current, NUM_OF_SAMPLES);
		 		 printf("OPTO noshd forward current peak: %u\r\n", peak_forward_opto_current);
		 		 break;
		 	 case 11:
		 		peak_emitter_opto_current = max(emitter_opto_current, NUM_OF_SAMPLES);
		 		 printf("OPTO noshd emitter current peak: %u\r\n", peak_emitter_opto_current);

		 		// now we have all noshd data, find ctr = emitter/forward * 50 for our resistor selection

		 		noshd_ctr = (uint16_t) (( ((float) peak_emitter_opto_current) / ((float) peak_forward_opto_current) ) * 50);
		 		 printf("OPTO noshd ctr %%: %u\r\n", noshd_ctr);

		 		break;
		 	 case 12:
		 		peak_forward_opto_current = max(forward_opto_current, NUM_OF_SAMPLES);
		 		 printf("OPTO mel forward current peak: %u\r\n", peak_forward_opto_current);
		 		 break;
		 	 case 13:
		 		peak_emitter_opto_current = max(emitter_opto_current, NUM_OF_SAMPLES);
		 		 printf("OPTO mel emitter current peak: %u\r\n", peak_emitter_opto_current);

		 		// now we have all noshd data, find ctr = emitter/forward * 50 for our resistor selection

		 		mel_ctr = (uint16_t) (( ((float) peak_emitter_opto_current) / ((float) peak_forward_opto_current) ) * 50);
		 		 printf("OPTO mel ctr %%: %u\r\n", mel_ctr);

		 		break;
		 	 case 14:
		 		peak_forward_opto_current = max(forward_opto_current, NUM_OF_SAMPLES);
		 		 printf("OPTO al forward current peak: %u\r\n", peak_forward_opto_current);
		 		 break;
		 	 case 15:
		 		peak_emitter_opto_current = max(emitter_opto_current, NUM_OF_SAMPLES);
		 		 printf("OPTO al emitter current peak: %u\r\n", peak_emitter_opto_current);

		 		// now we have all noshd data, find ctr = emitter/forward * 50 for our resistor selection

		 		al_ctr = (uint16_t) (( ((float) peak_emitter_opto_current) / ((float) peak_forward_opto_current) ) * 50);
		 		 printf("OPTO al ctr %%: %u\r\n", al_ctr);

		 		break;
		 }

	}
  snprintf(adc_buf, 15, "%u,%u,%u,", noshd_ctr, mel_ctr, al_ctr);
  write_sdcard_file(adc_buf);
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
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


  //a short delay is important to let the SD card settle
  HAL_Delay(1000);

  HAL_TIM_Base_Start(&htim1);

  // obc enable interrupt listen
  //HAL_I2C_EnableListen_IT(&hi2c1);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 500);
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
  hi2c1.Init.OwnAddress1 = 36;
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
  htim1.Init.Prescaler = 24-1;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOSFET_PWR_GPIO_Port, MOSFET_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOSFET_PWR_Pin */
  GPIO_InitStruct.Pin = MOSFET_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOSFET_PWR_GPIO_Port, &GPIO_InitStruct);

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

	turn_on_5v_plane();


	//--------------------------- start cycle

	mount_sdcard();
	print_sdcard_stats();

	open_sdcard_file_write("sample");

	// may need to keep small or increase max size in write function if this gets too long
	write_sdcard_file("op_noshd_p,op_mel_p,op_al_p,vref_noshd,vref_mel,vref_al,lm35,opto_noshd,opto_mel,opto_al\r\n");

	const uint32_t num_samples = 450; // 2sec 450 samples = 15minutes


	for(int cnt = 0; cnt < num_samples; cnt++)
	{

		// order is very important here, since it correlates to order of data written to csv
		preform_opamp_measurement_log_to_sd();
		printf("------------------------------\r\n");
		preform_vref_measurement_log_to_sd();
		printf("------------------------------\r\n");
		read_lm35();
		printf("------------------------------\r\n");
		preform_opto_measurement_log_to_sd();
		printf("------------------------------\r\n");

		// new row
		write_sdcard_file("\r\n");
		osDelay(2000);
	}

	printf("all done\r\n");

	close_sdcard_file();

	// always do this after testing is done so if power is cut, no data is lost
	unmount_sdcard();


	//------------------------- end cycle

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
	  printf("Bricked");
	  HAL_Delay(1000);
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

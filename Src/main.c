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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include "info.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX(a,b) \
	({ __typeof__ (a) _a = (a); \
	   __typeof__ (b) _b = (b); \
	 _a > _b ? _a : _b; })

#define MIN(a,b) \
	({ __typeof__ (a) _a = (a); \
	   __typeof__ (b) _b = (b); \
	 _a < _b ? _a : _b; })

#define DATA_SIZE (200)
#define PI (3.1415926)
#define WSz (256)
#define WSzf (256.000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;
DMA_HandleTypeDef hdma_dac_ch2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for receiveTask */
osThreadId_t receiveTaskHandle;
const osThreadAttr_t receiveTask_attributes = {
  .name = "receiveTask",
  .stack_size = 900 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for commandTask */
osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
  .name = "commandTask",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for captureTask */
osThreadId_t captureTaskHandle;
const osThreadAttr_t captureTask_attributes = {
  .name = "captureTask",
  .stack_size = 300 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for genQueue01 */
osMessageQueueId_t genQueue01Handle;
const osMessageQueueAttr_t genQueue01_attributes = {
  .name = "genQueue01"
};
/* Definitions for UARTSem01 */
osSemaphoreId_t UARTSem01Handle;
const osSemaphoreAttr_t UARTSem01_attributes = {
  .name = "UARTSem01"
};
/* USER CODE BEGIN PV */
uint8_t idx_1 = 0;
uint8_t rx_char_1 = 0;

uint8_t Rx_data_1[1];
uint8_t Rx_data_2[1];
uint8_t Rx_flag_1 =0;
uint8_t Rx_flag_2 =0;
uint8_t Rx_Buffer_1[DATA_SIZE]={0};
uint8_t Rx_Buffer_2[DATA_SIZE]={0};
uint8_t Ch1_flag=0;
uint8_t Ch2_flag=0;

uint8_t Capture_flag=0;
uint16_t Received_Waveform[20000]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void Start_receiveTask(void *argument);
void Start_commandTask(void *argument);
void StartCaptureTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart == &huart1){
		HAL_UART_Receive_IT(&huart1, &rx_char_1, 1);
		Rx_Buffer_1[idx_1] = rx_char_1;
		idx_1++;
		if(rx_char_1 == '\r'){
			Rx_flag_1 = 1;
		}
	}
	else if (huart == &huart2)
		Rx_flag_2 = 1;
}

void Update_DMA(COMMAND *running_command, uint16_t wave[], uint16_t wave_ch1[], uint16_t wave_ch2[], uint16_t wave_size, uint16_t *scale) {
    if (running_command->channel == 1) {
    	HAL_TIM_Base_Stop(&htim2);
    	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
    	if(running_command->freq >1000){ // high speed mode
			for(int i=0; i<25; i++){
				wave_ch1[i] = wave[i*10]; // decimate signal
			}
			*scale = (int) 10*10000000.0 / (running_command->freq*WSz)-1;
			__HAL_TIM_SET_AUTORELOAD(&htim2, *scale);
			HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, wave_ch1, 25, DAC_ALIGN_12B_R);
			HAL_TIM_Base_Start(&htim2);
			return;
		}
    	for (int i = 0; i < 256; i++) {
            wave_ch1[i] = wave[i];
        }
        __HAL_TIM_SET_AUTORELOAD(&htim2, *scale);

        HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, wave_ch1, 256, DAC_ALIGN_12B_R);
        HAL_TIM_Base_Start(&htim2);
    }
    else if (running_command->channel == 2) {
    	HAL_TIM_Base_Start(&htim4);
    	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_2);

    	for (int i = 0; i < 256; i++) {
            wave_ch2[i] = wave[i];
        }
        __HAL_TIM_SET_AUTORELOAD(&htim4, *scale);

		HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_2, wave_ch2, 256, DAC_ALIGN_12B_R);
        HAL_TIM_Base_Start(&htim2);
    }
    return;
}
void process_cmd(char name1[], char name2[], char cmd[], char params[]){
	COMMAND command;
	uint8_t rx_buffer[DATA_SIZE] = {0};
	char sel[4] = {0};
	uint8_t on = 0;
	int led = 0;

	if(strcmp(cmd, "msg") == 0 || strcmp(cmd, "MSG") == 0){
		strcat(params, "\r\n");
		HAL_UART_Transmit(&huart2, params, DATA_SIZE, 1000);
	}
	else if(strcmp(cmd, "led") == 0||strcmp(cmd, "LED") == 0){
		sscanf(params, "%3s %d", sel, &led);
		if(sel[1] == 'n' || sel[1] == 'N'){
			on = GPIO_PIN_RESET;
		}
		else if(sel[1] == 'f'||sel[1] == 'F'){
			on = GPIO_PIN_SET;
		}
		if(led == 2){
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, on);
		}
		else if(led == 3){
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, on);
		}
	}
	else if(strcmp(cmd, "cap") == 0||strcmp(cmd, "CAP") == 0){
		Capture_flag = 1; // this will be read by Cap task
	}
	else if(strcmp(cmd, "gen") == 0||strcmp(cmd, "GEN") == 0){
		if(sscanf(params, "%d %c %f %f %f %d", &command.channel, &command.type,
			  &command.freq, &command.minv, &command.maxv, &command.noise) !=6){
			snprintf(rx_buffer, DATA_SIZE, "failed because incorrect size of param \r\n", command.channel);
			HAL_UART_Transmit(&huart2, rx_buffer, DATA_SIZE, 1000);
			return;
		}
		command.type = toupper(command.type); // put type to upper
		//------------------------------------------------------------------------------------
		// verify generation
		//------------------------------------------------------------------------------------
		if(command.channel != 1 && command.channel != 2){
			snprintf(rx_buffer, DATA_SIZE, "failed because channel = %d \r\n", command.channel);
			HAL_UART_Transmit(&huart2, rx_buffer, DATA_SIZE, 1000);
			return;
		}
		if(command.type != 'R' && command.type != 'T' && command.type != 'S' && command.type != 'A'){
			snprintf(rx_buffer, DATA_SIZE, "failed because type = %c \r\n", command.type);
			HAL_UART_Transmit(&huart2, rx_buffer, DATA_SIZE, 1000);
			return;
		}
		if(command.freq > 10000 || (command.freq < 0.5 && command.freq != 0) ){
			snprintf(rx_buffer, DATA_SIZE, "failed because frequency = %f \r\n", command.freq);
			HAL_UART_Transmit(&huart2, rx_buffer, DATA_SIZE, 1000);
			return;
		}
		if(command.minv < 0 || command.minv > 3.3){
			snprintf(rx_buffer, DATA_SIZE, "failed because minv = %f \r\n", command.minv);
			HAL_UART_Transmit(&huart2, rx_buffer, DATA_SIZE, 1000);
			return;
		}
		if(command.maxv < command.maxv || command.maxv > 3.3){
			snprintf(rx_buffer, DATA_SIZE, "failed because maxv = %f \r\n", command.maxv);
			HAL_UART_Transmit(&huart2, rx_buffer, DATA_SIZE, 1000);
			return;
		}
		if(command.noise < 0 || command.maxv > 12){
			snprintf(rx_buffer, DATA_SIZE, "failed because noise = %d \r\n", command.noise);
			HAL_UART_Transmit(&huart2, rx_buffer, DATA_SIZE, 1000);
			return;
		}
		//------------------------------------------------------------------------------------
		snprintf(rx_buffer, DATA_SIZE,"\r\n gen = %d, chann = %d, type = %c, freq = %f, minv = %f, maxv = %f, noise = %d \r\n",
			  command.gen, command.channel, command.type, command.freq,
			  command.minv, command.maxv, command.noise);
		HAL_UART_Transmit(&huart2, rx_buffer, DATA_SIZE, 1000);
		osMessageQueuePut(genQueue01Handle, &command, 1U, 100);
	}
	return;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim4);
  HAL_UART_Receive_IT(&huart2, Rx_data_2, 1);
  HAL_UART_Receive_IT(&huart1, Rx_data_1, 1);
  uint8_t rx_d[50] = "start\r\n";
  HAL_UART_Transmit(&huart2, rx_d, sizeof(rx_d), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of UARTSem01 */
  UARTSem01Handle = osSemaphoreNew(1, 1, &UARTSem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of genQueue01 */
  genQueue01Handle = osMessageQueueNew (15, sizeof(struct COMMANDS_STRUCT), &genQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of receiveTask */
  receiveTaskHandle = osThreadNew(Start_receiveTask, NULL, &receiveTask_attributes);

  /* creation of commandTask */
  commandTaskHandle = osThreadNew(Start_commandTask, NULL, &commandTask_attributes);

  /* creation of captureTask */
  captureTaskHandle = osThreadNew(StartCaptureTask, NULL, &captureTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Noise wave generation on DAC OUT1
  */
  if (HAL_DACEx_NoiseWaveGenerate(&hdac1, DAC_CHANNEL_1, DAC_LFSRUNMASK_BIT0) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Noise wave generation on DAC OUT2
  */
  if (HAL_DACEx_NoiseWaveGenerate(&hdac1, DAC_CHANNEL_2, DAC_LFSRUNMASK_BIT0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 101-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED2_Pin|LED3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/* USER CODE BEGIN Header_Start_receiveTask */
/**
* @brief Function implementing the receiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_receiveTask */
void Start_receiveTask(void *argument)
{
  /* USER CODE BEGIN Start_receiveTask */
  char myname[] = "quinn";
  char myname_1[] = "jack";
  //uint8_t idx_1 = 0;
  uint8_t idx_2 = 0;
  uint8_t clear_flag_1 = 0;
  uint8_t clear_flag_2 = 0;

  char nl[4] = "\r\n";
  char name1[32] = {0};
  char name2[32] = {0};
  char cmd[32] = {0};
  char params[DATA_SIZE] = {0};
  uint8_t buffer[DATA_SIZE] = {0};

  uint8_t rx_char = 0;
  //uint8_t rx_char_1 = 0;



  uint8_t state =0;
//COMMAND command;
	/* Infinite loop */
  for(;;)
  {
	  osDelay(1);
	  if(Rx_flag_1 == 0 && Rx_flag_2 == 0){
		  osDelay(1);
		  continue;
	  }

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Read UART 1
//-------------------------------------------------------------------------------------------------------------------------------------------------

	  if(Rx_flag_1 == 1){
		  Rx_flag_1 = 0;
		  if(sscanf(Rx_Buffer_1, "<%31[^>] > <%31[^>] > <%31[^>] > <%199[^>] >", name1, name2, cmd, params) == 4) {
			  if(strcmp(myname_1, name2)!=0){
				  HAL_UART_Transmit(&huart1, Rx_Buffer_1, DATA_SIZE, 1000);
			  }
			  else{
				  process_cmd(name1, name2, cmd, params);
				  memset(name1, 0, sizeof(name1[0]) * 32);
				  memset(name2, 0, sizeof(name1[0]) * 32);
				  memset(cmd, 0, sizeof(name1[0]) * 32);
				  memset(params, 0, sizeof(name1[0]) * DATA_SIZE);
			  }
		  }
		  else{
			  memset(buffer, 0, sizeof(buffer[0]) * DATA_SIZE); // wipe Rx_command
			  strcpy(buffer,"\r\ninvalid command by size! 01\r\n");
			  HAL_UART_Transmit(&huart2, buffer, DATA_SIZE, 1000);
		  }
		  idx_1 = 0;
		  memset(buffer, 0, sizeof(buffer[0]) * DATA_SIZE); // wipe Rx_command
		  memset(Rx_Buffer_1, 0, sizeof(Rx_Buffer_1[0]) * DATA_SIZE); // wipe Rx_command
	  }
//-------------------------------------------------------------------------------------------------------------------------------------------------
// Read UART 2
//-------------------------------------------------------------------------------------------------------------------------------------------------
	  if(Rx_flag_2 == 1){
		  HAL_UART_Receive_IT(&huart2, &rx_char, 1);
		  HAL_UART_Transmit(&huart2, &rx_char, 1, 10);
		  Rx_Buffer_2[idx_2] = rx_char;
		  idx_2++;
		  Rx_flag_2 = 0;
		  if(rx_char != '\r'){
			  continue;
		  }
		  HAL_UART_Transmit(&huart2, nl, 4, 100);
		  if(sscanf(Rx_Buffer_2, "<%31[^>] > <%31[^>] > <%199[^>] >",name2, cmd, params) == 3) {
			  if(strcmp(myname, name2)!=0){
				  //add <myname>
				  memset(buffer, 0, sizeof(buffer[0]) * DATA_SIZE);
				  strcat(buffer,"<");
				  strcat(buffer, myname);
				  strcat(buffer,"> ");
				  strcat(buffer, Rx_Buffer_2);
				  HAL_UART_Transmit(&huart1, buffer, DATA_SIZE, 1000);
			  }
			  else{
				  process_cmd(name1, name2, cmd, params);
				  memset(name1, 0, sizeof(name1[0]) * 32);
				  memset(name2, 0, sizeof(name1[0]) * 32);
				  memset(cmd, 0, sizeof(name1[0]) * 32);
				  memset(params, 0, sizeof(name1[0]) * DATA_SIZE);
			  }
		  }
		  else{
			  memset(buffer, 0, sizeof(buffer[0]) * DATA_SIZE); // wipe Rx_command
			  strcpy(buffer,"\r\ninvalid command by size! 02\r\n");
			  HAL_UART_Transmit(&huart2, buffer, DATA_SIZE, 1000);
		  }
		  idx_2 = 0;
		  memset(buffer, 0, sizeof(buffer[0]) * DATA_SIZE); // wipe Rx_command
		  memset(Rx_Buffer_2, 0, sizeof(Rx_Buffer_2[0]) * DATA_SIZE); // wipe Rx_command
	  }
  }
  /* USER CODE END Start_receiveTask */
}

/* USER CODE BEGIN Header_Start_commandTask */
/**
* @brief Function implementing the commandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_commandTask */
void Start_commandTask(void *argument)
{
  /* USER CODE BEGIN Start_commandTask */
  COMMAND running_command;
  float val =0.0;
  uint16_t wave[256] = {0};
  uint16_t wave_ch1[256]={0};
  uint16_t wave_ch2[256]={0};
  uint32_t scale =1000;
  double min =0;
  double max =0;
	/* Infinite loop */
  for(;;)
  {
	if(osMessageQueueGetCount(genQueue01Handle) != 0){
		osMessageQueueGet(genQueue01Handle, &running_command, 0, 100);
		if(running_command.freq == 0)
		{
			for(int i = 0; i < WSz; i++)
			{
					wave[i] = (uint16_t)(4095*running_command.minv/3.3);
			}

			scale = 100; // does not matter, just not -1
			Update_DMA(&running_command, wave, wave_ch1, wave_ch2, 256, &scale);
			continue;
		}

		switch(running_command.type){
		case('S'):
			for(int j=0; j<256; j++){
				val = (sin(j*2*PI/256.0)/2.0 + 0.5)*4095.0 * (running_command.maxv - running_command.minv)/3.3 + 4095.0*running_command.minv/3.3;
				wave[j] = (int) val;
			}
			scale =(int) 10000000 / (running_command.freq*WSz)-1;
		break;
		case('T'):
			for (int i=0; i<WSz; i++)
				{
					val = (abs(i-WSz/2.0)*(4095.0/WSzf)*((running_command.maxv - running_command.minv)/3.3) + 4095.0*running_command.minv/3.3);
					wave[i] = (int) val;
				}
			scale =(int) 10000000 / (running_command.freq*WSz)-1;
			break;
		case('R'):
			for (int i=0; i<WSz; i++)
				{
					if(i>WSz/2){
						val = 4095.0/3.3*running_command.minv; // minv
					}
					else{
						val = 4095.0/3.3*running_command.maxv; // maxv
					}
					wave[i] = (int) val;
				}
			scale =(int) 10000000 / (running_command.freq*WSz)-1; //TIM2->ARR = scale;
			break;
		case('A'):
			min = 4095;
			max = 0;

			for(int i = 0; i < WSz; i++)
			{
				if(ekg[i] > max)
				{
					max = ekg[i];
				}

				if(ekg[i] < min)
				{
					min = ekg[i];
				}

			}

			for(int i = 0; i < WSz; i++)
			{
				wave[i] = (ekg[i] - min) * ((running_command.maxv - running_command.minv) * 4095.0/3.3) / (max - min) + (running_command.minv * 4095.0/3.3);
			}
			scale = (int) 10000000 / (running_command.freq*256)-1;
		break;
		}
		// update DMA
		Update_DMA(&running_command, wave, wave_ch1, wave_ch2, 256, &scale);
		// if noise valid
		if(running_command.channel == 1)
			HAL_DACEx_NoiseWaveGenerate(&hdac1, DAC1_CHANNEL_1, running_command.noise << 8U);
		else
			HAL_DACEx_NoiseWaveGenerate(&hdac1, DAC1_CHANNEL_2, running_command.noise << 8U);

	}
	osDelay(1);
  }
  /* USER CODE END Start_commandTask */
}

/* USER CODE BEGIN Header_StartCaptureTask */
/**
* @brief Function implementing the captureTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCaptureTask */
void StartCaptureTask(void *argument)
{
  /* USER CODE BEGIN StartCaptureTask */
  /* Infinite loop */
	uint32_t scale = (int) 1000-1; //TIM2->ARR = scale;
	uint16_t min =9999;
	uint16_t max = 0;
	float min_f =0;
	float max_f=0;
	uint8_t tx_buffer[200];
  for(;;)
  {

	if(Capture_flag ==1){
		HAL_ADC_Start_DMA (&hadc1, Received_Waveform, 20000); // in regular mode, will stop after waveform is full
		osDelay(2100); // 2.1s
		HAL_ADC_Stop_DMA(&hadc1);
		Capture_flag = 0;

		HAL_TIM_Base_Start(&htim4);
		HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_2);

		__HAL_TIM_SET_AUTORELOAD(&htim4, scale);

		HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_2, Received_Waveform, 20000, DAC_ALIGN_12B_R);
		HAL_TIM_Base_Start(&htim2);

		max = Received_Waveform[0];
		min = Received_Waveform[0];
		for(int i=1;i<20000; i++){
			max = MAX(max, Received_Waveform[i]);
			min = MIN(min, Received_Waveform[i]);
		}
		max_f = max * 3.3/4095.0;
		min_f = min * 3.3/4095.0;
		memset(tx_buffer, 0, sizeof(tx_buffer[0]) * 200); // wipe Rx_command
		snprintf(tx_buffer, 200, "min: %f, max: %f\r\n", min_f, max_f);
		HAL_UART_Transmit(&huart2, tx_buffer, 200, 140);
	}
    osDelay(1);
  }
  /* USER CODE END StartCaptureTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * Structure with the information of each sensor
 * Sensor name: Commercial reference of current sensor as string.
 * Sensor type: Current sensor technology type as string.
 * Main gas: Current sensor main response gas as string.
 * Response time: Current sensor response time (t90) in seconds.
 */
typedef struct Sensor{
  char Sensor_name[11];
  char Sensor_type[14];
  char Main_gas[20];
  int16_t Response_time;
} Sensor;
/**
 * Structure with the information of PCB
 * PCB unique ID: Consecutive for the PCB ID.
 * Number of sensors: Number of sensor ADC readings
 * Manufacturing date: PCB manufacturing date as string on dd/mm/yyyy format.
 * PCB capabilities: Only the 4 LSBs are used to specify the environmental sensors
	that the PCB has, since not all readings are mandatory. The order is as follows:
	temperature_degC, temperaturePCB_degC, humidity_percent, absolutePressure_kPa (00001111).
 */
typedef struct PCB{
  int16_t PCBUniqueID;
  int8_t NumberOfSensors;
  char ManufacturingDate[10];
  int8_t PCBCapabilities;
} PCB;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//I2C functions
#define REQUEST_DEVICE_TYPE 1
#define REQUEST_DEVICE_METADATA_BASIC 2
#define REQUEST_DEVICE_METADATA_COMPLETE 3
#define REQUEST_DEVICE_VOLTAGE_DATA 4
//PCB configuration
#define temperature_degC 1
#define temperaturePCB_degC 1
#define humidity_percent 1
#define absolutePressure_kPa 1

#define temperature_degC_Response_time	10
#define temperaturePCB_degC_Response_time	10
#define humidity_percent_Response_time	10
#define absolutePressure_kPa_Response_time	10

#define PCBuniqueID 40
#define Numberofsensors 10
#define Manufacturingdate "dd/mm/yyyy"


//Sensor 01 parameters
#define Sensor01_name			"xxx1"
#define Sensor01_type			"fire"
#define Sensor01_Main_gas		"azufre"
#define Sensor01_Response_time	10
#define Sensor01_ADC_Channel	ADC_CHANNEL_6
//Sensor 02 parameters
#define Sensor02_name			"xxx1"
#define Sensor02_type			"fire"
#define Sensor02_Main_gas		"azufre"
#define Sensor02_Response_time	10
#define Sensor02_ADC_Channel	ADC_CHANNEL_6
//Sensor 03 parameters
#define Sensor03_name			"xxx1"
#define Sensor03_type			"fire"
#define Sensor03_Main_gas		"azufre"
#define Sensor03_Response_time	10
#define Sensor03_ADC_Channel	ADC_CHANNEL_6
//Sensor 04 parameters
#define Sensor04_name			"xxx1"
#define Sensor04_type			"fire"
#define Sensor04_Main_gas		"azufre"
#define Sensor04_Response_time	10
#define Sensor04_ADC_Channel	ADC_CHANNEL_6
//Sensor 05 parameters
#define Sensor05_name			"xxx1"
#define Sensor05_type			"fire"
#define Sensor05_Main_gas		"azufre"
#define Sensor05_Response_time	10
#define Sensor05_ADC_Channel	ADC_CHANNEL_6
//Sensor 06 parameters
#define Sensor06_name			"xxx1"
#define Sensor06_type			"fire"
#define Sensor06_Main_gas		"azufre"
#define Sensor06_Response_time	10
#define Sensor06_ADC_Channel	ADC_CHANNEL_6
//Sensor 07 parameters
#define Sensor07_name			"xxx1"
#define Sensor07_type			"fire"
#define Sensor07_Main_gas		"azufre"
#define Sensor07_Response_time	10
#define Sensor07_ADC_Channel	ADC_CHANNEL_6
//Sensor 08 parameters
#define Sensor08_name			"xxx1"
#define Sensor08_type			"fire"
#define Sensor08_Main_gas		"azufre"
#define Sensor08_Response_time	10
#define Sensor08_ADC_Channel	ADC_CHANNEL_6
//Sensor 09 parameters
#define Sensor09_name			"xxx1"
#define Sensor09_type			"fire"
#define Sensor09_Main_gas		"azufre"
#define Sensor09_Response_time	10
#define Sensor09_ADC_Channel	ADC_CHANNEL_6
//Sensor 10 parameters
#define Sensor10_name			"xxx1"
#define Sensor10_type			"fire"
#define Sensor10_Main_gas		"azufre"
#define Sensor10_Response_time	10
#define Sensor10_ADC_Channel	ADC_CHANNEL_6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

PCD_HandleTypeDef hpcd_USB_FS;

osThreadId Task01_I2CHandle;
osMutexId MutexADC1Handle;
osSemaphoreId SemI2CHandle;
/* USER CODE BEGIN PV */

PCB pcb;
ADC_ChannelConfTypeDef sConfig2 = {0};
float DataSensor[Numberofsensors+4];
uint32_t SensorChannel[Numberofsensors];
//Definition of atmospheric data sensors
Sensor ExternalTemperatureSensor;
Sensor InternalTemperatureSensor;
Sensor HumiditySensor;
Sensor PressureSensor;

Sensor MatrizSensor[Numberofsensors];

//Definition of tasks for atmospheric data sensors
osThreadId Task02Handle;
osThreadId Task03Handle;
osThreadId Task04Handle;
osThreadId Task05Handle;
//Definition of N task for sensors
osThreadId TaskN01Handle;
osThreadId TaskN02Handle;
osThreadId TaskN03Handle;
osThreadId TaskN04Handle;
osThreadId TaskN05Handle;
osThreadId TaskN06Handle;
osThreadId TaskN07Handle;
osThreadId TaskN08Handle;
osThreadId TaskN09Handle;
osThreadId TaskN10Handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USB_PCD_Init(void);
void StartTask01_I2C(void const * argument);

/* USER CODE BEGIN PFP */
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);
void StartTaskN(void const * argument);
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  //preconfigure the ADC to change channels
	sConfig2.Rank = ADC_REGULAR_RANK_1;
	sConfig2.SingleDiff = ADC_SINGLE_ENDED;
	sConfig2.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
	sConfig2.OffsetNumber = ADC_OFFSET_NONE;
	sConfig2.Offset = 0;
  // PCB data initialization
  pcb.PCBUniqueID=PCBuniqueID;
  pcb.NumberOfSensors=Numberofsensors;
  strcpy(pcb.ManufacturingDate,Manufacturingdate);
  pcb.PCBCapabilities=temperature_degC*8 + temperaturePCB_degC*4 + humidity_percent*2 + absolutePressure_kPa;


  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of MutexADC1 */
  osMutexDef(MutexADC1);
  MutexADC1Handle = osMutexCreate(osMutex(MutexADC1));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of SemI2C */
  osSemaphoreDef(SemI2C);
  SemI2CHandle = osSemaphoreCreate(osSemaphore(SemI2C), 1);

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
  /* definition and creation of Task01_I2C */
  osThreadDef(Task01_I2C, StartTask01_I2C, osPriorityRealtime, 0, 128);
  Task01_I2CHandle = osThreadCreate(osThread(Task01_I2C), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* definition and creation of Task02 */
#if(temperature_degC==1)
	osThreadDef(Task02, StartTask02, osPriorityNormal, 0, 128);
	Task02Handle = osThreadCreate(osThread(Task02), NULL);
#endif
  /* definition and creation of Task03 */
#if(temperaturePCB_degC==1)
	osThreadDef(Task03, StartTask03, osPriorityNormal, 0, 128);
	Task03Handle = osThreadCreate(osThread(Task03), NULL);
#endif
  /* definition and creation of Task04 */
#if(humidity_percent==1)
	osThreadDef(Task04, StartTask04, osPriorityNormal, 0, 128);
	Task04Handle = osThreadCreate(osThread(Task04), NULL);
#endif
  /* definition and creation of Task05 */
#if(absolutePressure_kPa==1)
	osThreadDef(Task05, StartTask05, osPriorityNormal, 0, 128);
	Task05Handle = osThreadCreate(osThread(Task05), NULL);
#endif


  /* definition and creation of TaskN01 */
  //Sensor initialization
  	strcpy(MatrizSensor[0].Sensor_name,Sensor01_name);
  	strcpy(MatrizSensor[0].Sensor_type,Sensor01_type);
  	strcpy(MatrizSensor[0].Main_gas,Sensor01_Main_gas);
  	MatrizSensor[0].Response_time=Sensor01_Response_time;
  	SensorChannel[0]=Sensor01_ADC_Channel;
	osThreadDef(TaskN01, StartTaskN, osPriorityNormal, 0, 128);
	TaskN01Handle = osThreadCreate(osThread(TaskN01), (void*) 0);
  /* definition and creation of TaskN02 */
#if(Numberofsensors>1)
	//Sensor initialization
	strcpy(MatrizSensor[1].Sensor_name,Sensor02_name);
	strcpy(MatrizSensor[1].Sensor_type,Sensor02_type);
	strcpy(MatrizSensor[1].Main_gas,Sensor02_Main_gas);
	MatrizSensor[1].Response_time=Sensor02_Response_time;
	SensorChannel[1]=Sensor02_ADC_Channel;
	osThreadDef(TaskN02, StartTaskN, osPriorityNormal, 0, 128);
	TaskN02Handle = osThreadCreate(osThread(TaskN02), (void*) 1);
#endif
  /* definition and creation of TaskN03 */
#if(Numberofsensors>2)
  	//Sensor initialization
  	strcpy(MatrizSensor[2].Sensor_name,Sensor03_name);
  	strcpy(MatrizSensor[2].Sensor_type,Sensor03_type);
  	strcpy(MatrizSensor[2].Main_gas,Sensor03_Main_gas);
  	MatrizSensor[2].Response_time=Sensor03_Response_time;
  	SensorChannel[2]=Sensor03_ADC_Channel;
  	osThreadDef(TaskN03, StartTaskN, osPriorityNormal, 0, 128);
  	TaskN03Handle = osThreadCreate(osThread(TaskN03), (void*) 2);
#endif
  /* definition and creation of TaskN04 */
#if(Numberofsensors>3)
  	//Sensor initialization
  	strcpy(MatrizSensor[3].Sensor_name,Sensor04_name);
  	strcpy(MatrizSensor[3].Sensor_type,Sensor04_type);
  	strcpy(MatrizSensor[3].Main_gas,Sensor04_Main_gas);
  	MatrizSensor[3].Response_time=Sensor04_Response_time;
  	SensorChannel[3]=Sensor04_ADC_Channel;
  	osThreadDef(TaskN04, StartTaskN, osPriorityNormal, 0, 128);
  	TaskN04Handle = osThreadCreate(osThread(TaskN04), (void*) 3);
#endif
  /* definition and creation of TaskN05 */
#if(Numberofsensors>4)
	//Sensor initialization
	strcpy(MatrizSensor[4].Sensor_name,Sensor05_name);
	strcpy(MatrizSensor[4].Sensor_type,Sensor05_type);
	strcpy(MatrizSensor[4].Main_gas,Sensor05_Main_gas);
	MatrizSensor[4].Response_time=Sensor05_Response_time;
	SensorChannel[4]=Sensor05_ADC_Channel;
	osThreadDef(TaskN05, StartTaskN, osPriorityNormal, 0, 128);
	TaskN05Handle = osThreadCreate(osThread(TaskN05), (void*) 4);
#endif
  /* definition and creation of TaskN06 */
#if(Numberofsensors>5)
	//Sensor initialization
	strcpy(MatrizSensor[5].Sensor_name,Sensor06_name);
	strcpy(MatrizSensor[5].Sensor_type,Sensor06_type);
	strcpy(MatrizSensor[5].Main_gas,Sensor06_Main_gas);
	MatrizSensor[5].Response_time=Sensor06_Response_time;
	SensorChannel[5]=Sensor06_ADC_Channel;
	osThreadDef(TaskN06, StartTaskN, osPriorityNormal, 0, 128);
	TaskN06Handle = osThreadCreate(osThread(TaskN06), (void*) 5);
#endif
  /* definition and creation of TaskN07 */
#if(Numberofsensors>6)
	//Sensor initialization
	strcpy(MatrizSensor[6].Sensor_name,Sensor07_name);
	strcpy(MatrizSensor[6].Sensor_type,Sensor07_type);
	strcpy(MatrizSensor[6].Main_gas,Sensor07_Main_gas);
	MatrizSensor[6].Response_time=Sensor07_Response_time;
	SensorChannel[6]=Sensor07_ADC_Channel;
	osThreadDef(TaskN07, StartTaskN, osPriorityNormal, 0, 128);
	TaskN07Handle = osThreadCreate(osThread(TaskN07), (void*) 6);
#endif
  /* definition and creation of TaskN08 */
#if(Numberofsensors>7)
  //Sensor initialization
	strcpy(MatrizSensor[7].Sensor_name,Sensor08_name);
	strcpy(MatrizSensor[7].Sensor_type,Sensor08_type);
	strcpy(MatrizSensor[7].Main_gas,Sensor08_Main_gas);
	MatrizSensor[7].Response_time=Sensor08_Response_time;
	SensorChannel[7]=Sensor08_ADC_Channel;
	osThreadDef(TaskN08, StartTaskN, osPriorityNormal, 0, 128);
	TaskN08Handle = osThreadCreate(osThread(TaskN08), (void*) 7);
#endif
  /* definition and creation of TaskN09 */
#if(Numberofsensors>8)
	//Sensor initialization
	strcpy(MatrizSensor[8].Sensor_name,Sensor09_name);
	strcpy(MatrizSensor[8].Sensor_type,Sensor09_type);
	strcpy(MatrizSensor[8].Main_gas,Sensor09_Main_gas);
	MatrizSensor[8].Response_time=Sensor09_Response_time;
	SensorChannel[8]=Sensor09_ADC_Channel;
	osThreadDef(TaskN09, StartTaskN, osPriorityNormal, 0, 128);
	TaskN09Handle = osThreadCreate(osThread(TaskN09), (void*) 8);
#endif
  /* definition and creation of TaskN10 */
#if(Numberofsensors>9)
	//Sensor initialization
	strcpy(MatrizSensor[9].Sensor_name,Sensor10_name);
	strcpy(MatrizSensor[9].Sensor_type,Sensor10_type);
	strcpy(MatrizSensor[9].Main_gas,Sensor10_Main_gas);
	MatrizSensor[9].Response_time=Sensor10_Response_time;
	SensorChannel[9]=Sensor10_ADC_Channel;
	osThreadDef(TaskN10, StartTaskN, osPriorityNormal, 0, 128);
	TaskN10Handle = osThreadCreate(osThread(TaskN10), (void*) 9);
#endif

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 2;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MISOA7_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MISOA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

#if(temperature_degC==1)
/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread environment temperature reading.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  //Sensor initialization
	strcpy(ExternalTemperatureSensor.Sensor_name,"SHT31-ARP-B");
	strcpy(ExternalTemperatureSensor.Sensor_type,"Temperature");
	strcpy(ExternalTemperatureSensor.Main_gas,"....");
	ExternalTemperatureSensor.Response_time=temperature_degC_Response_time;
  /* Infinite loop */
  for(;;)
  {
	/*use of the ADC with mutex, this so that only one task can use the ADC at a time*/
	osMutexWait(MutexADC1Handle, 100);
	sConfig2.Channel=ADC_CHANNEL_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig2) != HAL_OK){Error_Handler();}
	// Start ADC Conversion
	HAL_ADC_Start(&hadc1);
	// Poll ADC1 Perihperal & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc1, 1);
	// Read The ADC Conversion Result
	uint32_t Vadc=HAL_ADC_GetValue(&hadc1);
	// stop The ADC
	HAL_ADC_Stop(&hadc1);
	osMutexRelease(MutexADC1Handle);
	//The temperature formula is T=-66.875 + 218.75*Vt/Vd
	//where Vd=3.3, Vt=adc*3.3/2^12
	//The temperature formula is T=-66.875 + 218.75*Vadc/4096
	DataSensor[0]=-66.875+(53.40576172e-3*Vadc);
    osDelay(ExternalTemperatureSensor.Response_time);
  }
  /* USER CODE END StartTask02 */
}
#endif
#if(temperaturePCB_degC==1)
/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task03 thread internal temperature reading.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  //Sensor initialization
	strcpy(InternalTemperatureSensor.Sensor_name,"Internal");
	strcpy(InternalTemperatureSensor.Sensor_type,"Micro-controller temperature ");
	strcpy(InternalTemperatureSensor.Main_gas,"....");
	InternalTemperatureSensor.Response_time=temperaturePCB_degC_Response_time;
  /* Infinite loop */
  for(;;)
  {
    /*use of the ADC with mutex, this so that only one task can use the ADC at a time*/
	osMutexWait(MutexADC1Handle, 100);
	sConfig2.Channel=ADC_CHANNEL_TEMPSENSOR;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig2) != HAL_OK){Error_Handler();}
	// Start ADC Conversion
	HAL_ADC_Start(&hadc1);
	// Poll ADC1 Perihperal & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc1, 1);
	// Read The ADC Conversion Result
	uint32_t Vadc=HAL_ADC_GetValue(&hadc1);
	// stop The ADC
	HAL_ADC_Stop(&hadc1);
	osMutexRelease(MutexADC1Handle);
	//The formula is Temperature (in °C) = {(V25 – Vadc) / Avg_Slope} + 25
	//where V25=1.43, Avg_Slope=4.3, Vadc=adc*3.3/4096
	DataSensor[1]=((1.43 - (805.6640625e-6 * Vadc)) / 4.3) + 25;
    osDelay(InternalTemperatureSensor.Response_time);
  }
  /* USER CODE END StartTask03 */
}
#endif
#if (humidity_percent==1)
/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Task04 thread Humidity sensor reading.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  //Sensor initialization
	strcpy(HumiditySensor.Sensor_name,"SHT31-ARP-B");
	strcpy(HumiditySensor.Sensor_type,"Humidity:");
	strcpy(HumiditySensor.Main_gas,"....");
	HumiditySensor.Response_time=humidity_percent_Response_time;
  /* Infinite loop */
  for(;;)
  {
    /*use of the ADC with mutex, this so that only one task can use the ADC at a time*/
	osMutexWait(MutexADC1Handle, 100);
	sConfig2.Channel=ADC_CHANNEL_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig2) != HAL_OK){Error_Handler();}
	// Start ADC Conversion
	HAL_ADC_Start(&hadc1);
	// Poll ADC1 Perihperal & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc1, 1);
	// Read The ADC Conversion Result
	uint32_t Vadc=HAL_ADC_GetValue(&hadc1);
	// stop The ADC
	HAL_ADC_Stop(&hadc1);
	osMutexRelease(MutexADC1Handle);
	//The temperature formula is Rh=-12.5 + 125*Vt/Vd
	//where Vd=3.3, Vt=adc*3.3/2^12
	//The temperature formula is Rh=-12.5 + 125*Vadc/4096
    DataSensor[2]=-12.5 + (30.51757813e-3*Vadc);
    osDelay(HumiditySensor.Response_time);
  }
  /* USER CODE END StartTask04 */
}
#endif
#if(absolutePressure_kPa==1)
/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the Task05 thread Absolute pressure sensor reading.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  //Sensor initialization
	strcpy(PressureSensor.Sensor_name,"KP229-E2701-XTMA1");
	strcpy(PressureSensor.Sensor_type,"Absolute pressure");
	strcpy(PressureSensor.Main_gas,"....");
	PressureSensor.Response_time=humidity_percent_Response_time;
  /* Infinite loop */
  for(;;)
  {
    /*use of the ADC with mutex, this so that only one task can use the ADC at a time*/
	osMutexWait(MutexADC1Handle, 100);
	sConfig2.Channel=ADC_CHANNEL_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig2) != HAL_OK){Error_Handler();}
	// Start ADC Conversion
	HAL_ADC_Start(&hadc1);
	// Poll ADC1 Perihperal & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc1, 1);
	// Read The ADC Conversion Result
	uint32_t Vadc=HAL_ADC_GetValue(&hadc1);
	// stop The ADC
	HAL_ADC_Stop(&hadc1);
	osMutexRelease(MutexADC1Handle);
	//The temperature formula is P=(Vp/Vdd-b)/a
	//where Vp=adc*3.3/2^12, Vdd=3.3, b=0.05069, a=0.00293.
	//The temperature formula is P=-b/a+adc/a/4096=-17.3003413+83.32444539e-3*adc
	DataSensor[3]=-17.3003413 + (83.32444539e-3*Vadc);
    osDelay(PressureSensor.Response_time);
  }
  /* USER CODE END StartTask05 */
}
#endif
/* USER CODE BEGIN Header_StartTaskN */
/**
* @brief Function implementing the TaskN thread of gas sensor reading.
* @param argument: used Sensor_number
*
* @retval None
*/
/* USER CODE END Header_StartTaskN */
void StartTaskN(void const * argument)
{
  /* USER CODE BEGIN StartTaskN */

  /* Infinite loop */
  for(;;)
  {
    /*use of the ADC with mutex, this so that only one task can use the ADC at a time*/
	osMutexWait(MutexADC1Handle, 100);
	sConfig2.Channel=SensorChannel[(int)argument];
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig2) != HAL_OK){Error_Handler();}
	// Start ADC Conversion
	HAL_ADC_Start(&hadc1);
	// Poll ADC1 Perihperal & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc1, 1);
	// Read The ADC Conversion Result
	uint32_t Vadc=HAL_ADC_GetValue(&hadc1);
	// stop The ADC
	HAL_ADC_Stop(&hadc1);
	osMutexRelease(MutexADC1Handle);
	//The voltage value in miliVolts is Vadc=adc*3300/4096
	DataSensor[((int)argument)+3]=805.6640625e-3*Vadc;
    osDelay(MatrizSensor[(int)argument].Response_time*1000);
  }
  /* USER CODE END StartTaskN*/
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01_I2C */
/**
  * @brief  Function implementing the Task01_I2C thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01_I2C */
void StartTask01_I2C(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t pRecognized=0;
	HAL_I2C_Slave_Transmit_IT(&hi2c1,(uint8_t*)&pRecognized, 1);
	osSemaphoreWait(SemI2CHandle, 0xFFFFFFFF);
  /* Infinite loop */
  for(;;)
  {
	uint8_t pFuncion=0;
	HAL_I2C_Slave_Seq_Receive_IT(&hi2c1,(uint8_t*)&pFuncion, 1,I2C_FIRST_FRAME);
    osSemaphoreWait(SemI2CHandle, 0xFFFFFFFF);
    if(pFuncion==REQUEST_DEVICE_TYPE){
    	HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1,(uint8_t*)&pRecognized, 1,I2C_LAST_FRAME);
    	osSemaphoreWait(SemI2CHandle, 0xFFFFFFFF);
    }
    else if(pFuncion==REQUEST_DEVICE_METADATA_BASIC){
    	HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1,(uint8_t*)&pcb, 14,I2C_LAST_FRAME);
		osSemaphoreWait(SemI2CHandle, 0xFFFFFFFF);
    }
    else if(pFuncion==REQUEST_DEVICE_METADATA_COMPLETE){
    	HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1,(uint8_t*)MatrizSensor, 47*Numberofsensors,I2C_LAST_FRAME);
		osSemaphoreWait(SemI2CHandle, 0xFFFFFFFF);
    }
    else if(pFuncion==REQUEST_DEVICE_VOLTAGE_DATA){
    	HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1,(uint8_t*)DataSensor, 14+(4*Numberofsensors),I2C_LAST_FRAME);
		osSemaphoreWait(SemI2CHandle, 0xFFFFFFFF);
    }
  }
  /* USER CODE END 5 */
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


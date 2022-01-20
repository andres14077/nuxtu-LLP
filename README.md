# STM32 STM32F303VCT6 Nuxtu gas sensor arrays

This repository presents a solution for the NUXTU company, this will have the following stages of development.
1. project creation and configuration according      	
   to development board					100%
2. FreeRTOS implementation.				100%
3. Configure the operating system to have 4 
   tasks for reading the ambient sensors.		100%
4. Configure the operating system to have N 
   tasks for reading the gas sensors.			100%
5. ADC peripheral configuration				100%
6. I2C peripheral configuration				40%
7. Create I2C slave functions				00%

The proposed solution is based on a freeRTOS.

They consist of 5 main tasks and N tasks that depend on the day of the test, these tasks are:
1. A high priority task which will be called when an interrupt is executed by the I2C peripheral, this will be in charge of processing and sending the message by the I2C.
2. 4 tasks that measure environmental variables from time to time, these tasks are created only if the sensors are available.
3. N tasks to measure the gas sensors from time to time, this task is the same in all cases, the input parameter varies when it is created.


The first thing is the creation of the project in STM32CubeIDE where the STM32F3DISCOVERY is configured.
FreeRTOS CMSIS v1 is implemented in the Middleware section, no changes are made to its configurations.
In the System core in the SYS section, change the Timebase Source from SysTick to TIM6
The task for handling I2C is created with a priority in real time in the configuration page.
A binary semaphore for I2C use called SemI2C is created.
A Mutex is created for ADC1 called MutexADC1
Before creating the initial code, the ADC peripheral is configured, in which the different channels, from 1 to 9, and the internal temperature sensor are activated. Sampling time is taken as 61.5 cyclos, which gives us a time of 1.281 microseconds.
The I2c configuration has been configured in normal mode.

Subsequently, the initial code is created with the configurations described.
For the use of the sensors, an object called `Sensor` is created that has the basic information of each sensor.
```
typedef struct Sensor{
  char Sensor_name[11];
  char Sensor_type[14];
  char Main_gas[20];
  int16_t Response_time;
} Sensor;
```
Another object that is created is `PCB`, this contains the information of the board
```
typedef struct PCB{
  int16_t PCBUniqueID;
  int8_t NumberOfSensors;
  char ManufacturingDate[10];
  int8_t PCBCapabilities;
} PCB;
```
The PCB data is defined at the beginning for an easy change of this information.
```
//PCB configuration
#define temperature_degC 1
#define temperaturePCB_degC 1
#define humidity_percent 1
#define absolutePressure_kPa 1

#define PCBuniqueID 40
#define Numberofsensors 2
#define Manufacturingdate "dd/mm/yyyy"
```

For each sensor the following parameters are defined:
```
#define Sensor01_name			""
#define Sensor01_type			""
#define Sensor01_Main_gas		""
#define Sensor01_Response_time		xx
#define Sensor01_ADC_number		xx
#define Sensor01_ADC_Channel	ADC_CHANNEL_x
```
All sensors are created as global variables so that they are accessible by all tasks. 
```
//Definition of atmospheric data sensors
Sensor ExternalTemperatureSensor;
Sensor InternalTemperatureSensor;
Sensor HumiditySensor;
Sensor PressureSensor;

Sensor MatrizSensor[Numberofsensors];
```
The handlers for the tasks are also created as global variables, it is limited that N can reach up to 10.
```
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
```

In the definition of functions, the following are defined manually:
```
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);
void StartTaskN(void const * argument);
```
Where the first 4 are responsible for calculating the environmental variables, and the last one is a generic function for the N gas sensors.

At the beginning of main, the peripheral configuration is performed, the object is initialized with the characteristics of the PCB board, and a variable called `sConfig2` is initialized, this will be used later to be able to change the ADC channel in each task.

```
ADC_ChannelConfTypeDef sConfig2 = {0};
...
//preconfigure the ADC to change channels
sConfig2.Rank = ADC_REGULAR_RANK_1;
sConfig2.SingleDiff = ADC_SINGLE_ENDED;
sConfig2.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
sConfig2.OffsetNumber = ADC_OFFSET_NONE;
sConfig2.Offset = 0;
```
For the creation of the necessary tasks for reading the environmental variables, the `StartTask0X` function is created, it initializes the sensor data indicating the name, type, and response time, to later enter an infinite loop. In this, the use of the ADC will be requested through a Mutex so that no other task can use it.
first the ADC channel is changed, then the ADC is started, a Poll is requested and the measured value is obtained, finally we stop the ADC and it is released for use in other tasks.
```
#if(temperaturePCB_degC==1)
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
```
Each internal sensor has its own calibration formula which we can consult in the following links.

1. [Internal temperature sensor ecuation.](https://www.st.com/resource/en/reference_manual/dm00043574-stm32f303xb-c-d-e-stm32f303x6-8-stm32f328x8-stm32f358xc-stm32f398xe-advanced-arm-based-mcus-stmicroelectronics.pdf#page=373&zoom=100,165,0)

2. [Values of electrical constants for internal temperature sensor.](https://www.st.com/resource/en/datasheet/stm32f303vc.pdf#page=124)

3. [Humidity and temperature sensor SHT31-ARP-B.](https://www.mouser.com/pdfdocs/SHT3x-ARP_Datasheet.pdf#page=8)

4. [Sensor KP229-E2701-XTMA1.](https://www.infineon.com/dgdl/Infineon-KP229E2701-DS-v01_00-en.pdf?fileId=db3a30432ad629a6012af6bce0290b5d#page=12)

To dynamically create the task of reading these sensors depending on the configuration of the board, the tasks are initialized in a conditional.

```
/* definition and creation of Task0X */
#if(temperaturePCB_degC==1)
	osThreadDef(Task0x, StartTask0x, osPriorityNormal, 0, 128);
	Task0xHandle = osThreadCreate(osThread(Task0x), NULL);
#endif
```

For the array of N gas sensors, the `StartTaskN` function is created, this, unlike the previous ones, receives a parameter when it is created, which is the number of the sensor in the array, and it does not initialize the sensor parameters, these are they are going to initialize before starting the task, another difference is that the read value is only transformed to millivolts and not to the representative quantity of the gas.
```
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
	DataSensor[(int)argument]=805.6640625e-3*Vadc;
	osDelay(MatrizSensor[(int)argument].Response_time*1000);
  }
  /* USER CODE END StartTaskN*/
}
```
To dynamically create the tasks of these sensors, they are created with a conditional dependent on the number of sensors, taking into account that there will always be at least one sensor.
```
/* definition and creation of TaskN0X */
  if(Numberofsensors>X-1){
	//Sensor initialization
	strcpy(MatrizSensor[X-1].Sensor_name,Sensor0X_name);
	strcpy(MatrizSensor[X-1].Sensor_type,Sensor0X_type);
	strcpy(MatrizSensor[X-1].Main_gas,Sensor0X_Main_gas);
	MatrizSensor[X-1].Response_time=Sensor0X_Response_time;
	SensorChannel[X-1]=Sensor0X_ADC_Channel;
	osThreadDef(TaskN0X, StartTaskN, osPriorityNormal, 0, 128);
	TaskN0XHandle = osThreadCreate(osThread(TaskN0X), (void*) X-1);
  }
```
The implementation of the I2C protocol was done with interruptions and the RTOS, for this a semaphore is used which allows the I2C task to know when a message from the master has been received or when a request from the master has been answered.

In the call of the interruption, a condition is placed to know if the peripheral is ready for a new operation, which indicates that it has finished receiving or transmitting the information. Inside, the semaphore is released, so that the highest priority task can continue its operation.

```
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */
	if(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY){
		osSemaphoreRelease(SemI2CHandle);
	}
  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}
```
The I2C task starts waiting for an acknowledgment from the master with a READ_BYTE, to which it responds with a 0.
later it enters the infinite loop, in this it waits for the master to send it the number of the process that it will receive as a `uint8`, depending on the instruction that the task must execute, it will send the pointer to the object where the information is is requested.
```
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
	HAL_I2C_Slave_Receive_IT(&hi2c1,(uint8_t*)&pFuncion, 1);
    osSemaphoreWait(SemI2CHandle, 0xFFFFFFFF);
    if(pFuncion==1){
    	HAL_I2C_Slave_Transmit_IT(&hi2c1,(uint8_t*)&pRecognized, 1);
    	osSemaphoreWait(SemI2CHandle, 0xFFFFFFFF);
    }
    else if(pFuncion==2){
    	HAL_I2C_Slave_Transmit_IT(&hi2c1,(uint8_t*)&pcb, 14);
		osSemaphoreWait(SemI2CHandle, 0xFFFFFFFF);
    }
    else if(pFuncion==3){
    	HAL_I2C_Slave_Transmit_IT(&hi2c1,(uint8_t*)&MatrizSensor, 47*Numberofsensors);
		osSemaphoreWait(SemI2CHandle, 0xFFFFFFFF);
    }
    else if(pFuncion==4){
    	HAL_I2C_Slave_Transmit_IT(&hi2c1,(uint8_t*)&DataSensor, 14+(4*Numberofsensors));
		osSemaphoreWait(SemI2CHandle, 0xFFFFFFFF);
    }
  }
  /* USER CODE END 5 */
}
```
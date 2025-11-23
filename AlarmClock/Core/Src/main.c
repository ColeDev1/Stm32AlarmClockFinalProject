/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "arm_math.h"

#include "stm32l4s5i_iot01.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_qspi.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ringtone_data.h"
#include "audio_player.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UA_BUF_SIZE 64
#define config 0
#define waitingForAlarm 1
#define snooze 2
#define display 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

LPTIM_HandleTypeDef hlptim1;

OSPI_HandleTypeDef hospi1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t audio_ready = 0;
static uint32_t last_button_time = 0;
int state = config; // 0 is configuration, 1 is wait for alarm, 2 is snooze, 3 is display
char *city = "Montreal";

uint8_t uart_byte;

uint8_t char_buffer[UA_BUF_SIZE];
uint8_t city_buffer[UA_BUF_SIZE];    // store city string
uint8_t timer_buffer[UA_BUF_SIZE];        // store timer string
uint8_t duration_buffer[UA_BUF_SIZE];
volatile uint16_t rx_index = 0;          // current write index into char_buffer
volatile uint8_t line_ready = 0; // set to 1 when user presses ENTER (submit data)

volatile uint8_t clock_state = 0; // 0: location config, 1: timer config, 2: duration config 3: clock mode

volatile uint8_t prompt_printed = 0; // flag to make each prompt prints only once
volatile uint8_t clock_triggered = 1; // flag set to 1 when alarm is triggered
int duration_minutes = 0;
int snoozeCounter = 0;
int alarmDurationCounter = 0;
int currentMin;
int currentHour;
int alarmMin;
int alarmHour;
int valid_input;
uint8_t alreadyDisplaying = 0;
uint8_t stateChanged;
RTC_TimeTypeDef inputTime = { 0 };
RTC_TimeTypeDef currentTime = { 0 };
RTC_AlarmTypeDef inputAlarm = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//button isr

	//waiting for alarm (1) -> configure (0)
	//snooze (2) -> configure (0)
	if (state == waitingForAlarm || state == snooze) {
		state = config;
		stateChanged = 1;
	}
	//if in display mode go to snooze
	if (state == display) {
		state = snooze;
		stateChanged = 1;
		Audio_Stop();
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim6) {
	    Audio_TimerCallback();

	} else if (htim == &htim7) {
		//counts while alarm is playing
		if (state == display){
			alarmDurationCounter++;
			if (alarmDurationCounter >= 120) {
				state = waitingForAlarm;
				Audio_Stop();
				stateChanged = 1;
			}
		} else {	//count snooze duration
			snoozeCounter++;

			//condition to stop playing alarm
			if (snoozeCounter >= 120){
				state = display;
				stateChanged = 1;
			}
		}
	} else if (htim == &htim16) {
		//end of snooze
		state = display;
		stateChanged = 1;
	}
}
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	//wake up from low power
	//set state to display
	state = display;
	stateChanged = 1;
}

void SetAlarm(uint8_t hour, uint8_t minute, uint8_t second) {
	RTC_AlarmTypeDef sAlarm = { 0 };

	sAlarm.AlarmTime.Hours = hour;
	sAlarm.AlarmTime.Minutes = minute;
	sAlarm.AlarmTime.Seconds = second;
	sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	sAlarm.Alarm = RTC_ALARM_A;

	HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);
}
int parseTime(char* str, int * hour, int * minute){
	//str should be in the form 12:15
	//return -1 if it fails
	//returns 1 if it success

	char * middle;
	//parse hours
	long interm = strtol(str, &middle, 10);
	if (*middle != ':' || middle == str) return -1;

	if (interm < 0 || interm > 23) return -1;
	*hour = (int) interm;

	//parsing minutes
	interm = strtol(middle + 1, &middle,10);
	if (*middle != '\0') return -1;
	if (interm < 0 || interm > 59) return -1;
	* minute = (int) interm;

	return 1;

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
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_OCTOSPI1_Init();
  MX_LPTIM1_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  Audio_Init();

  BSP_HSENSOR_Init();
  BSP_TSENSOR_Init();
  Audio_Store(ringtone_raw, ringtone_raw_len);
  audio_ready = 1;

	//timer 6 for DAC
	//timer 7 for inbetween alarm time
	//lowpower timer for alarm.

	//prompt user for info

	// Start UART reception in interrupt mode (1 byte at a time)
	HAL_UART_Receive_IT(&huart1, &uart_byte, 1);

	char uartMsg[256];
	const char *cityMsg = "[Enter Location]: ";
	const char *timerMsg = "\r\n[Enter Timer (e.g. 14:30)]: ";
	const char *durationMsg = "\r\n[Enter Timer Duration (in min)]: ";

	// Intro banner
	const char *banner = "\r\n=== ALARM CLOCK CONFIG ===\r\n"
			"You will be asked for:\r\n"
			"1) Location\r\n"
			"2) Timer\r\n"
			"3) Timer Duration\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*) banner, strlen(banner),
			HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		//state machine below
		if (state == config) { //this state will be where configuration occurs and then where the alarm is set
			while (state == config) {
				if (line_ready) {
					// handle buffer info when user presses enter
					line_ready = 0;
					if (clock_state == 0) {
						strncpy((char*) city_buffer, (char*) char_buffer,
								UA_BUF_SIZE - 1);
						city_buffer[UA_BUF_SIZE - 1] = '\0';
						city = (char *) city_buffer;
						clock_state++;
					} else if (clock_state == 1) {
						strncpy((char*) timer_buffer, (char*) char_buffer,
								UA_BUF_SIZE - 1);
						timer_buffer[UA_BUF_SIZE - 1] = '\0';

						valid_input = parseTime((char*)timer_buffer,&currentHour,&currentMin);
						if (valid_input == -1){
							char * invalidInput = "\r\nTime format is Hour:Minute in 24hr time, Try Again";
							HAL_UART_Transmit(&huart1, (uint8_t*) invalidInput, strlen(invalidInput), HAL_MAX_DELAY);
						}else clock_state ++;
					} else if (clock_state == 2) {
						strncpy((char*) duration_buffer, (char*) char_buffer,
								UA_BUF_SIZE - 1);
						duration_buffer[UA_BUF_SIZE - 1] = '\0';

						valid_input = parseTime((char*)duration_buffer,&alarmHour,&alarmMin);
						if (valid_input == -1){
							char * invalidInput = "Time format is Hour:Minute\r\nTry Again";
							HAL_UART_Transmit(&huart1, (uint8_t*) invalidInput, strlen(invalidInput), HAL_MAX_DELAY);

						} else clock_state++;

					}
//					clock_state++;
					prompt_printed = 0;
				}

				if (clock_state == 0 && !prompt_printed) {
					HAL_UART_Transmit(&huart1, (uint8_t*) cityMsg, strlen(cityMsg),
							HAL_MAX_DELAY);
					prompt_printed = 1;
				} else if (clock_state == 1 && !prompt_printed) {
					HAL_UART_Transmit(&huart1, (uint8_t*) timerMsg,
							strlen(timerMsg), HAL_MAX_DELAY);
					prompt_printed = 1;
				} else if (clock_state == 2 && !prompt_printed) {
					HAL_UART_Transmit(&huart1, (uint8_t*) durationMsg,
							strlen(durationMsg), HAL_MAX_DELAY);
					prompt_printed = 1;
				} else if (clock_state == 3 && !prompt_printed && clock_triggered) //only print prompt once, in the future we will add another flag to this condition, "clock_triggered", when alarm goes off
						{

					int tempSensor = (int) BSP_TSENSOR_ReadTemp();
					int humiditySensor = (int) BSP_HSENSOR_ReadHumidity();

					snprintf(uartMsg, sizeof(uartMsg),
							"\r\n[ALARM_CLOCK] Temp: %d°C | Humidity: %d%%\r\n"
									"[CONFIG] Location: %s | Time: %s | Alarm Time: %s\r\n",
							tempSensor, humiditySensor, (char*) city_buffer,
							(char*) timer_buffer, duration_buffer);

					HAL_UART_Transmit(&huart1, (uint8_t*) uartMsg, strlen(uartMsg),
							HAL_MAX_DELAY);
					prompt_printed = 1;
					state = waitingForAlarm;
					stateChanged = 1;
					//HAL_Delay(1000);
				} // end of config while loop
			}
			//leaving config
			//Set the RTC Time + alarm
			inputTime.Hours = currentHour;
			inputTime.Minutes = currentMin;
			inputTime.Seconds = 0x0;
			inputTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			inputTime.StoreOperation = RTC_STOREOPERATION_RESET;
			  if (HAL_RTC_SetTime(&hrtc, &inputTime, RTC_FORMAT_BIN) != HAL_OK)
			  {
			    Error_Handler();
			  }

			  /** Enable the Alarm A
			  */
			  inputAlarm.AlarmTime.Hours = alarmHour;
			  inputAlarm.AlarmTime.Minutes = alarmMin;
			  inputAlarm.AlarmTime.Seconds = 0x0;
			  inputAlarm.AlarmTime.SubSeconds = 0x0;
			  inputAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			  inputAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
			  inputAlarm.AlarmMask = RTC_ALARMMASK_NONE;
			  inputAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
			  inputAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
			  inputAlarm.AlarmDateWeekDay = 0x1;
			  inputAlarm.Alarm = RTC_ALARM_A;
			  HAL_RTC_DeactivateAlarm(&hrtc,RTC_ALARM_A);
			  if (HAL_RTC_SetAlarm_IT(&hrtc, &inputAlarm, RTC_FORMAT_BIN) != HAL_OK)
			  {
			    Error_Handler();
			  }

		} else if (state == waitingForAlarm) { //this is where the system is put into low power mode
			if (stateChanged) {
				alreadyDisplaying = 0;
				alarmDurationCounter = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
				if (state == waitingForAlarm) stateChanged = 0;
			}
			//go into low power mode
			//__asm__("WFI");

		} else if (state == snooze) {
			//start the shorter timer to wait for the two minutes
			if (stateChanged) {
				alreadyDisplaying = 0;
				snoozeCounter = 0;
				HAL_TIM_Base_Start_IT(&htim7); // Interrupt at 1hz
				stateChanged = 0;
			}
		} else if (state == display) {
			if (stateChanged) {
				HAL_TIM_Base_Start_IT(&htim7);

				Audio_Play();
				//display all the stuff through UART
				int tempSensor = (int) BSP_TSENSOR_ReadTemp();
				int humiditySensor = (int) BSP_HSENSOR_ReadHumidity();
				HAL_RTC_GetTime(&hrtc,&currentTime,RTC_FORMAT_BCD);
				char curTimeStr[6];
				sprintf(curTimeStr, "%02d:%02d", currentTime.Hours, currentTime.Minutes);

				snprintf(uartMsg, sizeof(uartMsg),
						"\r\n[!!!ALARM!!!]\r\n"
						"\r\nTempertature in %s : %d°C | Humidity: %d%%\r\n"
						"The Time is: %s",
				city, tempSensor, humiditySensor, curTimeStr);
				HAL_UART_Transmit(&huart1, (uint8_t*) uartMsg, strlen(uartMsg),HAL_MAX_DELAY);
				stateChanged = 0;

			}
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x30A175AB;
  hi2c1.Init.OwnAddress1 = 0;
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
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
  hlptim1.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_POS1;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable Calibration
  */
  if (HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_1HZ) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 15000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1831;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE11 PE12 PE13
                           PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		uint8_t c = uart_byte;

		if (c == '\r' || c== '\r\n' || c == '\n') { // ENTER pressed

			char_buffer[rx_index] = '\0';
			rx_index = 0;         //buffer index reset
			line_ready = 1;       //indicating "enter" been pressed

		} else {
			// if not \r, append to character buffer
			if (rx_index < (UA_BUF_SIZE - 1)) {
				char_buffer[rx_index++] = c;
			}
		}

		// Restart reception for the next byte (always!)
		HAL_UART_Receive_IT(&huart1, &uart_byte, 1);
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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

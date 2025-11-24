// Includes
#include "main.h"
#include "wifi.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "stm32l4s5i_iot01.h"
#include <stdlib.h>
#include "ringtone_data.h"
#include "audio_player.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_qspi.h"
// WIFI CONFIG
#define WIFI_SSID     "Enohpi"
#define WIFI_PASSWORD "rome1234"
#define WIFI_SECURITY WIFI_ECN_WPA2_PSK

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  30000
#define SOCKET             0

#define UA_BUF_SIZE 64
#define config 0
#define waitingForAlarm 1
#define snooze 2
#define display 3

// Gloabal variables
static uint8_t http_request[512];
static uint8_t http_response[4096];

extern UART_HandleTypeDef hDiscoUart;

extern SPI_HandleTypeDef hspi;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

OSPI_HandleTypeDef hospi1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

static uint8_t audio_ready = 0;
static uint32_t last_button_time = 0;
int state = config; // 0 is configuration, 1 is wait for alarm, 2 is snooze, 3 is display
char *city;

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
char * weatherCond;
char * outsideTemp;
uint8_t alreadyDisplaying = 0;
uint8_t stateChanged;
RTC_TimeTypeDef inputTime = { 0 };
RTC_TimeTypeDef currentTime = { 0 };
RTC_AlarmTypeDef inputAlarm = {0};

// Function prototypes
static int wifi_start(void);
static int wifi_connect(void);
static int http_get_working(void);
char* get_json_value(const char* json, const char* key, char* out, int out_size);

static void SystemClock_Config(void);

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
int parseTime(char* str, int * hour, int * minute);

// printf() prints to UART
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  BSP_LED_Init(LED2);
  MX_GPIO_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();

  /* Initialize UART */
  hDiscoUart.Instance = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate = 115200;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  BSP_COM_Init(COM1, &hDiscoUart);


  //  MX_USART1_UART_Init();
    MX_OCTOSPI1_Init();
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

  printf("\n****** Group 30's AlarmClock  ******\r\n");

  /* Initialize and connect WiFi */
  if (wifi_start() != 0) Error_Handler();
  if (wifi_connect() != 0) Error_Handler();

  /* Perform HTTP request */
//  if (http_get_working() != 0) {
//    printf("HTTP request failed\r\n");
//    Error_Handler();
//  } else {
//    printf("SUCCESS!\r\n");
//    BSP_LED_Toggle(LED2);
//  }
  HAL_UART_Receive_IT(&hDiscoUart, &uart_byte, 1);

  	char uartMsg[256];
  	const char *cityMsg = "[Enter Location]: ";
  	//const char *timerMsg = "\r\n[Enter Current Time (e.g. 14:30)]: ";
  	const char *durationMsg = "\r\n[Enter Alarm Time (e.g. 6:15)]: ";
  	const char * waiting = "\r\n\t[Waiting]";
  	const char * snoozing = "\r\n\t[Snoozing for 2 minutes]";
  	// Intro banner
  	const char *banner = "\r\n=== ALARM CLOCK CONFIG ===\r\n"
  			"You will be asked for:\r\n"
  			"1) Location\r\n"
  			"2) Alarm Time\r\n";
  	HAL_UART_Transmit(&hDiscoUart, (uint8_t*) banner, strlen(banner),
  			HAL_MAX_DELAY);

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
	  						if (strlen((char*)city_buffer) == 0) {
	  							 city = "Montreal";
	  						}else {
	  							city = (char *) city_buffer;
	  						}
	  						http_get_working();
	  						//use the following for input validation
//	  						if (http_get_working() != 0) {
	  						//    printf("HTTP request failed\r\n");
	  						//    Error_Handler();
	  						//  }
	  						printf("Current Time is %d:%d", currentHour, currentMin);
	  						clock_state++;
	  					} else if (clock_state == 1) {
//
//	  						strncpy((char*) timer_buffer, (char*) char_buffer,
//	  								UA_BUF_SIZE - 1);
//	  						timer_buffer[UA_BUF_SIZE - 1] = '\0';
//
//	  						valid_input = parseTime((char*)timer_buffer,&currentHour,&currentMin);
//	  						if (valid_input == -1){
//	  							char * invalidInput = "\r\nTime format is (Hour):(Minute) in 24hr time, Try Again";
//	  							HAL_UART_Transmit(&hDiscoUart, (uint8_t*) invalidInput, strlen(invalidInput), HAL_MAX_DELAY);
//	  						}else clock_state ++;
//	  					} else if (clock_state == 2) {
	  						strncpy((char*) duration_buffer, (char*) char_buffer,
	  								UA_BUF_SIZE - 1);
	  						duration_buffer[UA_BUF_SIZE - 1] = '\0';

	  						valid_input = parseTime((char*)duration_buffer,&alarmHour,&alarmMin);
	  						if (valid_input == -1){
	  							char * invalidInput = "Time format is Hour:Minute\r\nTry Again";
	  							HAL_UART_Transmit(&hDiscoUart, (uint8_t*) invalidInput, strlen(invalidInput), HAL_MAX_DELAY);

	  						} else clock_state++;

	  					}
	  //					clock_state++;
	  					prompt_printed = 0;
	  				}

	  				if (clock_state == 0 && !prompt_printed) {
	  					HAL_UART_Transmit(&hDiscoUart, (uint8_t*) cityMsg, strlen(cityMsg),
	  							HAL_MAX_DELAY);
	  					prompt_printed = 1;
	  				} else if (clock_state == 1 && !prompt_printed) {
//	  					HAL_UART_Transmit(&hDiscoUart, (uint8_t*) timerMsg,
//	  							strlen(timerMsg), HAL_MAX_DELAY);
//	  					prompt_printed = 1;
//	  				} else if (clock_state == 2 && !prompt_printed) {
	  					HAL_UART_Transmit(&hDiscoUart, (uint8_t*) durationMsg,
	  							strlen(durationMsg), HAL_MAX_DELAY);
	  					prompt_printed = 1;
	  				} else if (clock_state == 2 && !prompt_printed && clock_triggered) //only print prompt once, in the future we will add another flag to this condition, "clock_triggered", when alarm goes off
	  						{

	  					int tempSensor = (int) BSP_TSENSOR_ReadTemp();
	  					int humiditySensor = (int) BSP_HSENSOR_ReadHumidity();

	  					snprintf(uartMsg, sizeof(uartMsg),
	  							"\r\n[ALARM_CLOCK] Temp: %d°C | Humidity: %d%%\r\n"
	  									"[CONFIG] Location: %s | Time: %s | Alarm Time: %s\r\n",
	  							tempSensor, humiditySensor, (char*) city_buffer,
	  							(char*) timer_buffer, duration_buffer);

	  					HAL_UART_Transmit(&hDiscoUart, (uint8_t*) uartMsg, strlen(uartMsg),
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
	  			if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET) {
	  				state = snooze;
	  				stateChanged = 1;
	  			}
	  			if (stateChanged) {
	  				HAL_UART_Transmit(&hDiscoUart, (uint8_t*) waiting, strlen(waiting), HAL_MAX_DELAY);
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
	  				HAL_UART_Transmit(&hDiscoUart, (uint8_t*) snoozing, strlen(snoozing), HAL_MAX_DELAY);
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
	  				HAL_RTC_GetTime(&hrtc,&currentTime,RTC_FORMAT_BIN);
	  				char curTimeStr[6];
	  				sprintf(curTimeStr, "%d:%d", currentTime.Hours, currentTime.Minutes);

	  				snprintf(uartMsg, sizeof(uartMsg),
	  						"\r\n[!!!ALARM!!!]\r\n"
	  						"\r\nTempertature in %s : %d°C | Humidity: %d%%\r\n"
	  						"The Time is: %s",
	  				city, tempSensor, humiditySensor, curTimeStr);
	  				HAL_UART_Transmit(&hDiscoUart, (uint8_t*) uartMsg, strlen(uartMsg),HAL_MAX_DELAY);
	  				stateChanged = 0;

	  			}
	  		}
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
static int wifi_start(void)
{
  uint8_t MAC_Addr[6];

  if (WIFI_Init() == WIFI_STATUS_OK) {
    printf("WiFi initialized\r\n");

    if (WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK) {
      printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
             MAC_Addr[0], MAC_Addr[1], MAC_Addr[2],
             MAC_Addr[3], MAC_Addr[4], MAC_Addr[5]);
      return 0;
    }
  }
  printf("WiFi init failed\r\n");
  return -1;
}


static int wifi_connect(void)
{
  uint8_t IP_Addr[4];
  int retry = 0;

  printf("Connecting to: %s\r\n", WIFI_SSID);

  while (retry < 3) {
    if (WIFI_Connect(WIFI_SSID, WIFI_PASSWORD, WIFI_SECURITY) == WIFI_STATUS_OK) {
      printf("WiFi connected\r\n");
      HAL_Delay(3000); // Wait for DHCP

      if (WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK) {
        printf("IP: %d.%d.%d.%d\r\n", IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]);
        return 0;
      }
    }
    printf("Connection failed (attempt %d/3)\r\n", ++retry);
    HAL_Delay(2000);
  }
  return -1;
}


int http_get_working()
{
    WIFI_Status_t status;
    uint8_t server_ip[4] = {185,93,1,249};
    char http_request[512];
    char recv_buffer[1024];        // FIXED: not const
    char full_response[4096];      // store full response
    int full_len = 0;

    uint16_t recv_len = 0;

    printf("=== Getting the Weather ===\r\n");

    status = WIFI_OpenClientConnection(SOCKET, WIFI_TCP_PROTOCOL, "WeatherAPI", server_ip, 80, 0);

    if (status != WIFI_STATUS_OK) {
        printf("ERROR: Open connection failed: %d\r\n", status);
        return -1;
    }

//    sprintf(http_request,
//        "GET /v1/current.json?key=%s&q=%s&aqi=no HTTP/1.1\r\n"
//        "Host: api.weatherapi.com\r\n"
//        "User-Agent: STM32-WiFi\r\n"
//        "Accept: */*\r\n"
//        "Connection: close\r\n\r\n",
//        "65e43dc2e6944954b35162401251311", "Montreal");

    sprintf(http_request,
        "GET /v1/current.json?key=%s&q=%s&aqi=no HTTP/1.1\r\n"
        "Host: api.weatherapi.com\r\n"
        "User-Agent: STM32-WiFi\r\n"
        "Accept: */*\r\n"
        "Connection: close\r\n\r\n",
        "65e43dc2e6944954b35162401251311", city);
    WIFI_SendData(SOCKET, (uint8_t*)http_request, strlen(http_request), &recv_len, 10000);

    memset(recv_buffer, 0, sizeof(recv_buffer));
    full_response[0] = 0;

    uint16_t read_len;
    do {
        read_len = sizeof(recv_buffer);
        status = WIFI_ReceiveData(SOCKET, recv_buffer, sizeof(recv_buffer), &read_len, 2000);

        if (status == WIFI_STATUS_OK && read_len > 0)
        {
            printf("%.*s", read_len, recv_buffer);

            if (full_len + read_len < sizeof(full_response)-1)
            {
                memcpy(full_response + full_len, recv_buffer, read_len);
                full_len += read_len;
                full_response[full_len] = '\0';
            }
        }

    } while (status == WIFI_STATUS_OK);

    // Skip HTTP headers
    char* json_start = strstr(full_response, "\r\n\r\n");
    if (!json_start) {
        printf("NO JSON FOUND\n");
        return -1;
    }
    json_start += 4;

    char buf1[32], buf2[32], buf3[32];

    get_json_value(json_start, "localtime", buf1, sizeof(buf1));
    get_json_value(json_start, "temp_c",   buf2, sizeof(buf2));
    get_json_value(json_start, "text",    buf3, sizeof(buf3)); // "text" will give overcast,sunny, etc.

    char out[128];
    parseTime(buf1, &currentHour, &currentMin);
    weatherCond = buf3;
    outsideTemp = buf2;

    sprintf(out, "Time=%s  Temp=%s C  Sky=%s \r\n", buf1, buf2, buf3);
    printf("%s", out);

    WIFI_CloseClientConnection(SOCKET);

    return 0;
}



char* get_json_value(const char* json, const char* key, char* out, int out_size) {
    int key_len = strlen(key);
    int match_pos = 0;
    int i = 0;

    for (i = 0; json[i] != '\0'; i++) {
        // Match with starting quote
        if (json[i] == '"' && json[i+1] == key[0]) {
            match_pos = 0;
            i++; // skip initial quote
            while (json[i] != '\0' && match_pos < key_len && json[i] == key[match_pos]) {
                i++;
                match_pos++;
            }

            // Check ending quote
            if (match_pos == key_len && json[i] == '"') {
                i++; // skip closing quote

                // skip whitespace until colon
                while (json[i] && (json[i] == ' ' || json[i] == '\t' || json[i] == '\n'))
                    i++;

                if (json[i] != ':')
                    continue; // not a valid key:value

                i++; // skip ':'

                // skip whitespace before value
                while (json[i] && (json[i] == ' ' || json[i] == '\t' || json[i] == '\n'))
                    i++;

                // extract value
                int out_i = 0;

                if (json[i] == '"') {
                    // STRING value
                    i++;
                    while (json[i] && json[i] != '"' && out_i < out_size - 1)
                        out[out_i++] = json[i++];
                } else {
                    // NUMBER value
                    while (json[i] && json[i] != ',' && json[i] != '}' && out_i < out_size - 1) {
                        if (!isspace((unsigned char)json[i]))
                            out[out_i++] = json[i];
                        i++;
                    }
                }

                out[out_i] = '\0';
                return out;
            }
        }
    }

    // not found
    if (out_size > 0)
        out[0] = '\0';
    return out;
}
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
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
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
//static void MX_USART1_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART1_Init 0 */
//
//  /* USER CODE END USART1_Init 0 */
//
//  /* USER CODE BEGIN USART1_Init 1 */
//
//  /* USER CODE END USART1_Init 1 */
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART1_Init 2 */
//
//  /* USER CODE END USART1_Init 2 */
//
//}

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

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
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &hDiscoUart) {
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
		HAL_UART_Receive_IT(&hDiscoUart, &uart_byte, 1);
	}
}



/**
  * @brief  System Clock Configuration
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

/**
  * @brief  Error Handler
  */
void Error_Handler(void)
{
  BSP_LED_On(LED2);
  while(1) {
    HAL_Delay(1000);
  }
}

/**
  * @brief  EXTI line detection callback.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_1) {
    SPI_WIFI_ISR();
  }
}

/**
  * @brief  SPI3 IRQ Handler
  */
void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}

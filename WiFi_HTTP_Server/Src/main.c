
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Working WiFi HTTP Client using proven approach
  ******************************************************************************
  */
#include "main.h"

/* Private defines -----------------------------------------------------------*/
#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  30000  // Longer timeout for receive
#define SOCKET             0

/* Configuration */
#define WIFI_SSID     "SSID"
#define WIFI_PASSWORD "PASSWORD"
#define WIFI_SECURITY WIFI_ECN_WPA2_PSK

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef hDiscoUart;

static uint8_t http_request[512];
static uint8_t http_response[4096];

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static int wifi_start(void);
static int wifi_connect(void);
static int http_get_working(void);

/* Printf redirect */
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/**
  * @brief  Main program
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  BSP_LED_Init(LED2);

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

  printf("\n****** WiFi HTTP Client - WORKING VERSION ******\r\n");

  /* Initialize and connect WiFi */
  if (wifi_start() != 0) Error_Handler();
  if (wifi_connect() != 0) Error_Handler();

  /* Perform HTTP request */
  if (http_get_working() != 0) {
    printf("HTTP request failed\r\n");
    Error_Handler();
  } else {
    printf("SUCCESS!\r\n");
    BSP_LED_Toggle(LED2);
  }

  while (1) {
    HAL_Delay(1000);
  }
}

/**
  * @brief  Initialize WiFi module
  */
static int wifi_start(void)
{
  uint8_t MAC_Addr[6];

  if (WIFI_Init() == WIFI_STATUS_OK) {
    printf("WiFi initialized\r\n");

    if (WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK) {
      printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
             MAC_Addr[0], MAC_Addr[1], MAC_Addr[2],
             MAC_Addr[3], MAC_Addr[4], MAC_Addr[5]);
      return 0;
    }
  }
  printf("WiFi init failed\r\n");
  return -1;
}

/**
  * @brief  Connect to WiFi network
  */
static int wifi_connect(void)
{
  uint8_t IP_Addr[4];
  int retry = 0;

  printf("Connecting to: %s\r\n", WIFI_SSID);

  while (retry < 3) {
    if (WIFI_Connect(WIFI_SSID, WIFI_PASSWORD, WIFI_SECURITY) == WIFI_STATUS_OK) {
      printf("WiFi connected\r\n");
      HAL_Delay(3000); // Wait for DHCP

      if (WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK) {
        printf("IP: %d.%d.%d.%d\r\n", IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]);
        return 0;
      }
    }
    printf("Connection failed (attempt %d/3)\r\n", ++retry);
    HAL_Delay(2000);
  }
  return -1;
}

#include "wifi.h"
#include <stdio.h>
#include <string.h>

#define SOCKET 0

int http_get_working()
{
    WIFI_Status_t status;
    uint8_t server_ip[4] = {185, 93, 1, 249};
    char http_request[512];
    uint8_t recv_buffer[1024];
    uint16_t recv_len = 0;

    printf("=== HTTP GET (WeatherAPI) ===\r\n");
    printf("Server: api.weatherapi.com (185.93.1.249:80)\r\n");

    // 1. Open TCP connection
    status = WIFI_OpenClientConnection(SOCKET, WIFI_TCP_PROTOCOL,
                                       "WeatherAPI", server_ip, 80, 0);
    if (status != WIFI_STATUS_OK) {
        printf("ERROR: Open connection failed: %d\r\n", status);
        return -1;
    }
    printf("[1] Connection opened\r\n");

    // 2. Build and send HTTP request
    sprintf(http_request,
            "GET /v1/current.json?key=%s&q=%s&aqi=no HTTP/1.1\r\n"
            "Host: api.weatherapi.com\r\n"
            "User-Agent: STM32-WiFi\r\n"
            "Accept: */*\r\n"
            "Connection: close\r\n\r\n",
			"65e43dc2e6944954b35162401251311", "Montreal");

    printf("[2] Sending request...\r\n");
    status = WIFI_SendData(SOCKET, (uint8_t *)http_request, strlen(http_request), &recv_len, 10000);
    if (status != WIFI_STATUS_OK) {
        printf("ERROR: Send failed: %d\r\n", status);
        WIFI_CloseClientConnection(SOCKET);
        return -2;
    }
    printf("[2] Sent %u bytes\r\n", (unsigned)recv_len);

    // 3. Receive response
    printf("[3] Waiting for response...\r\n");
    memset(recv_buffer, 0, sizeof(recv_buffer));

    uint16_t read_len;

    do {
        read_len = sizeof(recv_buffer);
        status = WIFI_ReceiveData(SOCKET, recv_buffer, sizeof(recv_buffer), &read_len, 2000);
        if (status == WIFI_STATUS_OK && read_len > 0) {
            printf("%.*s", read_len, recv_buffer);
        } else {
            HAL_Delay(50);
        }
    } while (status == WIFI_STATUS_OK);

    // 4. Close connection
    WIFI_CloseClientConnection(SOCKET);
    printf("\r\n[4] Connection closed\r\n");

    return 0;
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

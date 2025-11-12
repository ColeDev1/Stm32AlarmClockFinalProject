
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
#define WIFI_SECURITY WIFI_ECN_WPA2_PSKf

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

/**
  * @brief  WORKING HTTP GET implementation
  */
/**
  * @brief  HTTP GET to local Python server
  */
static int http_get_working(void)
{
  uint16_t sent_len = 0, recv_len = 0;
  WIFI_Status_t status;

  uint8_t server_ip[4] = {34, 236, 82, 201}; // Your PC's IP on local network

  printf("\n=== HTTP GET ===\r\n");
  printf("Server: %d.%d.%d.%d:80\r\n", server_ip[0], server_ip[1], server_ip[2], server_ip[3]);

  /* Step 1: Open connection */
  printf("[1] Opening connection...\r\n");
  status = WIFI_OpenClientConnection(SOCKET, WIFI_TCP_PROTOCOL, "HTTP_Client", server_ip, 80, 0);  // Port 80
  if (status != WIFI_STATUS_OK) {
    printf("ERROR: Open connection failed: %d\r\n", status);
    return -1;
  }
  printf("[1] Connection opened\r\n");

  HAL_Delay(1000); // Small delay before sending

  /* Step 2: Build and send HTTP request */
  printf("[2] Sending request...\r\n");
  sprintf((char *)http_request,
          "GET /get HTTP/1.1\r\n"
          "Host: %d.%d.%d.%d:80\r\n"
          "User-Agent: STM32-WiFi\r\n"
          "Connection: close\r\n"
          "\r\n",
          server_ip[0], server_ip[1], server_ip[2], server_ip[3]);

  status = WIFI_SendData(SOCKET, http_request, strlen((char *)http_request),
                         &sent_len, WIFI_WRITE_TIMEOUT);
  if (status != WIFI_STATUS_OK || sent_len == 0) {
    printf("ERROR: Send failed: %d, sent: %d\r\n", status, sent_len);
    WIFI_CloseClientConnection(SOCKET);
    return -1;
  }
  printf("[2] Sent %d bytes\r\n", sent_len);

  /* Step 3: Wait for response */
  printf("[3] Waiting for response...\r\n");
  HAL_Delay(3000); // Give server time to respond

  /* Step 4: Receive data */
  printf("[4] Receiving data...\r\n");
  memset(http_response, 0, sizeof(http_response));

  status = WIFI_ReceiveData(SOCKET, http_response, sizeof(http_response) - 1,
                           &recv_len, 10000); // 10 second timeout

  printf("[4] Receive status: %d, received: %d bytes\r\n", status, recv_len);

  if (status == WIFI_STATUS_OK && recv_len > 0) {
    printf("\n=== HTTP RESPONSE ===\r\n");
    // Print entire response
    for(int i = 0; i < recv_len; i++) {
      printf("%c", http_response[i]);
    }
    printf("\n=== END RESPONSE ===\r\n");
  } else {
    printf("ERROR: No data received\r\n");

    // Debug: Try to receive with different buffer sizes
    printf("Trying with smaller buffer...\r\n");
    status = WIFI_ReceiveData(SOCKET, http_response, 512, &recv_len, 5000);
    printf("Small buffer receive: status=%d, bytes=%d\r\n", status, recv_len);

    if (recv_len > 0) {
      printf("Data received with small buffer:\r\n");
      for(int i = 0; i < recv_len; i++) {
        printf("%c", http_response[i]);
      }
      printf("\r\n");
    }
  }

  /* Step 5: Close connection */
  printf("[5] Closing connection...\r\n");
  WIFI_CloseClientConnection(SOCKET);
  printf("[5] Connection closed\r\n");

  return (recv_len > 0) ? 0 : -1;
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

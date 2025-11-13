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
// #include "cmsis_os.h"  // REMOVED RTOS

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>    // For printf
#include <string.h>   // For strlen
#include <math.h>     // For sinf
#include <stdlib.h>   // For malloc/free

// QSPI
#include <stm32l4s5i_iot01_qspi.h>

// ADD THIS LINE:
#include "stm32l4xx_hal_dac.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// --- Audio Generation Defines ---
#define AUDIO_SAMPLE_RATE     16000  // 16 kHz sampling rate
#define AUDIO_FREQUENCY       440    // 440 Hz (A4 note)
#define AUDIO_DURATION_SEC    2      // 2 seconds of audio
#define NUM_AUDIO_SAMPLES     (AUDIO_SAMPLE_RATE * AUDIO_DURATION_SEC) // Total samples = 32000
#define AUDIO_DATA_SIZE_BYTES (NUM_AUDIO_SAMPLES * sizeof(uint16_t))   // Total bytes = 64000
#define QSPI_AUDIO_ADDR       0x00000000 // Start address in QSPI

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// I2C_HandleTypeDef hi2c2; // REMOVED - Not used
OSPI_HandleTypeDef hospi1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// --- Added for Audio ---
DAC_HandleTypeDef hdac1;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_dac1_ch1;

// --- Audio Buffers & Flags ---
static uint16_t* g_full_wave_buffer = NULL; // Pointer for RAM buffer (will be malloc'd)
static volatile uint8_t g_is_playing = 0;   // Flag to prevent re-triggering
char uart_buf[100];                         // Buffer for printf
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
// static void MX_I2C2_Init(void); // REMOVED
static void MX_USART1_UART_Init(void);
static void MX_OCTOSPI1_Init(void);

/* USER CODE BEGIN PFP */
// --- Added for Audio ---
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC1_Init(void);

// --- The three functions you requested ---
void generate_and_store_sine(void);
void load_wave_into_ram(void);
void play_audio(void);

// Redirects printf to UART
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
  // MX_I2C2_Init(); // REMOVED
  MX_USART1_UART_Init();
  MX_OCTOSPI1_Init();

  /* USER CODE BEGIN 2 */
  // --- Initialize Audio Peripherals ---
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();

  // --- Initialize QSPI ---
  BSP_QSPI_Init();

  printf("--- Audio Player Initializing ---\r\n");

  // --- STEP 1: Erase QSPI (only need to do this once) ---
  printf("Erasing QSPI chip... (takes a few seconds)\r\n");
  BSP_QSPI_Erase_Chip();
  printf("QSPI Erase Complete.\r\n");

  // --- STEP 2: Call Function 1 (Generate & Store) ---
  printf("Calling generate_and_store_sine()...\r\n");
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); // LED on
  generate_and_store_sine();
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // LED off
  printf("Sine wave stored in QSPI.\r\n");

  // --- STEP 3: Call Function 2 (Load to RAM) ---
  printf("Calling load_wave_into_ram()...\r\n");
  load_wave_into_ram();
  printf("Wave loaded into RAM. Ready to play.\r\n");

  // --- Ready to Play ---
  printf("\r\n*** Press the BLUE USER BUTTON (B1) to play audio ***\r\n");

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS... */
  // ALL RTOS CODE REMOVED
  /* USER CODE END RTOS... */

  /* Start scheduler */
  // osKernelStart(); // REMOVED

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  GPIO_PinState lastButtonState = GPIO_PIN_SET; // Assume button is up

  while (1)
  {
    // --- Simple Button Debounce & Trigger ---
    GPIO_PinState currentButtonState = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);

    // Check for a falling edge (press)
    if (currentButtonState == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET)
    {
      // --- STEP 4: Call Function 3 (Play) ---
      play_audio();
    }

    lastButtonState = currentButtonState;
    HAL_Delay(20); // Poll button every 20ms

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
// static void MX_I2C2_Init(void) // REMOVED
// { ... }

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE(); // ADDED for DAC pin

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
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

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* ADDED: Configure DAC output pin */
  /* PA4 is DAC1_OUT1 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// --- ADDED NEW PERIPHERAL INIT FUNCTIONS ---

/**
  * @brief DMA controller clock enablement
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  /* Set timer update frequency to the audio sample rate */
  /* SysCoreClock is 120MHz. Prescaler=0. Period = (120MHz / 16000Hz) - 1 = 7499 */
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = (SystemCoreClock / AUDIO_SAMPLE_RATE) - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE; // Trigger DAC on update
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
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
  DAC_ChannelConfTypeDef sConfig = {0};

  /* DAC Initialization */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /* DAC channel1 Configuration */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO; // Triggered by TIM6
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipBooster = DAC_CHIPBOOSTER_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Connect DMA to DAC */
  hdma_dac1_ch1.Instance = DMA1_Channel3;
  hdma_dac1_ch1.Init.Request = DMA_REQUEST_DAC1_CH1;
  hdma_dac1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_dac1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dac1_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 12-bit data in 16-bit
  hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_dac1_ch1.Init.Mode = DMA_NORMAL; // DMA runs once
  hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&hdac1, DMA_Handle1, hdma_dac1_ch1);
}

// --- FUNCTION 1 ---
/**
  * @brief  Generates a sine wave and writes it to QSPI flash in chunks.
  */
void generate_and_store_sine(void)
{
  // Use a small buffer to generate and write in chunks
  #define SINE_WRITE_BUFFER_SIZE 256 // 256 samples * 2 bytes/sample = 512 bytes
  uint16_t sine_buffer[SINE_WRITE_BUFFER_SIZE];

  uint32_t qspi_addr = QSPI_AUDIO_ADDR;
  uint32_t buffer_index = 0;

  for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES; i++)
  {
      // Calculate sine wave value: (sinf(...) + 1) -> range [0, 2]
      // * 2047.5f -> range [0, 4095] (for 12-bit DAC)
      float val = (sinf(2.0f * M_PI * AUDIO_FREQUENCY * i / AUDIO_SAMPLE_RATE) + 1.0f) * 2047.5f;
      sine_buffer[buffer_index] = (uint16_t)val;
      buffer_index++;

      // If buffer is full, write to QSPI
      if (buffer_index == SINE_WRITE_BUFFER_SIZE)
      {
          BSP_QSPI_Write((uint8_t*)sine_buffer, qspi_addr, SINE_WRITE_BUFFER_SIZE * sizeof(uint16_t));
          qspi_addr += (SINE_WRITE_BUFFER_SIZE * sizeof(uint16_t));
          buffer_index = 0; // Reset buffer index
      }
  }

  // Write any remaining data
  if (buffer_index > 0)
  {
      BSP_QSPI_Write((uint8_t*)sine_buffer, qspi_addr, buffer_index * sizeof(uint16_t));
  }
}

// --- FUNCTION 2 ---
/**
  * @brief  Allocates RAM and copies the entire sine wave from QSPI into it.
  */
void load_wave_into_ram(void)
{
  // Allocate memory for the entire wave
  // Note: This requires 64000 bytes of RAM!
  g_full_wave_buffer = (uint16_t*)malloc(AUDIO_DATA_SIZE_BYTES);

  if (g_full_wave_buffer == NULL)
  {
    // Malloc failed! Not enough RAM.
    printf("FATAL ERROR: Failed to malloc %lu bytes!\r\n", (unsigned long)AUDIO_DATA_SIZE_BYTES);
    Error_Handler();
  }

  // Read the entire wave from QSPI into the allocated RAM buffer
  BSP_QSPI_Read((uint8_t*)g_full_wave_buffer, QSPI_AUDIO_ADDR, AUDIO_DATA_SIZE_BYTES);
}

// --- FUNCTION 3 ---
/**
  * @brief  Starts playing the audio from the RAM buffer.
  */
void play_audio(void)
{
  // Check if buffer is valid and if we are already playing
  if (g_full_wave_buffer == NULL || g_is_playing)
  {
    return; // Do nothing
  }

  g_is_playing = 1; // Set the "playing" flag

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); // LED on
  printf("Playback started...\r\n");

  // Start the Timer (triggers DAC)
  HAL_TIM_Base_Start(&htim6);

  // Start the DAC with DMA
  // It will send NUM_AUDIO_SAMPLES from g_full_wave_buffer to the DAC
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)g_full_wave_buffer, NUM_AUDIO_SAMPLES, DAC_ALIGN_12B_R);
}


// --- ADDED DMA Complete Callback ---
/**
  * @brief  Conversion complete callback in non-blocking mode for DAC channel 1
  * @note   This is an interrupt handler! It's called when the DMA is finished.
  */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
    // Stop the DAC and Timer
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim6);

    printf("Playback finished.\r\n");
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // LED off

    g_is_playing = 0; // Clear the "playing" flag
}

/**
 * @brief  DAC error callback
 */
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac)
{
  printf("DAC Error!\r\n");
  g_is_playing = 0;
  Error_Handler();
}

/* USER CODE END 4 */

// --- All RTOS task functions have been REMOVED ---

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
  printf("--- In Error_Handler() ---\r\n");
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); // Turn on LED
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  printf("Wrong parameters value: file %s on line %d\r\n", (char*)file, (int)line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

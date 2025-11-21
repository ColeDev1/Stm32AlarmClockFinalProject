#include "audio_player.h"
#include "main.h"
#include "stm32l4s5i_iot01_qspi.h"

// External handles
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim6;

// Simple configuration
#define QSPI_ADDR       0x000000
#define BUFFER_SIZE     2048
#define SAMPLE_RATE     8000

// Audio state
static uint32_t audio_size = 0;
static uint32_t current_pos = 0;
static uint8_t buffer[BUFFER_SIZE];
static uint8_t playing = 0;
static uint32_t buf_idx = 0;
static uint32_t buf_size = 0;

void Audio_Init(void) {
    // Initialize QSPI
    BSP_QSPI_Init();
    
    // Start DAC
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    
    // Configure timer for 8kHz sample rate
    // 80MHz / (10 * 1000) = 8000 Hz
    __HAL_TIM_SET_PRESCALER(&htim6, 9);
    __HAL_TIM_SET_AUTORELOAD(&htim6, 999);
}

void Audio_Store(const uint8_t* data, uint32_t size) {
    audio_size = size;
    
    // Erase enough 64KB blocks
    uint32_t blocks = (size + 0xFFFF) / 0x10000;
    for (uint32_t i = 0; i < blocks; i++) {
        BSP_QSPI_Erase_Block(QSPI_ADDR + (i * 0x10000));
    }
    
    // Write data in 256-byte chunks
    uint32_t offset = 0;
    while (offset < size) {
        uint32_t chunk = (size - offset > 256) ? 256 : (size - offset);
        BSP_QSPI_Write((uint8_t*)(data + offset), QSPI_ADDR + offset, chunk);
        offset += chunk;
    }
}

void Audio_Play(void) {
    if (audio_size == 0) return;
    
    // Reset playback
    current_pos = 0;
    buf_idx = 0;
    buf_size = 0;
    playing = 1;
    
    // Start timer
    HAL_TIM_Base_Start_IT(&htim6);
}

void Audio_Stop(void) {
    playing = 0;
    HAL_TIM_Base_Stop_IT(&htim6);
    
    // Center DAC output
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
}

void Audio_TimerCallback(void) {
    if (!playing) return;
    
    // Refill buffer when empty
    if (buf_idx >= buf_size) {
        uint32_t remaining = audio_size - current_pos;
        
        // End of audio?
        if (remaining == 0) {
            Audio_Stop();
            return;
        }
        
        // Read next chunk from QSPI
        buf_size = (remaining > BUFFER_SIZE) ? BUFFER_SIZE : remaining;
        BSP_QSPI_Read(buffer, QSPI_ADDR + current_pos, buf_size);
        buf_idx = 0;
        current_pos += buf_size;
    }
    
    // Get 16-bit sample (little-endian)
    int16_t sample = (int16_t)(buffer[buf_idx] | (buffer[buf_idx + 1] << 8));
    
    // Convert to 12-bit DAC value (0-4095)
    uint16_t dac_val = (uint16_t)((sample + 32768) >> 4);
    
    // Output to DAC
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_val);
    
    // Move to next sample
    buf_idx += 2;
}

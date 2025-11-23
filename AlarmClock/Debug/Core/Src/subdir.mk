################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/audio_player.c \
../Core/Src/main.c \
../Core/Src/stm32l4s5i_iot01.c \
../Core/Src/stm32l4s5i_iot01_accelero.c \
../Core/Src/stm32l4s5i_iot01_gyro.c \
../Core/Src/stm32l4s5i_iot01_hsensor.c \
../Core/Src/stm32l4s5i_iot01_magneto.c \
../Core/Src/stm32l4s5i_iot01_psensor.c \
../Core/Src/stm32l4s5i_iot01_qspi.c \
../Core/Src/stm32l4s5i_iot01_tsensor.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_hal_spi.c \
../Core/Src/stm32l4xx_hal_spi_ex.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/audio_player.o \
./Core/Src/main.o \
./Core/Src/stm32l4s5i_iot01.o \
./Core/Src/stm32l4s5i_iot01_accelero.o \
./Core/Src/stm32l4s5i_iot01_gyro.o \
./Core/Src/stm32l4s5i_iot01_hsensor.o \
./Core/Src/stm32l4s5i_iot01_magneto.o \
./Core/Src/stm32l4s5i_iot01_psensor.o \
./Core/Src/stm32l4s5i_iot01_qspi.o \
./Core/Src/stm32l4s5i_iot01_tsensor.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_hal_spi.o \
./Core/Src/stm32l4xx_hal_spi_ex.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/audio_player.d \
./Core/Src/main.d \
./Core/Src/stm32l4s5i_iot01.d \
./Core/Src/stm32l4s5i_iot01_accelero.d \
./Core/Src/stm32l4s5i_iot01_gyro.d \
./Core/Src/stm32l4s5i_iot01_hsensor.d \
./Core/Src/stm32l4s5i_iot01_magneto.d \
./Core/Src/stm32l4s5i_iot01_psensor.d \
./Core/Src/stm32l4s5i_iot01_qspi.d \
./Core/Src/stm32l4s5i_iot01_tsensor.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_hal_spi.d \
./Core/Src/stm32l4xx_hal_spi_ex.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Core/Components" -I"C:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/BSP/Components/Common" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/hts221" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/st25dv" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/lis3mdl" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/Common" -I../Core/Inc -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/audio_player.cyclo ./Core/Src/audio_player.d ./Core/Src/audio_player.o ./Core/Src/audio_player.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32l4s5i_iot01.cyclo ./Core/Src/stm32l4s5i_iot01.d ./Core/Src/stm32l4s5i_iot01.o ./Core/Src/stm32l4s5i_iot01.su ./Core/Src/stm32l4s5i_iot01_accelero.cyclo ./Core/Src/stm32l4s5i_iot01_accelero.d ./Core/Src/stm32l4s5i_iot01_accelero.o ./Core/Src/stm32l4s5i_iot01_accelero.su ./Core/Src/stm32l4s5i_iot01_gyro.cyclo ./Core/Src/stm32l4s5i_iot01_gyro.d ./Core/Src/stm32l4s5i_iot01_gyro.o ./Core/Src/stm32l4s5i_iot01_gyro.su ./Core/Src/stm32l4s5i_iot01_hsensor.cyclo ./Core/Src/stm32l4s5i_iot01_hsensor.d ./Core/Src/stm32l4s5i_iot01_hsensor.o ./Core/Src/stm32l4s5i_iot01_hsensor.su ./Core/Src/stm32l4s5i_iot01_magneto.cyclo ./Core/Src/stm32l4s5i_iot01_magneto.d ./Core/Src/stm32l4s5i_iot01_magneto.o ./Core/Src/stm32l4s5i_iot01_magneto.su ./Core/Src/stm32l4s5i_iot01_psensor.cyclo ./Core/Src/stm32l4s5i_iot01_psensor.d ./Core/Src/stm32l4s5i_iot01_psensor.o ./Core/Src/stm32l4s5i_iot01_psensor.su ./Core/Src/stm32l4s5i_iot01_qspi.cyclo ./Core/Src/stm32l4s5i_iot01_qspi.d ./Core/Src/stm32l4s5i_iot01_qspi.o ./Core/Src/stm32l4s5i_iot01_qspi.su ./Core/Src/stm32l4s5i_iot01_tsensor.cyclo ./Core/Src/stm32l4s5i_iot01_tsensor.d ./Core/Src/stm32l4s5i_iot01_tsensor.o ./Core/Src/stm32l4s5i_iot01_tsensor.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_hal_spi.cyclo ./Core/Src/stm32l4xx_hal_spi.d ./Core/Src/stm32l4xx_hal_spi.o ./Core/Src/stm32l4xx_hal_spi.su ./Core/Src/stm32l4xx_hal_spi_ex.cyclo ./Core/Src/stm32l4xx_hal_spi_ex.d ./Core/Src/stm32l4xx_hal_spi_ex.o ./Core/Src/stm32l4xx_hal_spi_ex.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src


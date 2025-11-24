################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/Src/audio_player.c \
../Application/User/Src/stm32l4s5i_iot01_accelero.c \
../Application/User/Src/stm32l4s5i_iot01_gyro.c \
../Application/User/Src/stm32l4s5i_iot01_hsensor.c \
../Application/User/Src/stm32l4s5i_iot01_magneto.c \
../Application/User/Src/stm32l4s5i_iot01_psensor.c \
../Application/User/Src/stm32l4s5i_iot01_qspi.c \
../Application/User/Src/stm32l4xx_hal_msp.c \
../Application/User/Src/stm32l4xx_it.c \
../Application/User/Src/syscalls.c 

OBJS += \
./Application/User/Src/audio_player.o \
./Application/User/Src/stm32l4s5i_iot01_accelero.o \
./Application/User/Src/stm32l4s5i_iot01_gyro.o \
./Application/User/Src/stm32l4s5i_iot01_hsensor.o \
./Application/User/Src/stm32l4s5i_iot01_magneto.o \
./Application/User/Src/stm32l4s5i_iot01_psensor.o \
./Application/User/Src/stm32l4s5i_iot01_qspi.o \
./Application/User/Src/stm32l4xx_hal_msp.o \
./Application/User/Src/stm32l4xx_it.o \
./Application/User/Src/syscalls.o 

C_DEPS += \
./Application/User/Src/audio_player.d \
./Application/User/Src/stm32l4s5i_iot01_accelero.d \
./Application/User/Src/stm32l4s5i_iot01_gyro.d \
./Application/User/Src/stm32l4s5i_iot01_hsensor.d \
./Application/User/Src/stm32l4s5i_iot01_magneto.d \
./Application/User/Src/stm32l4s5i_iot01_psensor.d \
./Application/User/Src/stm32l4s5i_iot01_qspi.d \
./Application/User/Src/stm32l4xx_hal_msp.d \
./Application/User/Src/stm32l4xx_it.d \
./Application/User/Src/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/Src/%.o Application/User/Src/%.su Application/User/Src/%.cyclo: ../Application/User/Src/%.c Application/User/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I"C:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/STM32CubeIDE/Application/User/Inc" -I"C:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/STM32CubeIDE/Application/User/Src" -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-Src

clean-Application-2f-User-2f-Src:
	-$(RM) ./Application/User/Src/audio_player.cyclo ./Application/User/Src/audio_player.d ./Application/User/Src/audio_player.o ./Application/User/Src/audio_player.su ./Application/User/Src/stm32l4s5i_iot01_accelero.cyclo ./Application/User/Src/stm32l4s5i_iot01_accelero.d ./Application/User/Src/stm32l4s5i_iot01_accelero.o ./Application/User/Src/stm32l4s5i_iot01_accelero.su ./Application/User/Src/stm32l4s5i_iot01_gyro.cyclo ./Application/User/Src/stm32l4s5i_iot01_gyro.d ./Application/User/Src/stm32l4s5i_iot01_gyro.o ./Application/User/Src/stm32l4s5i_iot01_gyro.su ./Application/User/Src/stm32l4s5i_iot01_hsensor.cyclo ./Application/User/Src/stm32l4s5i_iot01_hsensor.d ./Application/User/Src/stm32l4s5i_iot01_hsensor.o ./Application/User/Src/stm32l4s5i_iot01_hsensor.su ./Application/User/Src/stm32l4s5i_iot01_magneto.cyclo ./Application/User/Src/stm32l4s5i_iot01_magneto.d ./Application/User/Src/stm32l4s5i_iot01_magneto.o ./Application/User/Src/stm32l4s5i_iot01_magneto.su ./Application/User/Src/stm32l4s5i_iot01_psensor.cyclo ./Application/User/Src/stm32l4s5i_iot01_psensor.d ./Application/User/Src/stm32l4s5i_iot01_psensor.o ./Application/User/Src/stm32l4s5i_iot01_psensor.su ./Application/User/Src/stm32l4s5i_iot01_qspi.cyclo ./Application/User/Src/stm32l4s5i_iot01_qspi.d ./Application/User/Src/stm32l4s5i_iot01_qspi.o ./Application/User/Src/stm32l4s5i_iot01_qspi.su ./Application/User/Src/stm32l4xx_hal_msp.cyclo ./Application/User/Src/stm32l4xx_hal_msp.d ./Application/User/Src/stm32l4xx_hal_msp.o ./Application/User/Src/stm32l4xx_hal_msp.su ./Application/User/Src/stm32l4xx_it.cyclo ./Application/User/Src/stm32l4xx_it.d ./Application/User/Src/stm32l4xx_it.o ./Application/User/Src/stm32l4xx_it.su ./Application/User/Src/syscalls.cyclo ./Application/User/Src/syscalls.d ./Application/User/Src/syscalls.o ./Application/User/Src/syscalls.su

.PHONY: clean-Application-2f-User-2f-Src


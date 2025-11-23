################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Components/wm8994/wm8994.c 

OBJS += \
./Core/Components/wm8994/wm8994.o 

C_DEPS += \
./Core/Components/wm8994/wm8994.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Components/wm8994/%.o Core/Components/wm8994/%.su Core/Components/wm8994/%.cyclo: ../Core/Components/wm8994/%.c Core/Components/wm8994/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Core/Components" -I"C:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/BSP/Components/Common" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/hts221" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/st25dv" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/lis3mdl" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/Common" -I../Core/Inc -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Components-2f-wm8994

clean-Core-2f-Components-2f-wm8994:
	-$(RM) ./Core/Components/wm8994/wm8994.cyclo ./Core/Components/wm8994/wm8994.d ./Core/Components/wm8994/wm8994.o ./Core/Components/wm8994/wm8994.su

.PHONY: clean-Core-2f-Components-2f-wm8994


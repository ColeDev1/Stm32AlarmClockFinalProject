################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Components/st25dv/st25dv.c \
../Core/Components/st25dv/st25dv_reg.c 

OBJS += \
./Core/Components/st25dv/st25dv.o \
./Core/Components/st25dv/st25dv_reg.o 

C_DEPS += \
./Core/Components/st25dv/st25dv.d \
./Core/Components/st25dv/st25dv_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Components/st25dv/%.o Core/Components/st25dv/%.su Core/Components/st25dv/%.cyclo: ../Core/Components/st25dv/%.c Core/Components/st25dv/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Core/Components" -I"C:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/BSP/Components/Common" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/hts221" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/st25dv" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/lis3mdl" -I"C:/Users/colea/STM32CubeIDE/workspace_1.19.0/AlarmClock/Drivers/Components/Common" -I../Core/Inc -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/colea/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Components-2f-st25dv

clean-Core-2f-Components-2f-st25dv:
	-$(RM) ./Core/Components/st25dv/st25dv.cyclo ./Core/Components/st25dv/st25dv.d ./Core/Components/st25dv/st25dv.o ./Core/Components/st25dv/st25dv.su ./Core/Components/st25dv/st25dv_reg.cyclo ./Core/Components/st25dv/st25dv_reg.d ./Core/Components/st25dv/st25dv_reg.o ./Core/Components/st25dv/st25dv_reg.su

.PHONY: clean-Core-2f-Components-2f-st25dv


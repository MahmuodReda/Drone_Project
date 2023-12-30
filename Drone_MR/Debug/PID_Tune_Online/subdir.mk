################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PID_Tune_Online/PID_Tune_Online.c 

OBJS += \
./PID_Tune_Online/PID_Tune_Online.o 

C_DEPS += \
./PID_Tune_Online/PID_Tune_Online.d 


# Each subdirectory must supply rules for building sources it contributes
PID_Tune_Online/%.o PID_Tune_Online/%.su: ../PID_Tune_Online/%.c PID_Tune_Online/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I"C:/Users/SYSTEM_10_MAHUOK/STM32CubeIDE/workspace_1.9.0/Drone_MR/Draw_Parameter" -I../Core/Inc -I"C:/Users/SYSTEM_10_MAHUOK/STM32CubeIDE/workspace_1.9.0/Drone_MR/pid" -I"C:/Users/SYSTEM_10_MAHUOK/STM32CubeIDE/workspace_1.9.0/Drone_MR/PID_Tune_Online" -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-PID_Tune_Online

clean-PID_Tune_Online:
	-$(RM) ./PID_Tune_Online/PID_Tune_Online.d ./PID_Tune_Online/PID_Tune_Online.o ./PID_Tune_Online/PID_Tune_Online.su

.PHONY: clean-PID_Tune_Online


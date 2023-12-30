################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../wq/ESC_Initialization.c 

OBJS += \
./wq/ESC_Initialization.o 

C_DEPS += \
./wq/ESC_Initialization.d 


# Each subdirectory must supply rules for building sources it contributes
wq/%.o wq/%.su: ../wq/%.c wq/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I"C:/Users/SYSTEM_10_MAHUOK/STM32CubeIDE/workspace_1.9.0/Drone_MR/pid" -I"C:/Users/SYSTEM_10_MAHUOK/STM32CubeIDE/workspace_1.9.0/Drone_MR/wq" -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-wq

clean-wq:
	-$(RM) ./wq/ESC_Initialization.d ./wq/ESC_Initialization.o ./wq/ESC_Initialization.su

.PHONY: clean-wq


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ESC_initialization/ESC_initialization.c 

OBJS += \
./ESC_initialization/ESC_initialization.o 

C_DEPS += \
./ESC_initialization/ESC_initialization.d 


# Each subdirectory must supply rules for building sources it contributes
ESC_initialization/%.o ESC_initialization/%.su: ../ESC_initialization/%.c ESC_initialization/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ESC_initialization

clean-ESC_initialization:
	-$(RM) ./ESC_initialization/ESC_initialization.d ./ESC_initialization/ESC_initialization.o ./ESC_initialization/ESC_initialization.su

.PHONY: clean-ESC_initialization


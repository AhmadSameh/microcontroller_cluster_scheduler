################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/operations/operations.c 

OBJS += \
./Core/Inc/operations/operations.o 

C_DEPS += \
./Core/Inc/operations/operations.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/operations/%.o Core/Inc/operations/%.su Core/Inc/operations/%.cyclo: ../Core/Inc/operations/%.c Core/Inc/operations/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-operations

clean-Core-2f-Inc-2f-operations:
	-$(RM) ./Core/Inc/operations/operations.cyclo ./Core/Inc/operations/operations.d ./Core/Inc/operations/operations.o ./Core/Inc/operations/operations.su

.PHONY: clean-Core-2f-Inc-2f-operations


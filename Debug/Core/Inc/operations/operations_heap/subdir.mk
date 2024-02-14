################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/operations/operations_heap/operations_heap.c 

OBJS += \
./Core/Inc/operations/operations_heap/operations_heap.o 

C_DEPS += \
./Core/Inc/operations/operations_heap/operations_heap.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/operations/operations_heap/%.o Core/Inc/operations/operations_heap/%.su Core/Inc/operations/operations_heap/%.cyclo: ../Core/Inc/operations/operations_heap/%.c Core/Inc/operations/operations_heap/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-operations-2f-operations_heap

clean-Core-2f-Inc-2f-operations-2f-operations_heap:
	-$(RM) ./Core/Inc/operations/operations_heap/operations_heap.cyclo ./Core/Inc/operations/operations_heap/operations_heap.d ./Core/Inc/operations/operations_heap/operations_heap.o ./Core/Inc/operations/operations_heap/operations_heap.su

.PHONY: clean-Core-2f-Inc-2f-operations-2f-operations_heap


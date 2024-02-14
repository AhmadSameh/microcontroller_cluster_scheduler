################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/sheduler/priority_queue/priority_queue.c 

OBJS += \
./Core/Inc/sheduler/priority_queue/priority_queue.o 

C_DEPS += \
./Core/Inc/sheduler/priority_queue/priority_queue.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/sheduler/priority_queue/%.o Core/Inc/sheduler/priority_queue/%.su Core/Inc/sheduler/priority_queue/%.cyclo: ../Core/Inc/sheduler/priority_queue/%.c Core/Inc/sheduler/priority_queue/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-sheduler-2f-priority_queue

clean-Core-2f-Inc-2f-sheduler-2f-priority_queue:
	-$(RM) ./Core/Inc/sheduler/priority_queue/priority_queue.cyclo ./Core/Inc/sheduler/priority_queue/priority_queue.d ./Core/Inc/sheduler/priority_queue/priority_queue.o ./Core/Inc/sheduler/priority_queue/priority_queue.su

.PHONY: clean-Core-2f-Inc-2f-sheduler-2f-priority_queue


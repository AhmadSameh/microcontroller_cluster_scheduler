################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/sheduler/stack/stack.c 

OBJS += \
./Core/Inc/sheduler/stack/stack.o 

C_DEPS += \
./Core/Inc/sheduler/stack/stack.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/sheduler/stack/%.o Core/Inc/sheduler/stack/%.su Core/Inc/sheduler/stack/%.cyclo: ../Core/Inc/sheduler/stack/%.c Core/Inc/sheduler/stack/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-sheduler-2f-stack

clean-Core-2f-Inc-2f-sheduler-2f-stack:
	-$(RM) ./Core/Inc/sheduler/stack/stack.cyclo ./Core/Inc/sheduler/stack/stack.d ./Core/Inc/sheduler/stack/stack.o ./Core/Inc/sheduler/stack/stack.su

.PHONY: clean-Core-2f-Inc-2f-sheduler-2f-stack


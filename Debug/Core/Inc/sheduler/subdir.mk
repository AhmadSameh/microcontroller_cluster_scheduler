################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/sheduler/sheduler.c 

OBJS += \
./Core/Inc/sheduler/sheduler.o 

C_DEPS += \
./Core/Inc/sheduler/sheduler.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/sheduler/%.o Core/Inc/sheduler/%.su Core/Inc/sheduler/%.cyclo: ../Core/Inc/sheduler/%.c Core/Inc/sheduler/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-sheduler

clean-Core-2f-Inc-2f-sheduler:
	-$(RM) ./Core/Inc/sheduler/sheduler.cyclo ./Core/Inc/sheduler/sheduler.d ./Core/Inc/sheduler/sheduler.o ./Core/Inc/sheduler/sheduler.su

.PHONY: clean-Core-2f-Inc-2f-sheduler


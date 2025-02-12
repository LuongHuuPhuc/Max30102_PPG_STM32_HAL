################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Max30102_lib/Src/max30102_for_stm32_hal.c 

OBJS += \
./Max30102_lib/Src/max30102_for_stm32_hal.o 

C_DEPS += \
./Max30102_lib/Src/max30102_for_stm32_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Max30102_lib/Src/%.o Max30102_lib/Src/%.su Max30102_lib/Src/%.cyclo: ../Max30102_lib/Src/%.c Max30102_lib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/STMproject.Embedded-C/My_workspace/target/max30102_test/Max30102_lib/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Max30102_lib-2f-Src

clean-Max30102_lib-2f-Src:
	-$(RM) ./Max30102_lib/Src/max30102_for_stm32_hal.cyclo ./Max30102_lib/Src/max30102_for_stm32_hal.d ./Max30102_lib/Src/max30102_for_stm32_hal.o ./Max30102_lib/Src/max30102_for_stm32_hal.su

.PHONY: clean-Max30102_lib-2f-Src


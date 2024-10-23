################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ADS1191_Lib/ads1191.c 

OBJS += \
./ADS1191_Lib/ads1191.o 

C_DEPS += \
./ADS1191_Lib/ads1191.d 


# Each subdirectory must supply rules for building sources it contributes
ADS1191_Lib/%.o ADS1191_Lib/%.su ADS1191_Lib/%.cyclo: ../ADS1191_Lib/%.c ADS1191_Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/mfaru/STM32CubeIDE/workspace_1.16.0/F407VGTx_ADS1191/ADS1191_Lib" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ADS1191_Lib

clean-ADS1191_Lib:
	-$(RM) ./ADS1191_Lib/ads1191.cyclo ./ADS1191_Lib/ads1191.d ./ADS1191_Lib/ads1191.o ./ADS1191_Lib/ads1191.su

.PHONY: clean-ADS1191_Lib


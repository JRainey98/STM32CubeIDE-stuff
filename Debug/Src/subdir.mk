################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/017_rtc_lcd.c 

OBJS += \
./Src/017_rtc_lcd.o 

C_DEPS += \
./Src/017_rtc_lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"/Users/jordanrainey/Embedded-C/stm32f4xx_drivers/drivers/Inc" -I../Inc -I"/Users/jordanrainey/Embedded-C/stm32f4xx_drivers/bsp" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/017_rtc_lcd.d ./Src/017_rtc_lcd.o ./Src/017_rtc_lcd.su

.PHONY: clean-Src


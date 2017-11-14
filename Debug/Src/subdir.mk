################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/buffer.c \
../Src/flash.c \
../Src/lsm6ds3.c \
../Src/main.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/system_stm32l4xx.c 

OBJS += \
./Src/buffer.o \
./Src/flash.o \
./Src/lsm6ds3.o \
./Src/main.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o 

C_DEPS += \
./Src/buffer.d \
./Src/flash.d \
./Src/lsm6ds3.d \
./Src/main.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L475xx -I"C:/Users/Station13/Documents/GitHub/SPIFlash/Inc" -I"C:/Users/Station13/Documents/GitHub/SPIFlash/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/Station13/Documents/GitHub/SPIFlash/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Station13/Documents/GitHub/SPIFlash/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/Station13/Documents/GitHub/SPIFlash/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



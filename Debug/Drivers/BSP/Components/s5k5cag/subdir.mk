################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/s5k5cag/s5k5cag.c 

OBJS += \
./Drivers/BSP/Components/s5k5cag/s5k5cag.o 

C_DEPS += \
./Drivers/BSP/Components/s5k5cag/s5k5cag.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/s5k5cag/%.o Drivers/BSP/Components/s5k5cag/%.su Drivers/BSP/Components/s5k5cag/%.cyclo: ../Drivers/BSP/Components/s5k5cag/%.c Drivers/BSP/Components/s5k5cag/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F769xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I.././Drivers/BSP/Components/Common -I.././Drivers/BSP/STM32F769I-Discovery -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-s5k5cag

clean-Drivers-2f-BSP-2f-Components-2f-s5k5cag:
	-$(RM) ./Drivers/BSP/Components/s5k5cag/s5k5cag.cyclo ./Drivers/BSP/Components/s5k5cag/s5k5cag.d ./Drivers/BSP/Components/s5k5cag/s5k5cag.o ./Drivers/BSP/Components/s5k5cag/s5k5cag.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-s5k5cag


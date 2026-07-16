################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/i3g4250d.c \
../Drivers/BSP/stm32f3_discovery.c \
../Drivers/BSP/stm32f3_discovery_accelerometer.c \
../Drivers/BSP/stm32f3_discovery_gyroscope.c 

C_DEPS += \
./Drivers/BSP/i3g4250d.d \
./Drivers/BSP/stm32f3_discovery.d \
./Drivers/BSP/stm32f3_discovery_accelerometer.d \
./Drivers/BSP/stm32f3_discovery_gyroscope.d 

OBJS += \
./Drivers/BSP/i3g4250d.o \
./Drivers/BSP/stm32f3_discovery.o \
./Drivers/BSP/stm32f3_discovery_accelerometer.o \
./Drivers/BSP/stm32f3_discovery_gyroscope.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/%.o Drivers/BSP/%.su Drivers/BSP/%.cyclo: ../Drivers/BSP/%.c Drivers/BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/abdaq/STM32CubeIDE/workspace_1.19.0/Project2scratch/Drivers/BSP" -I"C:/Users/abdaq/STM32CubeIDE/workspace_1.19.0/Project2scratch/Drivers/Components/Common" -I"C:/Users/abdaq/STM32CubeIDE/workspace_1.19.0/Project2scratch/Drivers/Components/l3gd20" -I"C:/Users/abdaq/STM32CubeIDE/workspace_1.19.0/Project2scratch/Drivers/Components/lsm303dlhc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP

clean-Drivers-2f-BSP:
	-$(RM) ./Drivers/BSP/i3g4250d.cyclo ./Drivers/BSP/i3g4250d.d ./Drivers/BSP/i3g4250d.o ./Drivers/BSP/i3g4250d.su ./Drivers/BSP/stm32f3_discovery.cyclo ./Drivers/BSP/stm32f3_discovery.d ./Drivers/BSP/stm32f3_discovery.o ./Drivers/BSP/stm32f3_discovery.su ./Drivers/BSP/stm32f3_discovery_accelerometer.cyclo ./Drivers/BSP/stm32f3_discovery_accelerometer.d ./Drivers/BSP/stm32f3_discovery_accelerometer.o ./Drivers/BSP/stm32f3_discovery_accelerometer.su ./Drivers/BSP/stm32f3_discovery_gyroscope.cyclo ./Drivers/BSP/stm32f3_discovery_gyroscope.d ./Drivers/BSP/stm32f3_discovery_gyroscope.o ./Drivers/BSP/stm32f3_discovery_gyroscope.su

.PHONY: clean-Drivers-2f-BSP


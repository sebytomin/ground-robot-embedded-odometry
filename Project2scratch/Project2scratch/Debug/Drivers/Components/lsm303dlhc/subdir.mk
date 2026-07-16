################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/lsm303dlhc/lsm303dlhc.c 

C_DEPS += \
./Drivers/Components/lsm303dlhc/lsm303dlhc.d 

OBJS += \
./Drivers/Components/lsm303dlhc/lsm303dlhc.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/lsm303dlhc/%.o Drivers/Components/lsm303dlhc/%.su Drivers/Components/lsm303dlhc/%.cyclo: ../Drivers/Components/lsm303dlhc/%.c Drivers/Components/lsm303dlhc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/abdaq/STM32CubeIDE/workspace_1.19.0/Project2scratch/Drivers/BSP" -I"C:/Users/abdaq/STM32CubeIDE/workspace_1.19.0/Project2scratch/Drivers/Components/Common" -I"C:/Users/abdaq/STM32CubeIDE/workspace_1.19.0/Project2scratch/Drivers/Components/l3gd20" -I"C:/Users/abdaq/STM32CubeIDE/workspace_1.19.0/Project2scratch/Drivers/Components/lsm303dlhc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-lsm303dlhc

clean-Drivers-2f-Components-2f-lsm303dlhc:
	-$(RM) ./Drivers/Components/lsm303dlhc/lsm303dlhc.cyclo ./Drivers/Components/lsm303dlhc/lsm303dlhc.d ./Drivers/Components/lsm303dlhc/lsm303dlhc.o ./Drivers/Components/lsm303dlhc/lsm303dlhc.su

.PHONY: clean-Drivers-2f-Components-2f-lsm303dlhc


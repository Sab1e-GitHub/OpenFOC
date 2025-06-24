################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/I2C/Src/i2c.c 

OBJS += \
./Drivers/I2C/Src/i2c.o 

C_DEPS += \
./Drivers/I2C/Src/i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/I2C/Src/%.o Drivers/I2C/Src/%.su Drivers/I2C/Src/%.cyclo: ../Drivers/I2C/Src/%.c Drivers/I2C/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Core/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/ADC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/AS5600/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/I2C/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/INA181AX/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-I2C-2f-Src

clean-Drivers-2f-I2C-2f-Src:
	-$(RM) ./Drivers/I2C/Src/i2c.cyclo ./Drivers/I2C/Src/i2c.d ./Drivers/I2C/Src/i2c.o ./Drivers/I2C/Src/i2c.su

.PHONY: clean-Drivers-2f-I2C-2f-Src


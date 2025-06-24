################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/INA181AX/Src/ina181ax.c 

OBJS += \
./Drivers/INA181AX/Src/ina181ax.o 

C_DEPS += \
./Drivers/INA181AX/Src/ina181ax.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/INA181AX/Src/%.o Drivers/INA181AX/Src/%.su Drivers/INA181AX/Src/%.cyclo: ../Drivers/INA181AX/Src/%.c Drivers/INA181AX/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Core/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/ADC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/AS5600/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/I2C/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/INA181AX/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-INA181AX-2f-Src

clean-Drivers-2f-INA181AX-2f-Src:
	-$(RM) ./Drivers/INA181AX/Src/ina181ax.cyclo ./Drivers/INA181AX/Src/ina181ax.d ./Drivers/INA181AX/Src/ina181ax.o ./Drivers/INA181AX/Src/ina181ax.su

.PHONY: clean-Drivers-2f-INA181AX-2f-Src


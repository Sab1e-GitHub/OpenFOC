################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/WorkSpace/GitHubRepository/OpenFOC/Src/foc.c \
D:/WorkSpace/GitHubRepository/OpenFOC/Src/foc_utils.c \
D:/WorkSpace/GitHubRepository/OpenFOC/Src/lowpass_filter.c \
D:/WorkSpace/GitHubRepository/OpenFOC/Src/pid.c 

OBJS += \
./Drivers/OpenFOC/Src/foc.o \
./Drivers/OpenFOC/Src/foc_utils.o \
./Drivers/OpenFOC/Src/lowpass_filter.o \
./Drivers/OpenFOC/Src/pid.o 

C_DEPS += \
./Drivers/OpenFOC/Src/foc.d \
./Drivers/OpenFOC/Src/foc_utils.d \
./Drivers/OpenFOC/Src/lowpass_filter.d \
./Drivers/OpenFOC/Src/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/OpenFOC/Src/foc.o: D:/WorkSpace/GitHubRepository/OpenFOC/Src/foc.c Drivers/OpenFOC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Core/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/ADC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/AS5600/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/I2C/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/INA181AX/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/OpenFOC/Src/foc_utils.o: D:/WorkSpace/GitHubRepository/OpenFOC/Src/foc_utils.c Drivers/OpenFOC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Core/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/ADC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/AS5600/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/I2C/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/INA181AX/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/OpenFOC/Src/lowpass_filter.o: D:/WorkSpace/GitHubRepository/OpenFOC/Src/lowpass_filter.c Drivers/OpenFOC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Core/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/ADC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/AS5600/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/I2C/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/INA181AX/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/OpenFOC/Src/pid.o: D:/WorkSpace/GitHubRepository/OpenFOC/Src/pid.c Drivers/OpenFOC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Core/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/ADC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/AS5600/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/I2C/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/INA181AX/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc" -I"D:/WorkSpace/GitHubRepository/OpenFOC/Examples/STM32F103C8T6_FOC/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-OpenFOC-2f-Src

clean-Drivers-2f-OpenFOC-2f-Src:
	-$(RM) ./Drivers/OpenFOC/Src/foc.cyclo ./Drivers/OpenFOC/Src/foc.d ./Drivers/OpenFOC/Src/foc.o ./Drivers/OpenFOC/Src/foc.su ./Drivers/OpenFOC/Src/foc_utils.cyclo ./Drivers/OpenFOC/Src/foc_utils.d ./Drivers/OpenFOC/Src/foc_utils.o ./Drivers/OpenFOC/Src/foc_utils.su ./Drivers/OpenFOC/Src/lowpass_filter.cyclo ./Drivers/OpenFOC/Src/lowpass_filter.d ./Drivers/OpenFOC/Src/lowpass_filter.o ./Drivers/OpenFOC/Src/lowpass_filter.su ./Drivers/OpenFOC/Src/pid.cyclo ./Drivers/OpenFOC/Src/pid.d ./Drivers/OpenFOC/Src/pid.o ./Drivers/OpenFOC/Src/pid.su

.PHONY: clean-Drivers-2f-OpenFOC-2f-Src


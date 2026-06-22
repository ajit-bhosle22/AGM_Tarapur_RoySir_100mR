################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/FATFS/App/app_fatfs.c 

OBJS += \
./Drivers/FATFS/App/app_fatfs.o 

C_DEPS += \
./Drivers/FATFS/App/app_fatfs.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/FATFS/App/%.o Drivers/FATFS/App/%.su Drivers/FATFS/App/%.cyclo: ../Drivers/FATFS/App/%.c Drivers/FATFS/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H750xx -c -I"C:/Sayali/Yelsons_project/seawage_treatment_rms/AGM_Mumbai/Drivers/Ethernet_W5500" -I"C:/Sayali/Yelsons_project/seawage_treatment_rms/AGM_Mumbai/Drivers/FATFS" -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-FATFS-2f-App

clean-Drivers-2f-FATFS-2f-App:
	-$(RM) ./Drivers/FATFS/App/app_fatfs.cyclo ./Drivers/FATFS/App/app_fatfs.d ./Drivers/FATFS/App/app_fatfs.o ./Drivers/FATFS/App/app_fatfs.su

.PHONY: clean-Drivers-2f-FATFS-2f-App


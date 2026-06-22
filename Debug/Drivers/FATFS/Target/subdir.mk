################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/FATFS/Target/user_diskio.c 

OBJS += \
./Drivers/FATFS/Target/user_diskio.o 

C_DEPS += \
./Drivers/FATFS/Target/user_diskio.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/FATFS/Target/%.o Drivers/FATFS/Target/%.su Drivers/FATFS/Target/%.cyclo: ../Drivers/FATFS/Target/%.c Drivers/FATFS/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H750xx -c -I"C:/Sayali/Yelsons_project/seawage_treatment_rms/AGM_Mumbai/Drivers/Ethernet_W5500" -I"C:/Sayali/Yelsons_project/seawage_treatment_rms/AGM_Mumbai/Drivers/FATFS" -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-FATFS-2f-Target

clean-Drivers-2f-FATFS-2f-Target:
	-$(RM) ./Drivers/FATFS/Target/user_diskio.cyclo ./Drivers/FATFS/Target/user_diskio.d ./Drivers/FATFS/Target/user_diskio.o ./Drivers/FATFS/Target/user_diskio.su

.PHONY: clean-Drivers-2f-FATFS-2f-Target


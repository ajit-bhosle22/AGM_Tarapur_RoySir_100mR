################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Ethernet_W5500/socket.c \
../Drivers/Ethernet_W5500/tcp_client.c \
../Drivers/Ethernet_W5500/tcp_server.c \
../Drivers/Ethernet_W5500/wizchip_conf.c \
../Drivers/Ethernet_W5500/wizchip_port.c 

OBJS += \
./Drivers/Ethernet_W5500/socket.o \
./Drivers/Ethernet_W5500/tcp_client.o \
./Drivers/Ethernet_W5500/tcp_server.o \
./Drivers/Ethernet_W5500/wizchip_conf.o \
./Drivers/Ethernet_W5500/wizchip_port.o 

C_DEPS += \
./Drivers/Ethernet_W5500/socket.d \
./Drivers/Ethernet_W5500/tcp_client.d \
./Drivers/Ethernet_W5500/tcp_server.d \
./Drivers/Ethernet_W5500/wizchip_conf.d \
./Drivers/Ethernet_W5500/wizchip_port.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Ethernet_W5500/%.o Drivers/Ethernet_W5500/%.su Drivers/Ethernet_W5500/%.cyclo: ../Drivers/Ethernet_W5500/%.c Drivers/Ethernet_W5500/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H750xx -c -I"C:/Sayali/Yelsons_project/seawage_treatment_rms/AGM_Mumbai/Drivers/Ethernet_W5500" -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Ethernet_W5500

clean-Drivers-2f-Ethernet_W5500:
	-$(RM) ./Drivers/Ethernet_W5500/socket.cyclo ./Drivers/Ethernet_W5500/socket.d ./Drivers/Ethernet_W5500/socket.o ./Drivers/Ethernet_W5500/socket.su ./Drivers/Ethernet_W5500/tcp_client.cyclo ./Drivers/Ethernet_W5500/tcp_client.d ./Drivers/Ethernet_W5500/tcp_client.o ./Drivers/Ethernet_W5500/tcp_client.su ./Drivers/Ethernet_W5500/tcp_server.cyclo ./Drivers/Ethernet_W5500/tcp_server.d ./Drivers/Ethernet_W5500/tcp_server.o ./Drivers/Ethernet_W5500/tcp_server.su ./Drivers/Ethernet_W5500/wizchip_conf.cyclo ./Drivers/Ethernet_W5500/wizchip_conf.d ./Drivers/Ethernet_W5500/wizchip_conf.o ./Drivers/Ethernet_W5500/wizchip_conf.su ./Drivers/Ethernet_W5500/wizchip_port.cyclo ./Drivers/Ethernet_W5500/wizchip_port.d ./Drivers/Ethernet_W5500/wizchip_port.o ./Drivers/Ethernet_W5500/wizchip_port.su

.PHONY: clean-Drivers-2f-Ethernet_W5500


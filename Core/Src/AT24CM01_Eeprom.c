
#include <stdio.h>
#include <string.h>
#include "AT24CM01_Eeprom.h"
#include "lcd.h"
#include "modbus.h"
#include "DAC_Drvr.h"
#include "main_global.h"

device_config_t g_config;

// --------------------- Write Byte ------------------------------
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t memAddr, uint8_t data)
{
    uint8_t buf[3];
    buf[0] = (uint8_t)(memAddr >> 8);   // High byte of word address
    buf[1] = (uint8_t)(memAddr & 0xFF); // Low byte
    buf[2] = data;

    return HAL_I2C_Master_Transmit(&hi2c4, EEPROM_I2C_ADDR, buf, 3, HAL_MAX_DELAY);
}

// --------------------- Read Byte ------------------------------
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t memAddr, uint8_t *data)
{
    uint8_t addr[2];
    addr[0] = (uint8_t)(memAddr >> 8);
    addr[1] = (uint8_t)(memAddr & 0xFF);

    // Dummy write (send memory address)
    if (HAL_I2C_Master_Transmit(&hi2c4, EEPROM_I2C_ADDR, addr, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Read one byte
    return HAL_I2C_Master_Receive(&hi2c4, EEPROM_I2C_ADDR, data, 1, HAL_MAX_DELAY);
}

// --------------------- Write Page --------------------------------------------
HAL_StatusTypeDef EEPROM_WritePage(uint16_t memAddr, uint8_t *data, uint16_t len)
{
    if (len > EEPROM_PAGE_SIZE) return HAL_ERROR;

    uint8_t buf[EEPROM_PAGE_SIZE + 2];
    buf[0] = (uint8_t)(memAddr >> 8);
    buf[1] = (uint8_t)(memAddr & 0xFF);

    memcpy(&buf[2], data, len);

    return HAL_I2C_Master_Transmit(&hi2c4, EEPROM_I2C_ADDR, buf, len + 2, HAL_MAX_DELAY);
}


HAL_StatusTypeDef EEPROM_Write_Config(I2C_HandleTypeDef *hi2c, uint16_t addr, device_config_t *cfg) {

    // Wait until EEPROM is ready
    while (HAL_I2C_IsDeviceReady(hi2c, EEPROM_I2C_ADDR, 3, HAL_MAX_DELAY) != HAL_OK);

    HAL_StatusTypeDef status = HAL_ERROR;

    // Retry write up to 3 times
    for (int i = 0; i < 3; i++) {
        status = HAL_I2C_Mem_Write(hi2c, EEPROM_I2C_ADDR, addr,
                                   I2C_MEMADD_SIZE_16BIT,
                                   (uint8_t *)cfg, sizeof(device_config_t),
                                   HAL_MAX_DELAY);

        if (status == HAL_OK)
            break;

        HAL_Delay(5);  // short delay between retries
    }

    // Wait until EEPROM has completed the write cycle
    while (HAL_I2C_IsDeviceReady(hi2c, EEPROM_I2C_ADDR, 3, HAL_MAX_DELAY) != HAL_OK);

    return status;
}

HAL_StatusTypeDef EEPROM_Read_Config(I2C_HandleTypeDef *hi2c, uint16_t addr, device_config_t *cfg) {

    HAL_StatusTypeDef status = HAL_ERROR;

    // Retry read up to 3 times
    for (int i = 0; i < 3; i++) {
        status = HAL_I2C_Mem_Read(hi2c, EEPROM_I2C_ADDR, addr,
                                  I2C_MEMADD_SIZE_16BIT,
                                  (uint8_t *)cfg, sizeof(device_config_t),
                                  HAL_MAX_DELAY);

        if (status == HAL_OK)
            break;

        HAL_Delay(5);  // avoid I2C bus hammering
    }

    if (status != HAL_OK)
        return status;

      return HAL_OK;

}

HAL_StatusTypeDef WaitForEEPROM(I2C_HandleTypeDef *hi2c)
{
    uint32_t timeout = 100;  // ms
    while (HAL_I2C_IsDeviceReady(hi2c, EEPROM_I2C_ADDR, 1, timeout) != HAL_OK);
    return HAL_OK;
}

void cpy_reg_to_eeprom_reg(void)
{
	g_config.AUDIO_MODE                = Modbus_Registers.AUDIO_MODE;
	g_config.FREQ_RECV                 = Modbus_Registers.FREQ_RECV;
	g_config.FREQ_DTC                  = Modbus_Registers.FREQ_DTC;
	g_config.AGM_MODE                  = Modbus_Registers.AGM_MODE;
	g_config.Primary_Unit              = Modbus_Registers.Primary_Unit;
	g_config.Alarm_Value_MSB           = Modbus_Registers.Alarm_Value_MSB;
	g_config.Alarm_Value_LSB           = Modbus_Registers.Alarm_Value_LSB;
	g_config.Alarm_Unit                = Modbus_Registers.Alarm_Unit;
	g_config.RS485_Baud_Rate           = Modbus_Registers.RS485_Baud_Rate;
	g_config.RS485_Slave_Id            = Modbus_Registers.RS485_Slave_Id;
	g_config.Ethernet_IP_MSB           = Modbus_Registers.Ethernet_IP_MSB;
	g_config.Ethernet_IP_LSB           = Modbus_Registers.Ethernet_IP_LSB;
	g_config.Ethernet_Subnet_MSB       = Modbus_Registers.Ethernet_Subnet_MSB;
	g_config.Ethernet_Subnet_LSB       = Modbus_Registers.Ethernet_Subnet_LSB;
	g_config.Ethernet_Gateway_MSB      = Modbus_Registers.Ethernet_Gateway_MSB;
	g_config.Ethernet_Gateway_LSB      = Modbus_Registers.Ethernet_Gateway_LSB;
	g_config.Ethernet_Slave_Id         = Modbus_Registers.Ethernet_Slave_Id;
	g_config.Ethernet_Port             = Modbus_Registers.Ethernet_Port;
	g_config.Ethernet_IP_MSB_PC        = Modbus_Registers.Ethernet_IP_MSB_PC;
	g_config.Ethernet_IP_LSB_PC        = Modbus_Registers.Ethernet_IP_LSB_PC;
	g_config.Ethernet_Port_PC          = Modbus_Registers.Ethernet_Port_PC;
	g_config.Overload_MSB              = Modbus_Registers.Overload_MSB;
	g_config.Overload_LSB              = Modbus_Registers.Overload_LSB;
	g_config.Overload_Unit             = Modbus_Registers.Overload_Unit;
	g_config.Overrange_MSB             = Modbus_Registers.Overrange_MSB;
	g_config.Overrange_LSB             = Modbus_Registers.Overrange_LSB;
	g_config.Overrange_Unit            = Modbus_Registers.Overrange_Unit;
	g_config.PASSWORD_MSB              = Modbus_Registers.PASSWORD_MSB;
	g_config.PASSWORD_LSB              = Modbus_Registers.PASSWORD_LSB;
	g_config.Analog_4_to_20mA_Min_MSB  = Modbus_Registers.Analog_4_to_20mA_Min_MSB;
	g_config.Analog_4_to_20mA_Min_LSB  = Modbus_Registers.Analog_4_to_20mA_Min_LSB;
	g_config.Analog_4_to_20mA_Min_Unit = Modbus_Registers.Analog_4_to_20mA_Min_Unit;
	g_config.Analog_4_to_20mA_Max_MSB  = Modbus_Registers.Analog_4_to_20mA_Max_MSB;
	g_config.Analog_4_to_20mA_Max_LSB  = Modbus_Registers.Analog_4_to_20mA_Max_LSB;
	g_config.Analog_4_to_20mA_Max_Unit = Modbus_Registers.Analog_4_to_20mA_Max_Unit;

	// 4_20 Calib Factor
	int32_t a_fac = (int32_t)(g_cal_a * 100.0f);
	g_config.CAL_4_20MA_FACTA_MSB      = (a_fac >> 16) & 0xFFFF;
	g_config.CAL_4_20MA_FACTA_LSB      = a_fac & 0xFFFF;

	int32_t b_fac = (int32_t)(g_cal_b * 100.0f);

	g_config.CAL_4_20MA_FACTB_MSB      = (b_fac >> 16) & 0xFFFF;
	g_config.CAL_4_20MA_FACTB_LSB      = b_fac & 0xFFFF;

	g_config.SD_MODE                   = Modbus_Registers.SD_MODE;
	g_config.MAGIC_NUMBER              = 0xAA55;

}

void Save_Config_To_Eeprom(void)
{
	if(EEPROM_Write_Config(&hi2c4, EEPROM_ADDR, &g_config)!=HAL_OK)
	{
		printf("I2c Write Failed\r\n");
	}

	//	HAL_Delay(10);
	//
	//	if(EEPROM_Read_Config(&hi2c4, EEPROM_ADDR, &g_config)!=HAL_OK)
	//	{
	//		printf("I2c Read Failed\r\n");
	//	}
}

void Load_Default_Config(device_config_t *cfg)
{
	memset(cfg, 0, sizeof(device_config_t));

	cfg->Primary_Unit = 0;
	cfg->AGM_MODE     = 0;
    cfg->AUDIO_MODE   = 1;
    cfg->FREQ_RECV    = 0;
    cfg->FREQ_DTC     = 0;

	// Alarm Value = 10R/h
	cfg->Alarm_Value_MSB = 0;
	cfg->Alarm_Value_LSB = 1000 & 0xFFFF;
	cfg->Alarm_Unit      = 0;

	cfg->RS485_Slave_Id  = 1;
	cfg->RS485_Baud_Rate = 2; //9600

	// Password = 123
	cfg->PASSWORD_MSB = 0;
	cfg->PASSWORD_LSB = 1111;

	// IP: 192.168.0.10
	cfg->Ethernet_IP_MSB  = (192 << 8) | 168;
	cfg->Ethernet_IP_LSB  = (0 << 8)   | 10;

	// Subnet: 255.255.255.0
	cfg->Ethernet_Subnet_MSB = (255 << 8) | 255;
	cfg->Ethernet_Subnet_LSB = (255 << 8) | 0;

	// Gateway: 0.0.0.0
	cfg->Ethernet_Gateway_MSB = 0;
	cfg->Ethernet_Gateway_LSB = 0;

	cfg->Ethernet_Slave_Id = 1;
	cfg->Ethernet_Port     = 502;

	// PC: IP->192.168.1.53 , Port:502
	cfg->Ethernet_IP_MSB_PC = (192<<8) | 168;
	cfg->Ethernet_IP_LSB_PC = (1<<8)   | 53;
	cfg->Ethernet_Port_PC   = 502;

	// Overload = 200R/h
	cfg->Overload_MSB  = 0;
	cfg->Overload_LSB  = 20000 & 0xFFFF;
	cfg->Overload_Unit = 0;

	// Overrange = 100R/h
	cfg->Overrange_MSB  = 0;
	cfg->Overrange_LSB  = 10000 & 0xFFFF;
	cfg->Overrange_Unit = 0;

	// Analog Min = 0
	cfg->Analog_4_to_20mA_Min_MSB  = 0;
	cfg->Analog_4_to_20mA_Min_LSB  = 0;
	cfg->Analog_4_to_20mA_Min_Unit = 0;

	// Analog Max = 10R/h
	cfg->Analog_4_to_20mA_Max_MSB  = 0;
	cfg->Analog_4_to_20mA_Max_LSB  = 1000;
	cfg->Analog_4_to_20mA_Max_Unit = 0;

	// Calibration factors = 1
	cfg->CAL_4_20MA_FACTA_MSB = 0;
	cfg->CAL_4_20MA_FACTA_LSB = 1;
	cfg->CAL_4_20MA_FACTB_MSB = 0;
	cfg->CAL_4_20MA_FACTB_LSB = 1;
}

void Unstick_I2C_Bus(void) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //DeInit I2C peripheral BEFORE reset
    HAL_I2C_DeInit(&hi2c4);

    //Clock reset (optional if DeInit is enough)
    __HAL_RCC_I2C4_FORCE_RESET();
    HAL_Delay(2);
    __HAL_RCC_I2C4_RELEASE_RESET();

    //Reconfigure SDA/SCL as GPIO open-drain
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    //Clock out 9 bits on SCL while toggling SDA
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // SCL high
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // SDA low
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // SCL low
        HAL_Delay(1);
    }

    //Send STOP condition
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // SDA high
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // SCL high
    HAL_Delay(2);

    //Reconfigure SCL/SDA as I2C
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    //Reinitialize I2C peripheral
    HAL_I2C_Init(&hi2c4);

}


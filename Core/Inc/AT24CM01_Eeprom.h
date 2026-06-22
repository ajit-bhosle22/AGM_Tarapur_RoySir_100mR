#ifndef __AT24CM01_EEPROM_H__
#define __AT24CM01_EEPROM_H__

#include "stm32h7xx_hal.h"

// 7-bit addr shifted for HAL
#define EEPROM_I2C_ADDR   (0x50<<1)
#define EEPROM_PAGE_SIZE  256
#define EEPROM_ADDR       0x0000

typedef struct
{
	uint16_t AUDIO_MODE;                      //Addr0
	uint16_t Primary_Unit;                    //Addr1
	uint16_t AGM_MODE;                        //Addr2
	uint16_t Alarm_Value_MSB;                 //Addr3
	uint16_t Alarm_Value_LSB;                 //Addr4
	uint16_t Alarm_Unit;                      //Addr5
	uint16_t RS485_Slave_Id;                  //Addr6
	uint16_t RS485_Baud_Rate;                 //Addr7
	uint16_t PASSWORD_MSB;                    //Addr8
	uint16_t PASSWORD_LSB;                    //Addr9
	uint16_t Ethernet_IP_MSB;                 //Addr10
	uint16_t Ethernet_IP_LSB;                 //Addr11
	uint16_t Ethernet_Subnet_MSB;             //Addr12
	uint16_t Ethernet_Subnet_LSB;             //Addr13
	uint16_t Ethernet_Gateway_MSB;            //Addr14
	uint16_t Ethernet_Gateway_LSB;            //Addr15
	uint16_t Ethernet_Slave_Id;               //Addr16
	uint16_t Ethernet_Port;                   //Addr17
	uint16_t Overload_MSB;                    //Addr18
	uint16_t Overload_LSB;                    //Addr19
	uint16_t Overload_Unit;                   //Addr20
	uint16_t Overrange_MSB;                   //Addr21
	uint16_t Overrange_LSB;                   //Addr22
	uint16_t Overrange_Unit;                  //Addr23
	uint16_t Analog_4_to_20mA_Min_MSB;        //Addr24
	uint16_t Analog_4_to_20mA_Min_LSB;        //Addr25
	uint16_t Analog_4_to_20mA_Min_Unit;       //Addr26
	uint16_t Analog_4_to_20mA_Max_MSB;        //Addr27
	uint16_t Analog_4_to_20mA_Max_LSB;        //Addr28
	uint16_t Analog_4_to_20mA_Max_Unit;       //Addr29
	uint16_t CAL_4_20MA_FACTA_MSB;            //Addr30
	uint16_t CAL_4_20MA_FACTA_LSB;            //Addr31
	uint16_t CAL_4_20MA_FACTB_MSB;            //Addr32
	uint16_t CAL_4_20MA_FACTB_LSB;            //Addr33
	uint16_t FREQ_RECV;                       //Addr34
	uint16_t FREQ_DTC;                        //Addr35
	uint16_t MAGIC_NUMBER;                    //Addr36
} __attribute__((packed)) device_config_t;


extern device_config_t g_config;

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t memAddr, uint8_t data);
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t memAddr, uint8_t *data);
HAL_StatusTypeDef EEPROM_WritePage(uint16_t memAddr, uint8_t *data, uint16_t len);
HAL_StatusTypeDef EEPROM_Write_Config(I2C_HandleTypeDef *hi2c, uint16_t addr, device_config_t *cfg);
HAL_StatusTypeDef EEPROM_Read_Config(I2C_HandleTypeDef *hi2c, uint16_t addr, device_config_t *cfg);
HAL_StatusTypeDef WaitForEEPROM(I2C_HandleTypeDef *hi2c);

void Load_Default_Config(device_config_t *cfg);
void Save_Config_To_Eeprom(void);
void Unstick_I2C_Bus(void);
void cpy_reg_to_eeprom_reg(void);
//void config_modbus_registers(void);

#endif

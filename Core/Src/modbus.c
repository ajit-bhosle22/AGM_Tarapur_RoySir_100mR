
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "main_global.h"
#include "modbus.h"
#include "lcd.h"
#include "AT24CM01_Eeprom.h"
#include "Detector_Modbus_registers.h"
#include "Fault_Handler.h"
#include "pc_modbus.h"
#include "tcp_server_registers.h"
#include "DAC_Drvr.h"
#include "fatfs_sd.h"

#define MR_25_VAL             25.0f
#define MR_50_VAL             50.0f
#define MR_75_VAL             75.0f
#define MR_100_VAL            100.0f

#define DAC_4MA_CODE          738
#define DAC_20MA_CODE         3608

#define MODBUS_HV_ADDRESS     43
#define MODBUS_CALIB_ADDRESS  49

Write_ModbusState_t pc_state =  {0};
Write_ModbusState_t tcp_state = {0};

master_modbus_db_t Modbus_Registers = {0};
master_modbus_db_t Modbus_Registers_Write = {0};
master_modbus_db_t Modbus_Registers_PC_TCP = {0};
master_modbus_db_t Modbus_Registers_PC_TCP_Write = {0};

uint32_t cpm=0;
uint8_t  ip[4];
uint8_t  sn[4];
uint8_t  gw[4];

volatile uint16_t tcp_slave_id            =  1;
volatile uint16_t tcp_current_port        =  510;
bool tcp_reconfig_required = false;
bool rs485_reconfig_required = false;
double dose_mRh;
double e4mA_val  = 0.0f;
double e20mA_val = 0.0f;
char e4mA_buf[16];
char e20mA_buf[16];
uint32_t scaled_mr;

char Factor1_Value[12]    = "1";
char Factor2_Value[12]    = "1";
char Factor3_Value[12]    = "1";
char Factor4_Value[12]    = "1";

bool read_sd_flag         = false;

static void modbus_val_to_str(uint16_t msb, uint16_t lsb,
                               uint8_t unit, char* buf, size_t buf_size);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2){

		rx_index_dtc = Size;
		frame_recv_dtc = 1;
	}

	if (huart->Instance == UART8)
	{
		rx_index_PC = Size;
		frame_recv_PC = 1;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		RS485_RX_MODE_DTC();
		Modbus_Restart_RX_DMA_DTC();
	}

	if(huart->Instance == UART8)
	{
		RS485_RX_MODE_PC();
		Modbus_Restart_RX_DMA_PC();
	}
}

void update_cps_hv_mr(void)
{
	CPS_VAL = (Modbus_Registers_Detector.CPS_MSB<<16)|(Modbus_Registers_Detector.CPS_LSB);

	Hv_Val = (Modbus_Registers_Detector.HV_MSB << 16)| (Modbus_Registers_Detector.HV_LSB);
	Hv_Voltage = (float) (Hv_Val/100.0f);

	dose_mRh = convert_cps_to_mr_h();
	scaled_mr = (uint32_t)(dose_mRh * 1000.0f);

	Modbus_Registers.mR_MSB = (scaled_mr >>16) & 0xFFFF;
	Modbus_Registers.mR_LSB = scaled_mr & 0xFFFF;

	Modbus_Registers_PC_TCP.mR_MSB = (scaled_mr >>16) & 0xFFFF;
	Modbus_Registers_PC_TCP.mR_LSB = scaled_mr & 0xFFFF;
}


void initialize_modbus_registers()
{
	Modbus_Registers.CPS_MSB = Modbus_Registers_Detector.CPS_MSB;
	Modbus_Registers.CPS_LSB = Modbus_Registers_Detector.CPS_LSB;

	Modbus_Registers_PC_TCP.CPS_MSB = Modbus_Registers_Detector.CPS_MSB;
	Modbus_Registers_PC_TCP.CPS_LSB = Modbus_Registers_Detector.CPS_LSB;

	Modbus_Registers.HV_MSB = Modbus_Registers_Detector.HV_MSB;
	Modbus_Registers.HV_LSB = Modbus_Registers_Detector.HV_LSB;

	Modbus_Registers_PC_TCP.HV_MSB = Modbus_Registers_Detector.HV_MSB;
	Modbus_Registers_PC_TCP.HV_LSB = Modbus_Registers_Detector.HV_LSB;

	Modbus_Registers.CALIB_FACTOR1  = Modbus_Registers_Detector.CALIB_FACTOR1;
	Modbus_Registers.CALIB_FACTOR2  = Modbus_Registers_Detector.CALIB_FACTOR2;
	Modbus_Registers.CALIB_FACTOR3  = Modbus_Registers_Detector.CALIB_FACTOR3;
	Modbus_Registers.CALIB_FACTOR4  = Modbus_Registers_Detector.CALIB_FACTOR4;

	Modbus_Registers_PC_TCP.CALIB_FACTOR1  = Modbus_Registers_Detector.CALIB_FACTOR1;
	Modbus_Registers_PC_TCP.CALIB_FACTOR2  = Modbus_Registers_Detector.CALIB_FACTOR2;
	Modbus_Registers_PC_TCP.CALIB_FACTOR3  = Modbus_Registers_Detector.CALIB_FACTOR3;
	Modbus_Registers_PC_TCP.CALIB_FACTOR4  = Modbus_Registers_Detector.CALIB_FACTOR4;

}

void config_CalibFactors_Variables(void)
{
	uint32_t k = Modbus_Registers.CALIB_FACTOR1;
	uint32_t int_part  = k / 100;
	uint32_t frac_part = k % 100;
	if (frac_part == 0) {
		sprintf((char *)Factor1_Value, "%lu", int_part);
	} else if (frac_part % 10 == 0) {
		sprintf((char *)Factor1_Value, "%lu.%01lu", int_part, frac_part / 10);
	} else {
		sprintf((char *)Factor1_Value, "%lu.%02lu", int_part, frac_part);
	}

	k = Modbus_Registers.CALIB_FACTOR2;
	int_part  = k / 100;
	frac_part = k % 100;
	if (frac_part == 0) {
		sprintf((char *)Factor2_Value, "%lu", int_part);
	} else if (frac_part % 10 == 0) {
		sprintf((char *)Factor2_Value, "%lu.%01lu", int_part, frac_part / 10);
	} else {
		sprintf((char *)Factor2_Value, "%lu.%02lu", int_part, frac_part);
	}

	k = Modbus_Registers.CALIB_FACTOR3;
	int_part  = k / 100;
	frac_part = k % 100;
	if (frac_part == 0) {
		sprintf((char *)Factor3_Value, "%lu", int_part);
	} else if (frac_part % 10 == 0) {
		sprintf((char *)Factor3_Value, "%lu.%01lu", int_part, frac_part / 10);
	} else {
		sprintf((char *)Factor3_Value, "%lu.%02lu", int_part, frac_part);
	}

	k =  Modbus_Registers.CALIB_FACTOR4;
	int_part  = k / 100;
	frac_part = k % 100;
	if (frac_part == 0) {
		sprintf((char *)Factor4_Value, "%lu", int_part);
	} else if (frac_part % 10 == 0) {
		sprintf((char *)Factor4_Value, "%lu.%01lu", int_part, frac_part / 10);
	} else {
		sprintf((char *)Factor4_Value, "%lu.%02lu", int_part, frac_part);
	}
}

double convert_cps_to_mr_h()
{
	double dose_mr,factor;
	dose_mr = (float) CPS_VAL / GM_TUBE_SENSITIVITY;
	if (dose_mr <= (MR_25_VAL * atof(Factor1_Value)))
	{
		dose_mr = dose_mr / atof(Factor1_Value);
	}
	else if (dose_mr <= (MR_50_VAL * atof(Factor2_Value)))
	{
		dose_mr = dose_mr / atof(Factor2_Value);
	}
	else if (dose_mr <= (MR_75_VAL * atof(Factor3_Value)))
	{
		double raw_50  = (MR_50_VAL * atof(Factor2_Value));
		double raw_100 = (MR_100_VAL * atof(Factor3_Value));

		double f1 = 1/atof(Factor2_Value);
		double f2 = 1/atof(Factor3_Value);
		double r1 = (double)dose_mr - (double)raw_50;
		double sub = (f2-f1);
		double r2 = (raw_100 - raw_50);
		factor = (f1+((r1*sub)/r2));

		dose_mr = dose_mr * factor;
	}
	else
	{
		dose_mr = dose_mr / atof(Factor4_Value);
	}
	return dose_mr;
}

// -----------------------Call From LCD Save-------------------
void Update_Registers(void)
{
	Modbus_Registers.AUDIO_MODE = audio_mode;
	Modbus_Registers_PC_TCP.AUDIO_MODE = audio_mode;

	Modbus_Registers.FREQ_RECV = freq_recv_mode;
	Modbus_Registers_PC_TCP.FREQ_RECV = freq_recv_mode;

	Modbus_Registers.FREQ_DTC = freq_dtc_mode;
	Modbus_Registers_PC_TCP.FREQ_DTC = freq_dtc_mode;

	Modbus_Registers.Primary_Unit = primary_unit;
	Modbus_Registers_PC_TCP.Primary_Unit = primary_unit;

	uint16_t k = atoi(saved_rs485_slave_id);
	Modbus_Registers.RS485_Slave_Id = k;
	Modbus_Registers.RS485_Baud_Rate = baud_rate;
	Modbus_Registers_PC_TCP.RS485_Slave_Id = k;
	Modbus_Registers_PC_TCP.RS485_Baud_Rate = baud_rate;

	Modbus_Registers.Alarm_Unit = alarm_unit;
	Modbus_Registers_PC_TCP.Alarm_Unit = alarm_unit;
	double z = atof(alarm_val);
	uint32_t x;
	z = z*100;
	x = (uint32_t)(z + 0.5);

	Modbus_Registers.Alarm_Value_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers.Alarm_Value_LSB = (x & 0xFFFF);
	Modbus_Registers_PC_TCP.Alarm_Value_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers_PC_TCP.Alarm_Value_LSB = (x & 0xFFFF);

	Modbus_Registers.Overrange_Unit = overrange_unit;
	Modbus_Registers_PC_TCP.Overrange_Unit = overrange_unit;
	z = atof(overrange_val);
	z = z*100;
	x = (uint32_t)(z + 0.5);
	Modbus_Registers.Overrange_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers.Overrange_LSB = (x & 0xFFFF);
	Modbus_Registers_PC_TCP.Overrange_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers_PC_TCP.Overrange_LSB = (x & 0xFFFF);

	Modbus_Registers.Overload_Unit = overload_unit;
	Modbus_Registers_PC_TCP.Overload_Unit = overload_unit;
	z = atof(overload_val);
	z = z *100;
	x = (uint32_t)(z + 0.5);
	Modbus_Registers.Overload_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers.Overload_LSB = (x & 0xFFFF);
	Modbus_Registers_PC_TCP.Overload_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers_PC_TCP.Overload_LSB = (x & 0xFFFF);

	Modbus_Registers.Analog_4_to_20mA_Min_Unit = min_4_20_unit;
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_Unit = min_4_20_unit;
	z = atof(min_4_20_val);
	z = z*100;
	x = (uint32_t)(z + 0.5);
	Modbus_Registers.Analog_4_to_20mA_Min_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers.Analog_4_to_20mA_Min_LSB =  (x & 0xFFFF);
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_LSB = (x & 0xFFFF);

	Modbus_Registers.Analog_4_to_20mA_Max_Unit = max_4_20_unit;
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_Unit = max_4_20_unit;
	z = atof(max_4_20_val);
	z = z*100;
	x = (uint32_t)(z + 0.5);
	Modbus_Registers.Analog_4_to_20mA_Max_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers.Analog_4_to_20mA_Max_LSB = (x & 0xFFFF);
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_MSB = (x & 0xFFFF);

	Modbus_Registers.AGM_MODE = agm_mode;
	Modbus_Registers_PC_TCP.AGM_MODE = agm_mode;

	k = atoi(saved_eth_slave_id);
	Modbus_Registers.Ethernet_Slave_Id = k;
	Modbus_Registers_PC_TCP.Ethernet_Slave_Id = k;
	k= atoi(saved_eth_port);
	Modbus_Registers.Ethernet_Port = k;
	Modbus_Registers_PC_TCP.Ethernet_Port = k;

	uint16_t octet1, octet2, octet3, octet4;

	sscanf((char*)saved_eth_ip, "%hu.%hu.%hu.%hu",
			&octet1, &octet2, &octet3, &octet4);

	Modbus_Registers.Ethernet_IP_MSB = (octet1 << 8) | (octet2 & 0xFF);
	Modbus_Registers.Ethernet_IP_LSB = (octet3 << 8) | (octet4 & 0xFF);
	Modbus_Registers_PC_TCP.Ethernet_IP_MSB = (octet1 << 8) | (octet2 & 0xFF);
	Modbus_Registers_PC_TCP.Ethernet_IP_LSB = (octet3 << 8) | (octet4 & 0xFF);

	sscanf((char*)saved_eth_subnet, "%hu.%hu.%hu.%hu",
			&octet1, &octet2, &octet3, &octet4);

	Modbus_Registers.Ethernet_Subnet_MSB = (octet1 << 8) | (octet2 & 0xFF);
	Modbus_Registers.Ethernet_Subnet_LSB = (octet3 << 8) | (octet4 & 0xFF);
	Modbus_Registers_PC_TCP.Ethernet_Subnet_MSB = (octet1 << 8) | (octet2 & 0xFF);
	Modbus_Registers_PC_TCP.Ethernet_Subnet_LSB = (octet3 << 8) | (octet4 & 0xFF);

	sscanf((char*)saved_eth_gateway, "%hu.%hu.%hu.%hu",
			&octet1, &octet2, &octet3, &octet4);

	Modbus_Registers.Ethernet_Gateway_MSB = (octet1 << 8) | (octet2 & 0xFF);
	Modbus_Registers.Ethernet_Gateway_LSB = (octet3 << 8) | (octet4 & 0xFF);
	Modbus_Registers_PC_TCP.Ethernet_Gateway_MSB = (octet1 << 8) | (octet2 & 0xFF);
	Modbus_Registers_PC_TCP.Ethernet_Gateway_LSB = (octet3 << 8) | (octet4 & 0xFF);

	x = (uint32_t)atoi((const char *)correct_password);
	Modbus_Registers.PASSWORD_MSB = ( x >> 16 ) & 0xFFFF;
	Modbus_Registers.PASSWORD_LSB = ( x & 0xFFFF );
	Modbus_Registers_PC_TCP.PASSWORD_MSB = ( x >> 16 ) & 0xFFFF;
	Modbus_Registers_PC_TCP.PASSWORD_LSB = ( x & 0xFFFF );
}

// -------------------------Booting Time--------------------------
void config_modbus_registers(void)
{
	/*
	 * Modbus RTU
	*/
	Modbus_Registers.AUDIO_MODE                = g_config.AUDIO_MODE;
	Modbus_Registers.AGM_MODE                  = g_config.AGM_MODE;
	Modbus_Registers.Primary_Unit              = g_config.Primary_Unit;
	Modbus_Registers.Alarm_Value_MSB           = g_config.Alarm_Value_MSB;
	Modbus_Registers.Alarm_Value_LSB           = g_config.Alarm_Value_LSB;
	Modbus_Registers.Alarm_Unit                = g_config.Alarm_Unit;
	Modbus_Registers.RS485_Baud_Rate           = g_config.RS485_Baud_Rate;
	Modbus_Registers.RS485_Slave_Id            = g_config.RS485_Slave_Id;
	Modbus_Registers.Ethernet_IP_MSB           = g_config.Ethernet_IP_MSB;
	Modbus_Registers.Ethernet_IP_LSB           = g_config.Ethernet_IP_LSB;
	Modbus_Registers.Ethernet_Subnet_MSB       = g_config.Ethernet_Subnet_MSB;
	Modbus_Registers.Ethernet_Subnet_LSB       = g_config.Ethernet_Subnet_LSB;
	Modbus_Registers.Ethernet_Gateway_MSB      = g_config.Ethernet_Gateway_MSB;
	Modbus_Registers.Ethernet_Gateway_LSB      = g_config.Ethernet_Gateway_LSB;
	Modbus_Registers.Ethernet_Slave_Id         = g_config.Ethernet_Slave_Id;
	Modbus_Registers.Ethernet_Port             = g_config.Ethernet_Port;
	Modbus_Registers.Overload_MSB              = g_config.Overload_MSB;
	Modbus_Registers.Overload_LSB              = g_config.Overload_LSB;
	Modbus_Registers.Overload_Unit             = g_config.Overload_Unit;
	Modbus_Registers.Overrange_MSB             = g_config.Overrange_MSB;
	Modbus_Registers.Overrange_LSB             = g_config.Overrange_LSB;
	Modbus_Registers.Overrange_Unit            = g_config.Overrange_Unit;
	Modbus_Registers.PASSWORD_MSB              = g_config.PASSWORD_MSB;
	Modbus_Registers.PASSWORD_LSB              = g_config.PASSWORD_LSB;
	Modbus_Registers.Analog_4_to_20mA_Min_MSB  = g_config.Analog_4_to_20mA_Min_MSB;
	Modbus_Registers.Analog_4_to_20mA_Min_LSB  = g_config.Analog_4_to_20mA_Min_LSB;
	Modbus_Registers.Analog_4_to_20mA_Min_Unit = g_config.Analog_4_to_20mA_Min_Unit;
	Modbus_Registers.Analog_4_to_20mA_Max_MSB  = g_config.Analog_4_to_20mA_Max_MSB;
	Modbus_Registers.Analog_4_to_20mA_Max_LSB  = g_config.Analog_4_to_20mA_Max_LSB;
	Modbus_Registers.Analog_4_to_20mA_Max_Unit = g_config.Analog_4_to_20mA_Max_Unit;
    Modbus_Registers.FREQ_RECV                 = g_config.FREQ_RECV;
    Modbus_Registers.FREQ_DTC                  = g_config.FREQ_DTC;

	/*
	 * Modbus TCP
	 *
	*/
	Modbus_Registers_PC_TCP.AUDIO_MODE           = g_config.AUDIO_MODE;
	Modbus_Registers_PC_TCP.AGM_MODE             = g_config.AGM_MODE;
	Modbus_Registers_PC_TCP.Primary_Unit         = g_config.Primary_Unit;
	Modbus_Registers_PC_TCP.Alarm_Value_MSB      = g_config.Alarm_Value_MSB;
	Modbus_Registers_PC_TCP.Alarm_Value_LSB      = g_config.Alarm_Value_LSB;
	Modbus_Registers_PC_TCP.Alarm_Unit           = g_config.Alarm_Unit;
	Modbus_Registers_PC_TCP.RS485_Baud_Rate      = g_config.RS485_Baud_Rate;
	Modbus_Registers_PC_TCP.RS485_Slave_Id       = g_config.RS485_Slave_Id;
	Modbus_Registers_PC_TCP.Ethernet_IP_MSB      = g_config.Ethernet_IP_MSB;
	Modbus_Registers_PC_TCP.Ethernet_IP_LSB      = g_config.Ethernet_IP_LSB;
	Modbus_Registers_PC_TCP.Ethernet_Subnet_MSB  = g_config.Ethernet_Subnet_MSB;
	Modbus_Registers_PC_TCP.Ethernet_Subnet_LSB  = g_config.Ethernet_Subnet_LSB;
	Modbus_Registers_PC_TCP.Ethernet_Gateway_MSB = g_config.Ethernet_Gateway_MSB;
	Modbus_Registers_PC_TCP.Ethernet_Gateway_LSB = g_config.Ethernet_Gateway_LSB;
	Modbus_Registers_PC_TCP.Ethernet_Slave_Id    = g_config.Ethernet_Slave_Id;
	Modbus_Registers_PC_TCP.Ethernet_Port        = g_config.Ethernet_Port;
	Modbus_Registers_PC_TCP.Overload_MSB         = g_config.Overload_MSB;
	Modbus_Registers_PC_TCP.Overload_LSB         = g_config.Overload_LSB;
	Modbus_Registers_PC_TCP.Overload_Unit        = g_config.Overload_Unit;
	Modbus_Registers_PC_TCP.Overrange_MSB        = g_config.Overrange_MSB;
	Modbus_Registers_PC_TCP.Overrange_LSB        = g_config.Overrange_LSB;
	Modbus_Registers_PC_TCP.Overrange_Unit       = g_config.Overrange_Unit;
	Modbus_Registers_PC_TCP.PASSWORD_MSB         = g_config.PASSWORD_MSB;
	Modbus_Registers_PC_TCP.PASSWORD_LSB         = g_config.PASSWORD_LSB;
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_MSB  = g_config.Analog_4_to_20mA_Min_MSB;
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_LSB  = g_config.Analog_4_to_20mA_Min_LSB;
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_Unit = g_config.Analog_4_to_20mA_Min_Unit;
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_MSB  = g_config.Analog_4_to_20mA_Max_MSB;
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_LSB  = g_config.Analog_4_to_20mA_Max_LSB;
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_Unit = g_config.Analog_4_to_20mA_Max_Unit;
    Modbus_Registers_PC_TCP.FREQ_RECV                 = g_config.FREQ_RECV;
    Modbus_Registers_PC_TCP.FREQ_DTC                  = g_config.FREQ_DTC;
}

void config_variables(void)
{

	if(Modbus_Registers.AUDIO_MODE == 0)
	{
		audio_mode = 0;   //Audio Disbaled
		strcpy(temp_audio_mode_str,"1. Disable");
	}
	else
	{
		audio_mode = 1;   //Audio Enabled
		strcpy(temp_audio_mode_str,"2. Enable");
	}

	primary_unit = Modbus_Registers.Primary_Unit;

	if(Modbus_Registers.AGM_MODE == 0){
		agm_mode = 0;
		strcpy(temp_agm_mode_str,"1. Manual");
	}
	else{
		agm_mode = 1;
		strcpy(temp_agm_mode_str,"2. Auto");
	}

	if(Modbus_Registers.FREQ_RECV ==  0)
	{
		freq_recv_mode = 0;
		strcpy(temp_freq_recv_str,"1. Disable");
	}
	else
	{
		freq_recv_mode = 1;
		strcpy(temp_freq_recv_str,"2. Enable");
	}

	if(Modbus_Registers.FREQ_DTC == 0)
	{
		freq_dtc_mode = 0;
		strcpy(temp_freq_dtc_str,"1. Disable");
	}
	else
	{
		freq_dtc_mode = 1;
		strcpy(temp_freq_dtc_str,"2. Enable");
	}

	alarm_unit     = Modbus_Registers.Alarm_Unit;
	overload_unit  = Modbus_Registers.Overload_Unit;
	overrange_unit = Modbus_Registers.Overrange_Unit;
	min_4_20_unit  = Modbus_Registers.Analog_4_to_20mA_Min_Unit;
	max_4_20_unit  = Modbus_Registers.Analog_4_to_20mA_Max_Unit;

	modbus_val_to_str(Modbus_Registers.Alarm_Value_MSB,
	                  Modbus_Registers.Alarm_Value_LSB,
	                  alarm_unit,    alarm_val,    sizeof(alarm_val));

	modbus_val_to_str(Modbus_Registers.Overload_MSB,
	                  Modbus_Registers.Overload_LSB,
	                  overload_unit, overload_val, sizeof(overload_val));

	modbus_val_to_str(Modbus_Registers.Overrange_MSB,
	                  Modbus_Registers.Overrange_LSB,
	                  overrange_unit, overrange_val, sizeof(overrange_val));

	modbus_val_to_str(Modbus_Registers.Analog_4_to_20mA_Min_MSB,
	                  Modbus_Registers.Analog_4_to_20mA_Min_LSB,
	                  min_4_20_unit, min_4_20_val, sizeof(min_4_20_val));

	modbus_val_to_str(Modbus_Registers.Analog_4_to_20mA_Max_MSB,
	                  Modbus_Registers.Analog_4_to_20mA_Max_LSB,
	                  max_4_20_unit, max_4_20_val, sizeof(max_4_20_val));

	baud_rate = Modbus_Registers.RS485_Baud_Rate;
	snprintf((char*)saved_rs485_slave_id, sizeof(saved_rs485_slave_id),
			"%03u", Modbus_Registers.RS485_Slave_Id);

	/*
	 * Update RS485 Parameters
     */
	Modbus_Slave_Id_PC = Modbus_Registers.RS485_Slave_Id;
	pending_new_baud   = GetBaudRate(baud_rate);

	uint32_t k = ((uint32_t)Modbus_Registers.PASSWORD_MSB << 16) | Modbus_Registers.PASSWORD_LSB;
	itoa(k,correct_password, 10);

	/*
	 * Ethernet Settings
	 */
	tcp_slave_id     = Modbus_Registers.Ethernet_Slave_Id;
	tcp_current_port = Modbus_Registers.Ethernet_Port;

	ip[0] = (Modbus_Registers.Ethernet_IP_MSB >> 8) & 0xFF;
	ip[1] =  Modbus_Registers.Ethernet_IP_MSB       & 0xFF;
	ip[2] = (Modbus_Registers.Ethernet_IP_LSB >> 8) & 0xFF;
	ip[3] =  Modbus_Registers.Ethernet_IP_LSB       & 0xFF;

	sn[0] = (Modbus_Registers.Ethernet_Subnet_MSB >> 8) & 0xFF;
	sn[1] =  Modbus_Registers.Ethernet_Subnet_MSB       & 0xFF;
	sn[2] = (Modbus_Registers.Ethernet_Subnet_LSB >> 8) & 0xFF;
	sn[3] =  Modbus_Registers.Ethernet_Subnet_LSB       & 0xFF;

	gw[0] = (Modbus_Registers.Ethernet_Gateway_MSB >> 8) & 0xFF;
	gw[1] =  Modbus_Registers.Ethernet_Gateway_MSB       & 0xFF;
	gw[2] = (Modbus_Registers.Ethernet_Gateway_LSB >> 8) & 0xFF;
	gw[3] =  Modbus_Registers.Ethernet_Gateway_LSB       & 0xFF;

	snprintf((char*)saved_eth_ip, sizeof(saved_eth_ip),
			"%03u.%03u.%03u.%03u",
			(Modbus_Registers.Ethernet_IP_MSB >> 8) & 0xFF,
			Modbus_Registers.Ethernet_IP_MSB       & 0xFF,
			(Modbus_Registers.Ethernet_IP_LSB >> 8) & 0xFF,
			Modbus_Registers.Ethernet_IP_LSB       & 0xFF);

	snprintf((char*)saved_eth_subnet, sizeof(saved_eth_subnet),
			"%03u.%03u.%03u.%03u",
			(Modbus_Registers.Ethernet_Subnet_MSB >> 8) & 0xFF,
			Modbus_Registers.Ethernet_Subnet_MSB       & 0xFF,
			(Modbus_Registers.Ethernet_Subnet_LSB >> 8) & 0xFF,
			Modbus_Registers.Ethernet_Subnet_LSB       & 0xFF);

	snprintf((char*)saved_eth_gateway, sizeof(saved_eth_gateway),
			"%03u.%03u.%03u.%03u",
			(Modbus_Registers.Ethernet_Gateway_MSB >> 8) & 0xFF,
			Modbus_Registers.Ethernet_Gateway_MSB       & 0xFF,
			(Modbus_Registers.Ethernet_Gateway_LSB >> 8) & 0xFF,
			Modbus_Registers.Ethernet_Gateway_LSB       & 0xFF);

	snprintf((char*)saved_eth_slave_id, sizeof(saved_eth_slave_id),
			"%03u", Modbus_Registers.Ethernet_Slave_Id);

	snprintf((char*)saved_eth_port, sizeof(saved_eth_port),
			"%04u", Modbus_Registers.Ethernet_Port);

	int32_t x = ((int32_t)g_config.CAL_4_20MA_FACTA_MSB << 16) |
			g_config.CAL_4_20MA_FACTA_LSB;

	int32_t y = ((int32_t)g_config.CAL_4_20MA_FACTB_MSB << 16) |
			g_config.CAL_4_20MA_FACTB_LSB;

	g_cal_a = (double)x/100.0f;
	g_cal_b = (double)y/100.0f;


}

static void modbus_val_to_str(uint16_t msb, uint16_t lsb,
                               uint8_t unit, char* buf, size_t buf_size)
{
    uint32_t k          = ((uint32_t)msb << 16) | lsb;
    float    z          = ((float)k / 100.0f);
    z                   = ((int)(z * 1000.0f + 0.5f)) / 1000.0f;
    uint32_t factor_x100 = (uint32_t)(z * 100.0f + 0.5f);

    if (unit_is_integer(unit))
    {
        uint32_t int_val = factor_x100 / 100;
        if (int_val > 99999U) int_val = 99999U;
        snprintf(buf, buf_size, "%05lu  ", (unsigned long)int_val);
    }
    else
    {
        if ((factor_x100 / 100) > 9999U) factor_x100 = 999999U;
        snprintf(buf, buf_size, "%04lu.%02lu",
                 (unsigned long)(factor_x100 / 100),
                 (unsigned long)(factor_x100 % 100));
    }
}

void conf_rs485_tcp(void)
{
	if(tcp_reconfig_required == true){
		tcp_reconfig_required = false;
		Reconfigure_TCP();
	}

	if(rs485_reconfig_required == true)
	{
		rs485_reconfig_required = false;
		Reconfigure_RS485();
	}
}

void e4_20mA_calib(void)
{
	read_e4_20ma_CalibVal();
	if(e4mA_val > 0.0f && e20mA_val > 0.0f){
		Two_Point_Calibrate_DAC(e4mA_val,e20mA_val);
		e4mA_val  = 0.0f;
		e20mA_val = 0.0f;
	}
}

void cpy_4_20mA_factors_eeprom_reg()
{
	int32_t a_fac = (int32_t)(g_cal_a * 100.0f);

	g_config.CAL_4_20MA_FACTA_MSB = (a_fac >> 16) & 0xFFFF;
	g_config.CAL_4_20MA_FACTA_LSB = a_fac & 0xFFFF;

	int32_t b_fac = (int32_t)(g_cal_b * 100.0f);

	g_config.CAL_4_20MA_FACTB_MSB = (b_fac >> 16) & 0xFFFF;
	g_config.CAL_4_20MA_FACTB_LSB = b_fac & 0xFFFF;
}

void is_reconfigure_rtc_rs485(void)
{
	bool date_changed = false;
	bool time_changed = false;

	if(Modbus_Registers.RTC_DAY != Modbus_Registers_Write.RTC_DAY || Modbus_Registers.RTC_MONTH != Modbus_Registers_Write.RTC_MONTH
			|| Modbus_Registers.RTC_YEAR != Modbus_Registers_Write.RTC_YEAR)
	{
		date_changed = true;
	}

	if(Modbus_Registers.RTC_HOUR != Modbus_Registers_Write.RTC_HOUR || Modbus_Registers.RTC_MIN != Modbus_Registers_Write.RTC_MIN
			|| Modbus_Registers.RTC_SEC != Modbus_Registers_Write.RTC_SEC)
	{
		time_changed = true;
	}

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	uint16_t year;

	if(date_changed)
	{
		sDate.Date    = Modbus_Registers_Write.RTC_DAY;
		sDate.Month   = Modbus_Registers_Write.RTC_MONTH;
		year          = Modbus_Registers_Write.RTC_YEAR;
		sDate.Year    = year - 2000;
		sDate.WeekDay = Get_RTC_WeekDay(
				sDate.Date,
				sDate.Month,
				year);
	}

	if(time_changed)
	{
		sTime.Hours   = Modbus_Registers_Write.RTC_HOUR;
		sTime.Minutes = Modbus_Registers_Write.RTC_MIN;
		sTime.Seconds = Modbus_Registers_Write.RTC_SEC;
	}

	if(date_changed && Validate_Date_RS485_Registers(sDate.Date,sDate.Month,year))
	{
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	}

	if(time_changed && Validate_Time_RS485_Registers(sTime.Hours,sTime.Minutes,sTime.Seconds))
	{
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	}
}

void is_reconfigure_rtc_tcp(void)
{
	bool date_changed = false;
	bool time_changed = false;

	if(Modbus_Registers_PC_TCP.RTC_DAY != Modbus_Registers_PC_TCP_Write.RTC_DAY || Modbus_Registers_PC_TCP.RTC_MONTH != Modbus_Registers_PC_TCP_Write.RTC_MONTH
	|| Modbus_Registers_PC_TCP.RTC_YEAR != Modbus_Registers_PC_TCP_Write.RTC_YEAR)
	{
		date_changed = true;
	}

	if(Modbus_Registers_PC_TCP.RTC_HOUR != Modbus_Registers_PC_TCP_Write.RTC_HOUR || Modbus_Registers_PC_TCP.RTC_MIN != Modbus_Registers_PC_TCP_Write.RTC_MIN
	|| Modbus_Registers_PC_TCP.RTC_SEC != Modbus_Registers_PC_TCP_Write.RTC_SEC)
	{
		time_changed = true;
	}

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	uint16_t year;

	if(date_changed)
	{
		sDate.Date    = Modbus_Registers_PC_TCP_Write.RTC_DAY;
		sDate.Month   = Modbus_Registers_PC_TCP_Write.RTC_MONTH;
		year          = Modbus_Registers_PC_TCP_Write.RTC_YEAR;
		sDate.Year    = year - 2000;
		sDate.WeekDay = Get_RTC_WeekDay(
				sDate.Date,
				sDate.Month,
				year);
	}

	if(time_changed)
	{
		sTime.Hours   = Modbus_Registers_PC_TCP_Write.RTC_HOUR;
		sTime.Minutes = Modbus_Registers_PC_TCP_Write.RTC_MIN;
		sTime.Seconds = Modbus_Registers_PC_TCP_Write.RTC_SEC;
	}

	if(date_changed && Validate_Date_RS485_Registers(sDate.Date,sDate.Month,year))
	{
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	}

	if(time_changed && Validate_Time_RS485_Registers(sTime.Hours,sTime.Minutes,sTime.Seconds))
	{
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	}
}

void is_reconfigure_rs485_tcp(void)
{
	if(Modbus_Registers.Ethernet_IP_MSB != Modbus_Registers_Write.Ethernet_IP_MSB || Modbus_Registers.Ethernet_IP_LSB != Modbus_Registers_Write.Ethernet_IP_LSB ||
			Modbus_Registers.Ethernet_Subnet_MSB != Modbus_Registers_Write.Ethernet_Subnet_MSB || Modbus_Registers.Ethernet_Subnet_LSB != Modbus_Registers_Write.Ethernet_Subnet_LSB ||
			Modbus_Registers.Ethernet_Gateway_MSB != Modbus_Registers_Write.Ethernet_Gateway_MSB || Modbus_Registers.Ethernet_Gateway_LSB != Modbus_Registers_Write.Ethernet_Gateway_LSB ||
			Modbus_Registers.Ethernet_Slave_Id != Modbus_Registers_Write.Ethernet_Slave_Id || Modbus_Registers.Ethernet_Port != Modbus_Registers_Write.Ethernet_Port)
	{
		tcp_reconfig_required = true;
	}

	if(Modbus_Registers.RS485_Slave_Id != Modbus_Registers_Write.RS485_Slave_Id ||
			Modbus_Registers.RS485_Baud_Rate != Modbus_Registers_Write.RS485_Baud_Rate)
	{
		rs485_reconfig_required = true;
	}
}

void is_reconfigure_rs485_tcp_pc(void)
{

	if(Modbus_Registers_PC_TCP.Ethernet_IP_MSB != Modbus_Registers.Ethernet_IP_MSB || Modbus_Registers_PC_TCP.Ethernet_IP_LSB != Modbus_Registers.Ethernet_IP_LSB ||
			Modbus_Registers_PC_TCP.Ethernet_Subnet_MSB != Modbus_Registers.Ethernet_Subnet_MSB || Modbus_Registers_PC_TCP.Ethernet_Subnet_LSB != Modbus_Registers.Ethernet_Subnet_LSB ||
			Modbus_Registers_PC_TCP.Ethernet_Gateway_MSB != Modbus_Registers.Ethernet_Gateway_MSB || Modbus_Registers_PC_TCP.Ethernet_Gateway_LSB != Modbus_Registers.Ethernet_Gateway_LSB ||
			Modbus_Registers_PC_TCP.Ethernet_Slave_Id != Modbus_Registers.Ethernet_Slave_Id || Modbus_Registers_PC_TCP.Ethernet_Port != Modbus_Registers.Ethernet_Port)
	{
		tcp_reconfig_required = true;
	}

	if(Modbus_Registers.RS485_Slave_Id != Modbus_Registers_PC_TCP.RS485_Slave_Id ||
			Modbus_Registers.RS485_Baud_Rate != Modbus_Registers_PC_TCP.RS485_Baud_Rate)
	{
		rs485_reconfig_required = true;
	}

}

void is_sd_card_read_rs485(void)
{
	uint16_t status_sd = Modbus_Registers_Write.READ_SD;
	Modbus_Registers_Write.READ_SD = 0;
	if(status_sd){
		read_sd_flag = true;
	}
	else{
		read_sd_flag = false;
	}
}

void is_sd_card_read_tcp(void)
{
	uint16_t status_sd = Modbus_Registers_PC_TCP_Write.READ_SD;
	Modbus_Registers_PC_TCP_Write.READ_SD = 0;
	if(status_sd){
		read_sd_flag = true;
	}
	else{
		read_sd_flag = false;
	}
}

void is_reconfigure_hv_rs485(void)
{
	bool hv_reconfigure = false;
	bool calib_factor_reconfigure = false;
    bool dtc_freq_switch = false;

	if((Modbus_Registers_Write.HV_MSB != Modbus_Registers.HV_MSB)
			|| Modbus_Registers_Write.HV_LSB != Modbus_Registers.HV_LSB)
	{
		Modbus_Registers_Detector_Write.HV_MSB = Modbus_Registers_Write.HV_MSB;
		Modbus_Registers_Detector_Write.HV_LSB = Modbus_Registers_Write.HV_LSB;

		hv_reconfigure = true;
	}

	if((Modbus_Registers_Write.CALIB_FACTOR1 != Modbus_Registers.CALIB_FACTOR1 )
	|| (Modbus_Registers_Write.CALIB_FACTOR2 != Modbus_Registers.CALIB_FACTOR2)
	|| (Modbus_Registers_Write.CALIB_FACTOR3 != Modbus_Registers.CALIB_FACTOR3)
	|| (Modbus_Registers_Write.CALIB_FACTOR4 != Modbus_Registers.CALIB_FACTOR4)){

		Modbus_Registers_Detector_Write.CALIB_FACTOR1 = Modbus_Registers_Write.CALIB_FACTOR1;
		Modbus_Registers_Detector_Write.CALIB_FACTOR2 = Modbus_Registers_Write.CALIB_FACTOR2;
		Modbus_Registers_Detector_Write.CALIB_FACTOR3 = Modbus_Registers_Write.CALIB_FACTOR3;
		Modbus_Registers_Detector_Write.CALIB_FACTOR4 = Modbus_Registers_Write.CALIB_FACTOR4;

		calib_factor_reconfigure = true;
	}

	if(Modbus_Registers_Write.FREQ_DTC != Modbus_Registers.FREQ_RECV)
	{
		dtc_freq_switch_val = (Modbus_Registers_Write.FREQ_DTC == 1) ? 1:0;
		dtc_freq_switch = true;
	}

	if(hv_reconfigure == true && calib_factor_reconfigure == false && dtc_freq_switch == false)
	{
		modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = HV_WRITE_FLAG;

		hv_reconfigure = false;

	}
	else if(hv_reconfigure == false && calib_factor_reconfigure == true && dtc_freq_switch == false)
	{

		modbus_write_flag   = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = CALIB_WRITE_FLAG;

		calib_factor_reconfigure = false;

	}
	else if(hv_reconfigure == false && calib_factor_reconfigure == false && dtc_freq_switch == true)
	{
		modbus_write_flag = MODBUS_SINGLE_WRITE_FLAG;
		single_write_flag = DTC_FREQ_SWITCH_FLAG;

		dtc_freq_switch = false;
	}
	else if(hv_reconfigure == true && calib_factor_reconfigure == true && dtc_freq_switch == false)
	{
		//Need to Update FREQ Switch
		dtc_freq_switch_val =  freq_dtc_mode;

		modbus_write_flag   = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = HV_AND_CALIB_WRITE_FLAG;

		hv_reconfigure = false;
		calib_factor_reconfigure = false;
	}
	else if(hv_reconfigure == true && calib_factor_reconfigure == false && dtc_freq_switch == true)
	{
		modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = HV_AND_FREQ_WRITE_FLAG;

		hv_reconfigure  = false;
		dtc_freq_switch = false;
	}
	else if(hv_reconfigure == false && calib_factor_reconfigure == true && dtc_freq_switch == true)
	{
	    modbus_write_flag   = MODBUS_MULTIPLE_WRITE_FLAG;
	    multiple_write_flag = ALL_CONFIG_WRITE_FLAG;

	    calib_factor_reconfigure = true;
	    dtc_freq_switch          = true;
	}
	else if(hv_reconfigure == true && calib_factor_reconfigure == true && dtc_freq_switch == true)
	{
		modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = ALL_CONFIG_WRITE_FLAG;

		hv_reconfigure           = false;
		calib_factor_reconfigure = false;
		dtc_freq_switch          = false;
	}
}

void is_reconfigure_tcp_rs485(void)
{

	if(Modbus_Registers_PC_TCP.Ethernet_IP_MSB != Modbus_Registers_PC_TCP_Write.Ethernet_IP_MSB || Modbus_Registers_PC_TCP.Ethernet_IP_LSB != Modbus_Registers_PC_TCP_Write.Ethernet_IP_LSB ||
			Modbus_Registers_PC_TCP.Ethernet_Subnet_MSB != Modbus_Registers_PC_TCP_Write.Ethernet_Subnet_MSB || Modbus_Registers_PC_TCP.Ethernet_Subnet_LSB != Modbus_Registers_PC_TCP_Write.Ethernet_Subnet_LSB ||
			Modbus_Registers_PC_TCP.Ethernet_Gateway_MSB != Modbus_Registers_PC_TCP_Write.Ethernet_Gateway_MSB || Modbus_Registers_PC_TCP.Ethernet_Gateway_LSB != Modbus_Registers_PC_TCP_Write.Ethernet_Gateway_LSB ||
			Modbus_Registers_PC_TCP.Ethernet_Slave_Id != Modbus_Registers_PC_TCP_Write.Ethernet_Slave_Id || Modbus_Registers_PC_TCP.Ethernet_Port != Modbus_Registers_PC_TCP_Write.Ethernet_Port)
	{
		tcp_reconfig_required = true;
	}

	if(Modbus_Registers_PC_TCP_Write.RS485_Slave_Id != Modbus_Registers_PC_TCP.RS485_Slave_Id ||
			Modbus_Registers_PC_TCP_Write.RS485_Baud_Rate   != Modbus_Registers_PC_TCP.RS485_Baud_Rate)
	{
		rs485_reconfig_required = true;
	}

}

void is_reconfigure_hv_tcp(void)
{
	bool hv_reconfigure = false;
	bool calib_factor_reconfigure = false;
	bool dtc_freq_switch = false;

	if((Modbus_Registers_PC_TCP_Write.HV_MSB != Modbus_Registers_PC_TCP.HV_MSB)
			|| Modbus_Registers_PC_TCP_Write.HV_LSB != Modbus_Registers_PC_TCP.HV_LSB)
	{
		Modbus_Registers_Detector_Write.HV_MSB = Modbus_Registers_PC_TCP_Write.HV_MSB;
		Modbus_Registers_Detector_Write.HV_LSB = Modbus_Registers_PC_TCP_Write.HV_LSB;

		hv_reconfigure = true;
	}

	if((Modbus_Registers_PC_TCP_Write.CALIB_FACTOR1 != Modbus_Registers_PC_TCP.CALIB_FACTOR1 )
			|| (Modbus_Registers_PC_TCP_Write.CALIB_FACTOR2 != Modbus_Registers_PC_TCP.CALIB_FACTOR2)
			|| (Modbus_Registers_PC_TCP_Write.CALIB_FACTOR3 != Modbus_Registers_PC_TCP.CALIB_FACTOR3)
			|| (Modbus_Registers_PC_TCP_Write.CALIB_FACTOR4 != Modbus_Registers_PC_TCP.CALIB_FACTOR4)){

		Modbus_Registers_Detector_Write.CALIB_FACTOR1 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR1;
		Modbus_Registers_Detector_Write.CALIB_FACTOR2 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR2;
		Modbus_Registers_Detector_Write.CALIB_FACTOR3 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR3;
		Modbus_Registers_Detector_Write.CALIB_FACTOR4 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR4;

		calib_factor_reconfigure = true;
	}

	if(Modbus_Registers_PC_TCP_Write.FREQ_DTC != Modbus_Registers_PC_TCP.FREQ_RECV)
	{
		dtc_freq_switch_val = (Modbus_Registers_PC_TCP_Write.FREQ_DTC == 1) ? 1:0;
		dtc_freq_switch = true;
	}

	if(hv_reconfigure == true && calib_factor_reconfigure == false && dtc_freq_switch == false)
	{
		modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = HV_WRITE_FLAG;

		hv_reconfigure = false;

	}
	else if(hv_reconfigure == false && calib_factor_reconfigure == true && dtc_freq_switch == false)
	{

		modbus_write_flag   = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = CALIB_WRITE_FLAG;

		calib_factor_reconfigure = false;

	}
	else if(hv_reconfigure == false && calib_factor_reconfigure == false && dtc_freq_switch == true)
	{
		modbus_write_flag = MODBUS_SINGLE_WRITE_FLAG;
		single_write_flag = DTC_FREQ_SWITCH_FLAG;

		dtc_freq_switch = false;
	}
	else if(hv_reconfigure == true && calib_factor_reconfigure == true && dtc_freq_switch == false)
	{
		//Need to Update FREQ Switch
		dtc_freq_switch_val =  freq_dtc_mode;

		modbus_write_flag   = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = HV_AND_CALIB_WRITE_FLAG;

		hv_reconfigure = false;
		calib_factor_reconfigure = false;
	}
	else if(hv_reconfigure == true && calib_factor_reconfigure == false && dtc_freq_switch == true)
	{
		modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = HV_AND_FREQ_WRITE_FLAG;

		hv_reconfigure  = false;
		dtc_freq_switch = false;
	}
	else if(hv_reconfigure == false && calib_factor_reconfigure == true && dtc_freq_switch == true)
	{
		modbus_write_flag   = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = ALL_CONFIG_WRITE_FLAG;

		calib_factor_reconfigure = true;
		dtc_freq_switch          = true;
	}
	else if(hv_reconfigure == true && calib_factor_reconfigure == true && dtc_freq_switch == true)
	{
		modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = ALL_CONFIG_WRITE_FLAG;

		hv_reconfigure           = false;
		calib_factor_reconfigure = false;
		dtc_freq_switch          = false;
	}
}

void Reconfigure_RS485(void)
{
	int code = Modbus_Registers.RS485_Baud_Rate;
	pending_new_baud = GetBaudRate(code);
//	pending_baud_change = 1;

	// ---------Register Slave Id------------
	Modbus_Slave_Id_PC = Modbus_Registers.RS485_Slave_Id;

	PC_UART_ReInit(pending_new_baud);
}

void read_e4_20ma_CalibVal(void)
{
	uint32_t e4mA_scaled;
	uint32_t e20mA_scaled;

	e4mA_scaled = ((uint32_t)Modbus_Registers_Write.E4MA_CALIB_MSB << 16)
				  |  (uint32_t)Modbus_Registers_Write.E4MA_CALIB_LSB;

	snprintf(e4mA_buf, sizeof(e4mA_buf), "%lu.%04lu",
			(unsigned long)(e4mA_scaled / 10000),
			(unsigned long)(e4mA_scaled % 10000));


	e20mA_scaled = ((uint32_t)Modbus_Registers_Write.E20MA_CALIB_MSB << 16)
						  |  (uint32_t)Modbus_Registers_Write.E20MA_CALIB_LSB;

	snprintf(e20mA_buf, sizeof(e20mA_buf), "%lu.%04lu",
			(unsigned long)(e20mA_scaled / 10000),
			(unsigned long)(e20mA_scaled % 10000));

	e4mA_val  = (double)e4mA_scaled  / 10000.0;
	e20mA_val = (double)e20mA_scaled / 10000.0;
}

void read_e4_20mA_factors(void)
{
	int32_t x = ((int32_t)g_config.CAL_4_20MA_FACTA_MSB << 16) |
	                 g_config.CAL_4_20MA_FACTA_LSB;

	int32_t y = ((int32_t)g_config.CAL_4_20MA_FACTB_MSB << 16) |
	                 g_config.CAL_4_20MA_FACTB_LSB;

	g_cal_a = (double)x/100.0f;
	g_cal_b = (double)y/100.0f;
}

void update_detector_hv_register_rs485(void)
{
   Modbus_Registers_Detector.HV_MSB = Modbus_Registers_Write.HV_MSB;
   Modbus_Registers_Detector.HV_LSB = Modbus_Registers_Write.HV_LSB;
}

void update_detector_calib_registers_rs485(void)
{
	Modbus_Registers_Detector.CALIB_FACTOR1  = Modbus_Registers_Write.CALIB_FACTOR1;
	Modbus_Registers_Detector.CALIB_FACTOR2  = Modbus_Registers_Write.CALIB_FACTOR2;
	Modbus_Registers_Detector.CALIB_FACTOR3  = Modbus_Registers_Write.CALIB_FACTOR3;
	Modbus_Registers_Detector.CALIB_FACTOR4  = Modbus_Registers_Write.CALIB_FACTOR4;
}

void update_detector_hv_register_tcp(void)
{
   Modbus_Registers_Detector_Write.HV_MSB = Modbus_Registers_PC_TCP_Write.HV_MSB;
   Modbus_Registers_Detector_Write.HV_LSB = Modbus_Registers_PC_TCP_Write.HV_LSB;
}

void update_detector_calib_registers_tcp(void)
{
	Modbus_Registers_Detector_Write.CALIB_FACTOR1  = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR1;
	Modbus_Registers_Detector_Write.CALIB_FACTOR2  = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR2;
	Modbus_Registers_Detector_Write.CALIB_FACTOR3  = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR3;
	Modbus_Registers_Detector_Write.CALIB_FACTOR4  = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR4;
}

int GetBaudRate(uint8_t baud_index)
{
	switch(baud_index)
	{
	case 0:
		   return 2400;
	case 1:
		   return 4800;
	case 2:
		   return 9600;
	case 3:
		   return 14400;
	case 4:
		   return 19200;
	case 5:
		   return 38400;
	case 6:
		   return 57600;
	case 7:
		   return 115200;
	default:
		   return 9600;
	}
}

bool Validate_Date_RS485_Registers(uint16_t day,uint16_t month,uint16_t year)
{
	if(month < 1 || month > 12)
		return false;

	if(year < 2000 || year > 2099)
		return false;

	uint8_t days_in_month[] =
	{
			31,28,31,30,31,30,
			31,31,30,31,30,31
	};

	if(IsLeapYear(year))
		days_in_month[1] = 29;

	if(day < 1 || day > days_in_month[month-1])
		return false;

	return true;
}

bool Validate_Time_RS485_Registers(uint16_t hour,uint16_t min,uint16_t sec)
{
	if (hour > 23)
		return false;

	if (min > 59)
		return false;

	if (sec > 59)
		return false;

	return true;
}

void update_rtc_registers(void)
{
	if(g_1s_flags.rtc_poll)
	{
		RTC_TimeTypeDef sTime = {0};
		RTC_DateTypeDef sDate = {0};

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		Modbus_Registers.RTC_DAY   = (uint16_t)sDate.Date;
		Modbus_Registers.RTC_MONTH = (uint16_t)sDate.Month;
		Modbus_Registers.RTC_YEAR  = (uint16_t)sDate.Year + 2000;
		Modbus_Registers.RTC_HOUR  = (uint16_t)sTime.Hours;
		Modbus_Registers.RTC_MIN   = (uint16_t)sTime.Minutes;
		Modbus_Registers.RTC_SEC   = (uint16_t)sTime.Seconds;

		Modbus_Registers_PC_TCP.RTC_DAY   = (uint16_t)sDate.Date;
		Modbus_Registers_PC_TCP.RTC_MONTH = (uint16_t)sDate.Month;
		Modbus_Registers_PC_TCP.RTC_YEAR  = (uint16_t)sDate.Year + 2000;
		Modbus_Registers_PC_TCP.RTC_HOUR  = (uint16_t)sTime.Hours;
		Modbus_Registers_PC_TCP.RTC_MIN   = (uint16_t)sTime.Minutes;
		Modbus_Registers_PC_TCP.RTC_SEC   = (uint16_t)sTime.Seconds;

		printf("%02d:%02d:%02d\r\n",
				sTime.Hours,
				sTime.Minutes,
				sTime.Seconds);

		g_1s_flags.rtc_poll = false;
	}
}

void Fill_Record(void)
{
	sd_logs.RTC_DAY   = Modbus_Registers.RTC_DAY;
	sd_logs.RTC_MONTH = Modbus_Registers.RTC_MONTH;
	sd_logs.RTC_YEAR  = Modbus_Registers.RTC_YEAR;
	sd_logs.RTC_HOUR  = Modbus_Registers.RTC_HOUR;
	sd_logs.RTC_MIN   = Modbus_Registers.RTC_MIN;
    sd_logs.RTC_SEC   = Modbus_Registers.RTC_SEC;
    sd_logs.Overload  = Modbus_Registers.Overload;
    sd_logs.Overrun   = Modbus_Registers.Overrun;
    sd_logs.Alarm     = Modbus_Registers.Alarm;
    sd_logs.Hv_Fault  = Modbus_Registers.Hv_Fault;
    sd_logs.Dtc_Fault = Modbus_Registers.Dtc_Fault;
    sd_logs.CPS_MSB   = Modbus_Registers.CPS_MSB;
    sd_logs.CPS_LSB   = Modbus_Registers.CPS_LSB;
    sd_logs.mR_MSB    = Modbus_Registers.mR_MSB;
    sd_logs.mR_LSB    = Modbus_Registers.mR_LSB;
}

uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len)
{
	uint16_t crc = 0xFFFF;
	for (int pos = 0; pos < len; pos++) {
		crc ^= (uint16_t)buf[pos];
		for (int i = 8; i != 0; i--) {
			if ((crc & 0x0001) != 0) {
				crc >>= 1;
				crc ^= 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART7)
	{
		DTC_UART_Error_Flag = 1;
	}

	if(huart->Instance == UART8)
	{
		PC_UART_Error_Flag = 1;
	}
}

//void detector_write_tcp(uint16_t start_addr , uint16_t reg_cnt)
//{
//	if(reg_cnt >= MAX_REG_WRITE)
//	{
//		update_detector_hv_register_tcp();
//		update_detector_calib_registers_tcp();
//		multiple_write_flag = ALL_CONFIG_WRITE_FLAG;
//	}
//	else if(start_addr == MODBUS_HV_ADDRESS)
//	{
//		update_detector_hv_register_tcp();
//		multiple_write_flag = HV_WRITE_FLAG;
//	}
//	else if(start_addr == MODBUS_CALIB_ADDRESS)
//	{
//		update_detector_calib_registers_tcp();
//		multiple_write_flag = CALIB_WRITE_FLAG;
//	}
//}

//void detector_write_tcp(uint16_t start_addr , uint16_t reg_cnt)
//{
//	if(reg_cnt >= MAX_REG_WRITE)
//	{
//		update_detector_hv_register_tcp();
//		update_detector_calib_registers_tcp();
//		multiple_write_flag = ALL_CONFIG_WRITE_FLAG;
//	}
//	else if(start_addr == MODBUS_HV_ADDRESS)
//	{
//		update_detector_hv_register_tcp();
//		multiple_write_flag = HV_WRITE_FLAG;
//	}
//	else if(start_addr == MODBUS_CALIB_ADDRESS)
//	{
//		update_detector_calib_registers_tcp();
//		multiple_write_flag = CALIB_WRITE_FLAG;
//	}
//}

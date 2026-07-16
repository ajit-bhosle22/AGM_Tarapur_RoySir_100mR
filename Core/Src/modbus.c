
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>
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
#include "tcp_client.h"


#define MR_25_VAL            2.5f   //2.5mR
#define MR_50_VAL            5.0f   //5mR
#define MR_75_VAL            7.5f   //7.5mR
#define MR_100_VAL           10.0f  //10mR

#define DAC_4MA_CODE         738
#define DAC_20MA_CODE        3608

#define MODBUS_HV_ADDRESS     43
#define MODBUS_CALIB_ADDRESS  49

#define FACTOR_MIN  0.01

volatile uint8_t tx_complete_PC_sd = 0;
volatile uint8_t ack_received_PC_sd = 0;
volatile uint8_t  frame_recv_PC_sd = 0;
volatile uint16_t rx_index_PC_sd = 0;
uint8_t rx_buffer_dma_PC_sd[BUFFER_SIZE];

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

volatile uint16_t tcp_slave_id            = 1;
volatile uint16_t tcp_current_port        = 510;
volatile uint16_t tcp_current_port_pc     = 502;
bool tcp_reconfig_required = false;
bool rs485_reconfig_required = false;
bool tcp_reconfig_required_pc = false;

double dose_uRh;
double e4mA_val  = 0.0f;
double e20mA_val = 0.0f;
char e4mA_buf[16];
char e20mA_buf[16];
uint32_t scaled_umr;

char Factor1_Value[16]    = "1";
char Factor2_Value[16]    = "1";
char Factor3_Value[16]    = "1";
char Factor4_Value[16]    = "1";

static double Factor1 = 1.0;
static double Factor2 = 1.0;
static double Factor3 = 1.0;
static double Factor4 = 1.0;

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
		if(read_sd_flag)
		{
			rx_index_PC_sd = Size;
			frame_recv_PC_sd = 1;
		}else{
		rx_index_PC = Size;
		frame_recv_PC = 1;
		}
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
		if(read_sd_flag){
		tx_complete_PC_sd = 1;
		RS485_RX_MODE_PC();
		Modbus_Restart_RX_DMA_PC_SD();
		}
		else{
		RS485_RX_MODE_PC();
		Modbus_Restart_RX_DMA_PC();
		}
	}
}

static double register_to_factor(uint32_t reg_val)
{
    return (double)reg_val / 100.0;
}

void update_cps_hv_mr(void)
{
	static uint32_t cps_arr[60]  = {0};
	static int idx_pos = 0;
	uint32_t sum=0;

	if(Modbus_Registers.FREQ_RECV == 0){
		CPS_VAL = (Modbus_Registers_Detector.CPS_MSB<<16)|(Modbus_Registers_Detector.CPS_LSB);
	}
	else{
		CPS_VAL =  recv_pulse_per_sec;
	}

	// calculate cpm
	cps_arr[idx_pos] = CPS_VAL;
	idx_pos = (idx_pos + 1) % 60;

	for (int i = 0; i < 60; i++){
		sum += cps_arr[i];
	}
	cpm = sum;

	Hv_Val = (Modbus_Registers_Detector.HV_MSB << 16)| (Modbus_Registers_Detector.HV_LSB);
	Hv_Voltage = (float) (Hv_Val/100.0f);

	dose_uRh = convert_cps_to_ur_h();
	scaled_umr = (uint32_t)(dose_uRh * 1000.0f);

	Modbus_Registers.mR_MSB = (scaled_umr >>16) & 0xFFFF;
	Modbus_Registers.mR_LSB = scaled_umr & 0xFFFF;

	Modbus_Registers_PC_TCP.mR_MSB = (scaled_umr >>16) & 0xFFFF;
	Modbus_Registers_PC_TCP.mR_LSB = scaled_umr & 0xFFFF;
}

void initialize_modbus_registers()
{
	if(Modbus_Registers.FREQ_RECV == 0){

		Modbus_Registers.CPS_MSB               = Modbus_Registers_Detector.CPS_MSB;
		Modbus_Registers.CPS_LSB               = Modbus_Registers_Detector.CPS_LSB;

		Modbus_Registers_PC_TCP.CPS_MSB        = Modbus_Registers_Detector.CPS_MSB;
		Modbus_Registers_PC_TCP.CPS_LSB        = Modbus_Registers_Detector.CPS_LSB;
	}
	else
	{

		Modbus_Registers.CPS_MSB               = (recv_pulse_per_sec >> 16 & 0xFFFF);
		Modbus_Registers.CPS_LSB               = recv_pulse_per_sec & 0xFFFF;

		Modbus_Registers_PC_TCP.CPS_MSB        = (recv_pulse_per_sec >> 16 & 0xFFFF);
		Modbus_Registers_PC_TCP.CPS_LSB        = recv_pulse_per_sec & 0xFFFF;
	}

	Modbus_Registers.HV_MSB                = Modbus_Registers_Detector.HV_MSB;
	Modbus_Registers.HV_LSB                = Modbus_Registers_Detector.HV_LSB;

	Modbus_Registers_PC_TCP.HV_MSB         = Modbus_Registers_Detector.HV_MSB;
	Modbus_Registers_PC_TCP.HV_LSB         = Modbus_Registers_Detector.HV_LSB;

	Modbus_Registers.CALIB_FACTOR1         = Modbus_Registers_Detector.CALIB_FACTOR1;
	Modbus_Registers.CALIB_FACTOR2         = Modbus_Registers_Detector.CALIB_FACTOR2;
	Modbus_Registers.CALIB_FACTOR3         = Modbus_Registers_Detector.CALIB_FACTOR3;
	Modbus_Registers.CALIB_FACTOR4         = Modbus_Registers_Detector.CALIB_FACTOR4;

	Modbus_Registers_PC_TCP.CALIB_FACTOR1  = Modbus_Registers_Detector.CALIB_FACTOR1;
	Modbus_Registers_PC_TCP.CALIB_FACTOR2  = Modbus_Registers_Detector.CALIB_FACTOR2;
	Modbus_Registers_PC_TCP.CALIB_FACTOR3  = Modbus_Registers_Detector.CALIB_FACTOR3;
	Modbus_Registers_PC_TCP.CALIB_FACTOR4  = Modbus_Registers_Detector.CALIB_FACTOR4;

	Modbus_Registers.FREQ_DTC              = Modbus_Registers_Detector.DTC_FRQ_SWITCH;
	Modbus_Registers_PC_TCP.FREQ_DTC       = Modbus_Registers_Detector.DTC_FRQ_SWITCH;
}

void config_CalibFactors_Variables(void)
{
    double f;

    f = register_to_factor(Modbus_Registers.CALIB_FACTOR1);
    Factor1 = (f >= FACTOR_MIN) ? f : FACTOR_MIN;

    f = register_to_factor(Modbus_Registers.CALIB_FACTOR2);
    Factor2 = (f >= FACTOR_MIN) ? f : FACTOR_MIN;

    f = register_to_factor(Modbus_Registers.CALIB_FACTOR3);
    Factor3 = (f >= FACTOR_MIN) ? f : FACTOR_MIN;

    f = register_to_factor(Modbus_Registers.CALIB_FACTOR4);
    Factor4 = (f >= FACTOR_MIN) ? f : FACTOR_MIN;
}

double convert_cps_to_ur_h(void)
{
	double dose_mr = (double)CPS_VAL / (double)GM_TUBE_SENSITIVITY;

	if (dose_mr <= (MR_25_VAL * Factor1))
	{
		dose_mr = dose_mr / Factor1;
	}
	else if (dose_mr <= (MR_50_VAL * Factor2))
	{
		double raw_25 = MR_25_VAL * Factor1;
		double raw_50 = MR_50_VAL * Factor2;
		double f1     = 1.0 / Factor1;
		double f2     = 1.0 / Factor2;
		double r1     = dose_mr - raw_25;
		double r2     = raw_50  - raw_25;
		double factor = f1 + ((r1 * (f2 - f1)) / r2);
		dose_mr = dose_mr * factor;
	}
	else if (dose_mr <= (MR_75_VAL * Factor3))
	{
		double raw_50 = MR_50_VAL * Factor2;
		double raw_75 = MR_75_VAL * Factor3;
		double f1     = 1.0 / Factor2;
		double f2     = 1.0 / Factor3;
		double r1     = dose_mr - raw_50;
		double r2     = raw_75  - raw_50;
		double factor = f1 + ((r1 * (f2 - f1)) / r2);
		dose_mr = dose_mr * factor;
	}
	else if(dose_mr >= MR_100_VAL)
	{
		dose_mr = dose_mr / Factor4;
	}

	double dose_uR = dose_mr * 1000.0;
	return dose_uR;
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

	Modbus_Registers.SD_MODE         = sd_mode;
	Modbus_Registers_PC_TCP.SD_MODE  = sd_mode;

	Modbus_Registers.Primary_Unit = primary_unit;
	Modbus_Registers_PC_TCP.Primary_Unit = primary_unit;

	uint16_t k = atoi(saved_rs485_slave_id);
	Modbus_Registers.RS485_Slave_Id = k;
	Modbus_Registers.RS485_Baud_Rate = baud_rate;
	Modbus_Registers_PC_TCP.RS485_Slave_Id = k;
	Modbus_Registers_PC_TCP.RS485_Baud_Rate = baud_rate;

	Modbus_Registers.Alarm_Unit = alarm_unit;
	Modbus_Registers_PC_TCP.Alarm_Unit = alarm_unit;
//	double z = atof(alarm_val);
	uint32_t x = atoi(alarm_val);
//	z = z*100;
//	x = (uint32_t)(z + 0.5);

	Modbus_Registers.Alarm_Value_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers.Alarm_Value_LSB = (x & 0xFFFF);
	Modbus_Registers_PC_TCP.Alarm_Value_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers_PC_TCP.Alarm_Value_LSB = (x & 0xFFFF);

	Modbus_Registers.Overrange_Unit = overrange_unit;
	Modbus_Registers_PC_TCP.Overrange_Unit = overrange_unit;
//	z = atof(overrange_val);
//	z = z*100;
	x = atoi(overrange_val);
//	x = (uint32_t)(z + 0.5);
	Modbus_Registers.Overrange_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers.Overrange_LSB = (x & 0xFFFF);
	Modbus_Registers_PC_TCP.Overrange_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers_PC_TCP.Overrange_LSB = (x & 0xFFFF);

	Modbus_Registers.Overload_Unit = overload_unit;
	Modbus_Registers_PC_TCP.Overload_Unit = overload_unit;
//	z = atof(overload_val);
//	z = z *100;
//	x = (uint32_t)(z + 0.5);
	x = atoi(overload_val);
	Modbus_Registers.Overload_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers.Overload_LSB = (x & 0xFFFF);
	Modbus_Registers_PC_TCP.Overload_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers_PC_TCP.Overload_LSB = (x & 0xFFFF);

	Modbus_Registers.Analog_4_to_20mA_Min_Unit = min_4_20_unit;
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_Unit = min_4_20_unit;
//	z = atof(min_4_20_val);
//	z = z*100;
//	x = (uint32_t)(z + 0.5);
	x = atoi(min_4_20_val);
	Modbus_Registers.Analog_4_to_20mA_Min_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers.Analog_4_to_20mA_Min_LSB =  (x & 0xFFFF);
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_LSB = (x & 0xFFFF);

	Modbus_Registers.Analog_4_to_20mA_Max_Unit = max_4_20_unit;
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_Unit = max_4_20_unit;
//	z = atof(max_4_20_val);
//	z = z*100;
//	x = (uint32_t)(z + 0.5);
	x = atoi(max_4_20_val);
	Modbus_Registers.Analog_4_to_20mA_Max_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers.Analog_4_to_20mA_Max_LSB = (x & 0xFFFF);
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_MSB = ((x>>16)&0xFFFF);
	Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_LSB = (x & 0xFFFF);

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

	k= atoi(saved_eth_port_pc);
	Modbus_Registers.Ethernet_Port_PC = k;
	Modbus_Registers_PC_TCP.Ethernet_Port_PC = k;

	sscanf((char*)saved_eth_ip_pc, "%hu.%hu.%hu.%hu",
			&octet1, &octet2, &octet3, &octet4);

	Modbus_Registers.Ethernet_IP_MSB_PC = (octet1 << 8) | (octet2 & 0xFF);
	Modbus_Registers.Ethernet_IP_LSB_PC = (octet3 << 8) | (octet4 & 0xFF);
	Modbus_Registers_PC_TCP.Ethernet_IP_MSB_PC = (octet1 << 8) | (octet2 & 0xFF);
	Modbus_Registers_PC_TCP.Ethernet_IP_LSB_PC = (octet3 << 8) | (octet4 & 0xFF);

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
	Modbus_Registers.Ethernet_IP_MSB_PC        = g_config.Ethernet_IP_MSB_PC;
	Modbus_Registers.Ethernet_IP_LSB_PC        = g_config.Ethernet_IP_LSB_PC;
	Modbus_Registers.Ethernet_Port_PC          = g_config.Ethernet_Port_PC;
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
//    Modbus_Registers.FREQ_RECV                 = g_config.FREQ_RECV;
//    Modbus_Registers.FREQ_DTC                  = g_config.FREQ_DTC;
    Modbus_Registers.SD_MODE                   = g_config.SD_MODE;

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
	Modbus_Registers_PC_TCP.Ethernet_IP_MSB_PC   = g_config.Ethernet_IP_MSB_PC;
	Modbus_Registers_PC_TCP.Ethernet_IP_LSB_PC   = g_config.Ethernet_IP_LSB_PC;
	Modbus_Registers_PC_TCP.Ethernet_Port_PC     = g_config.Ethernet_Port_PC;
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
//    Modbus_Registers_PC_TCP.FREQ_RECV                 = g_config.FREQ_RECV;
//    Modbus_Registers_PC_TCP.FREQ_DTC                  = g_config.FREQ_DTC;
    Modbus_Registers_PC_TCP.SD_MODE                   = g_config.SD_MODE;
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

	if(Modbus_Registers.SD_MODE == 0)
	{
        sd_mode = 0;
        strcpy(temp_sdmode_str,"1. RTU");
	}
	else
	{
        sd_mode = 1;
        strcpy(temp_sdmode_str,"2. TCP/IP");
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
	 * Ethernet Settings (FOR DEVICE)
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


    /* -------------------ETH SET (FOR PC) ---------------------- */
	pc_ip[0] = (Modbus_Registers.Ethernet_IP_MSB_PC >> 8) & 0xFF;
	pc_ip[1] =  Modbus_Registers.Ethernet_IP_MSB_PC       & 0xFF;
	pc_ip[2] = (Modbus_Registers.Ethernet_IP_LSB_PC >> 8) & 0xFF;
	pc_ip[3] =  Modbus_Registers.Ethernet_IP_LSB_PC       & 0xFF;

	snprintf((char*)saved_eth_ip_pc, sizeof(saved_eth_ip_pc),
			"%03u.%03u.%03u.%03u",
			(Modbus_Registers.Ethernet_IP_MSB_PC >> 8) & 0xFF,
			Modbus_Registers.Ethernet_IP_MSB_PC       & 0xFF,
			(Modbus_Registers.Ethernet_IP_LSB_PC >> 8) & 0xFF,
			Modbus_Registers.Ethernet_IP_LSB_PC       & 0xFF);

	tcp_current_port_pc = Modbus_Registers.Ethernet_Port_PC;
	snprintf((char*)saved_eth_port_pc, sizeof(saved_eth_port_pc),
				"%04u", Modbus_Registers.Ethernet_Port_PC);


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
    if(k > 999999U)
    {
    	k = 999999U;
    }
    snprintf(buf, buf_size, "%06lu  ", (unsigned long)k);
}

//static void modbus_val_to_str(uint16_t msb, uint16_t lsb,
//                               uint8_t unit, char* buf, size_t buf_size)
//{
//    uint32_t k          = ((uint32_t)msb << 16) | lsb;
//    float    z          = ((float)k / 100.0f);
//    z                   = ((int)(z * 1000.0f + 0.5f)) / 1000.0f;
//    uint32_t factor_x100 = (uint32_t)(z * 100.0f + 0.5f);
//
//    if (unit_is_integer(unit))
//    {
//        uint32_t int_val = factor_x100 / 100;
//        if (int_val > 99999U) int_val = 99999U;
//        snprintf(buf, buf_size, "%05lu  ", (unsigned long)int_val);
//    }
//    else
//    {
//        if ((factor_x100 / 100) > 9999U) factor_x100 = 999999U;
//        snprintf(buf, buf_size, "%04lu.%02lu",
//                 (unsigned long)(factor_x100 / 100),
//                 (unsigned long)(factor_x100 % 100));
//    }
//}

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

	if(tcp_reconfig_required_pc == true)
	{
		tcp_reconfig_required_pc = false;
        Reconfig_TCP_PC();
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
	if((Modbus_Registers.Ethernet_IP_MSB != Modbus_Registers_Write.Ethernet_IP_MSB) || (Modbus_Registers.Ethernet_IP_LSB != Modbus_Registers_Write.Ethernet_IP_LSB) ||
	(Modbus_Registers.Ethernet_Subnet_MSB != Modbus_Registers_Write.Ethernet_Subnet_MSB) || (Modbus_Registers.Ethernet_Subnet_LSB != Modbus_Registers_Write.Ethernet_Subnet_LSB) ||
	(Modbus_Registers.Ethernet_Gateway_MSB != Modbus_Registers_Write.Ethernet_Gateway_MSB) || (Modbus_Registers.Ethernet_Gateway_LSB != Modbus_Registers_Write.Ethernet_Gateway_LSB) ||
	(Modbus_Registers.Ethernet_Slave_Id != Modbus_Registers_Write.Ethernet_Slave_Id) || (Modbus_Registers.Ethernet_Port != Modbus_Registers_Write.Ethernet_Port))
	{
		tcp_reconfig_required = true;
	}

	if((Modbus_Registers.RS485_Slave_Id != Modbus_Registers_Write.RS485_Slave_Id) ||
	(Modbus_Registers.RS485_Baud_Rate != Modbus_Registers_Write.RS485_Baud_Rate))
	{
		rs485_reconfig_required = true;
	}

	if((Modbus_Registers.Ethernet_IP_MSB_PC != Modbus_Registers_Write.Ethernet_IP_MSB_PC) || (Modbus_Registers.Ethernet_IP_LSB_PC != Modbus_Registers_Write.Ethernet_IP_LSB_PC)
	|| (Modbus_Registers.Ethernet_Port_PC != Modbus_Registers_Write.Ethernet_Port_PC))
	{
		tcp_reconfig_required_pc = true;
	}
}

void is_reconfigure_rs485_tcp_pc(void)
{

	if((Modbus_Registers_PC_TCP.Ethernet_IP_MSB != Modbus_Registers.Ethernet_IP_MSB) || (Modbus_Registers_PC_TCP.Ethernet_IP_LSB != Modbus_Registers.Ethernet_IP_LSB) ||
	(Modbus_Registers_PC_TCP.Ethernet_Subnet_MSB != Modbus_Registers.Ethernet_Subnet_MSB) || (Modbus_Registers_PC_TCP.Ethernet_Subnet_LSB != Modbus_Registers.Ethernet_Subnet_LSB) ||
	(Modbus_Registers_PC_TCP.Ethernet_Gateway_MSB != Modbus_Registers.Ethernet_Gateway_MSB) || (Modbus_Registers_PC_TCP.Ethernet_Gateway_LSB != Modbus_Registers.Ethernet_Gateway_LSB) ||
	(Modbus_Registers_PC_TCP.Ethernet_Slave_Id != Modbus_Registers.Ethernet_Slave_Id) || (Modbus_Registers_PC_TCP.Ethernet_Port != Modbus_Registers.Ethernet_Port))
	{
		tcp_reconfig_required = true;
	}

	if((Modbus_Registers.RS485_Slave_Id != Modbus_Registers_PC_TCP.RS485_Slave_Id) ||
	(Modbus_Registers.RS485_Baud_Rate != Modbus_Registers_PC_TCP.RS485_Baud_Rate))
	{
		rs485_reconfig_required = true;
	}

	if((Modbus_Registers_PC_TCP.Ethernet_IP_MSB_PC != Modbus_Registers.Ethernet_IP_MSB_PC) || (Modbus_Registers_PC_TCP.Ethernet_IP_LSB_PC != Modbus_Registers.Ethernet_IP_LSB_PC)
	||(Modbus_Registers_PC_TCP.Ethernet_Port_PC != Modbus_Registers.Ethernet_Port_PC))
	{
		tcp_reconfig_required_pc = true;
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

void check_ack_and_reset_rs485(uint16_t addr,uint16_t reg_cnt)
{
	if((addr <= REG_ACK) && ((addr +(reg_cnt - 1)) >= REG_ACK)){
		Ack_Button = Modbus_Registers_Write.ACK;
		Modbus_Registers_Write.ACK = 0;
	}

	if((addr <= REG_RESET) && ((addr +(reg_cnt - 1)) >= REG_RESET)){
		Reset_Button = Modbus_Registers_Write.Reset;
		Modbus_Registers_Write.Reset = 0;
	}
}

void is_reconfigure_hv_rs485(uint16_t addr, uint16_t reg_cnt)
{
	bool hv_reconfigure = false;
	bool calib_factor_reconfigure = false;
    bool dtc_freq_switch = false;

    if((addr <= REG_HV_MSB && (addr +(reg_cnt - 1)) >= REG_HV_MSB)
    		||(addr <= REG_HV_LSB && (addr +(reg_cnt - 1)) >= REG_HV_LSB)){

    	Modbus_Registers_Detector_Write.HV_MSB = Modbus_Registers_Write.HV_MSB;
    	Modbus_Registers_Detector_Write.HV_LSB = Modbus_Registers_Write.HV_LSB;

    	hv_reconfigure = true;
    }

//	if((Modbus_Registers_Write.HV_MSB != Modbus_Registers.HV_MSB)
//			|| Modbus_Registers_Write.HV_LSB != Modbus_Registers.HV_LSB)
//	{
//		Modbus_Registers_Detector_Write.HV_MSB = Modbus_Registers_Write.HV_MSB;
//		Modbus_Registers_Detector_Write.HV_LSB = Modbus_Registers_Write.HV_LSB;
//
//		hv_reconfigure = true;
//	}

    if((addr <= REG_CALIB_FACTOR1 && (addr +(reg_cnt - 1)) >= REG_CALIB_FACTOR1)
    		||(addr <= REG_CALIB_FACTOR2 && (addr +(reg_cnt - 1)) >= REG_CALIB_FACTOR2)
			||(addr <= REG_CALIB_FACTOR3 && (addr +(reg_cnt - 1)) >= REG_CALIB_FACTOR3)
			||(addr <= REG_CALIB_FACTOR4 && (addr +(reg_cnt - 1)) >= REG_CALIB_FACTOR4) )
    {
    	Modbus_Registers_Detector_Write.CALIB_FACTOR1 = Modbus_Registers_Write.CALIB_FACTOR1;
    	Modbus_Registers_Detector_Write.CALIB_FACTOR2 = Modbus_Registers_Write.CALIB_FACTOR2;
    	Modbus_Registers_Detector_Write.CALIB_FACTOR3 = Modbus_Registers_Write.CALIB_FACTOR3;
    	Modbus_Registers_Detector_Write.CALIB_FACTOR4 = Modbus_Registers_Write.CALIB_FACTOR4;

    	calib_factor_reconfigure = true;
    }

//	if((Modbus_Registers_Write.CALIB_FACTOR1 != Modbus_Registers.CALIB_FACTOR1 )
//	|| (Modbus_Registers_Write.CALIB_FACTOR2 != Modbus_Registers.CALIB_FACTOR2)
//	|| (Modbus_Registers_Write.CALIB_FACTOR3 != Modbus_Registers.CALIB_FACTOR3)
//	|| (Modbus_Registers_Write.CALIB_FACTOR4 != Modbus_Registers.CALIB_FACTOR4)){
//
//		Modbus_Registers_Detector_Write.CALIB_FACTOR1 = Modbus_Registers_Write.CALIB_FACTOR1;
//		Modbus_Registers_Detector_Write.CALIB_FACTOR2 = Modbus_Registers_Write.CALIB_FACTOR2;
//		Modbus_Registers_Detector_Write.CALIB_FACTOR3 = Modbus_Registers_Write.CALIB_FACTOR3;
//		Modbus_Registers_Detector_Write.CALIB_FACTOR4 = Modbus_Registers_Write.CALIB_FACTOR4;
//
//		calib_factor_reconfigure = true;
//	}

	if(addr <= FREQ_DTC_SWITCH && (addr +(reg_cnt - 1)) >= FREQ_DTC_SWITCH){
		dtc_freq_switch_val = (Modbus_Registers_Write.FREQ_DTC == 1) ? 1:0;
		dtc_freq_switch = true;
	}

//	if(Modbus_Registers_Write.FREQ_DTC != Modbus_Registers.FREQ_DTC)
//	{
//		dtc_freq_switch_val = (Modbus_Registers_Write.FREQ_DTC == 1) ? 1:0;
//		dtc_freq_switch = true;
//	}

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
		dtc_freq_switch_val =  Modbus_Registers.FREQ_DTC;

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

void check_ack_and_reset_tcp(uint16_t addr,uint16_t reg_cnt)
{
	if((addr <= REG_ACK) && ((addr +(reg_cnt - 1)) >= REG_ACK)){
		Ack_Button = Modbus_Registers_PC_TCP_Write.ACK;
		Modbus_Registers_PC_TCP_Write.ACK = 0;
	}

	if((addr <= REG_RESET) && ((addr +(reg_cnt - 1)) >= REG_RESET)){
		Reset_Button = Modbus_Registers_PC_TCP_Write.Reset;
		Modbus_Registers_PC_TCP_Write.Reset = 0;
	}
}

void is_reconfigure_hv_tcp(uint16_t addr,uint16_t reg_cnt)
{
	bool hv_reconfigure = false;
	bool calib_factor_reconfigure = false;
	bool dtc_freq_switch = false;

	if((addr <= REG_HV_MSB && (addr +(reg_cnt - 1)) >= REG_HV_MSB)
			||(addr <= REG_HV_LSB && (addr +(reg_cnt - 1)) >= REG_HV_LSB)){

		Modbus_Registers_Detector_Write.HV_MSB = Modbus_Registers_PC_TCP_Write.HV_MSB;
		Modbus_Registers_Detector_Write.HV_LSB = Modbus_Registers_PC_TCP_Write.HV_LSB;

		hv_reconfigure = true;
	}

	//	if((Modbus_Registers_PC_TCP_Write.HV_MSB != Modbus_Registers_PC_TCP.HV_MSB)
	//			|| Modbus_Registers_PC_TCP_Write.HV_LSB != Modbus_Registers_PC_TCP.HV_LSB)
	//	{
	//		Modbus_Registers_Detector_Write.HV_MSB = Modbus_Registers_PC_TCP_Write.HV_MSB;
	//		Modbus_Registers_Detector_Write.HV_LSB = Modbus_Registers_PC_TCP_Write.HV_LSB;
	//
	//		hv_reconfigure = true;
	//	}

    if((addr <= REG_CALIB_FACTOR1 && (addr +(reg_cnt - 1)) >= REG_CALIB_FACTOR1)
    		||(addr <= REG_CALIB_FACTOR2 && (addr +(reg_cnt - 1)) >= REG_CALIB_FACTOR2)
			||(addr <= REG_CALIB_FACTOR3 && (addr +(reg_cnt - 1)) >= REG_CALIB_FACTOR3)
			||(addr <= REG_CALIB_FACTOR4 && (addr +(reg_cnt - 1)) >= REG_CALIB_FACTOR4) )
    {
    	Modbus_Registers_Detector_Write.CALIB_FACTOR1 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR1;
    	Modbus_Registers_Detector_Write.CALIB_FACTOR2 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR2;
    	Modbus_Registers_Detector_Write.CALIB_FACTOR3 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR3;
    	Modbus_Registers_Detector_Write.CALIB_FACTOR4 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR4;

    	calib_factor_reconfigure = true;
    }

//	if((Modbus_Registers_PC_TCP_Write.CALIB_FACTOR1 != Modbus_Registers_PC_TCP.CALIB_FACTOR1 )
//			|| (Modbus_Registers_PC_TCP_Write.CALIB_FACTOR2 != Modbus_Registers_PC_TCP.CALIB_FACTOR2)
//			|| (Modbus_Registers_PC_TCP_Write.CALIB_FACTOR3 != Modbus_Registers_PC_TCP.CALIB_FACTOR3)
//			|| (Modbus_Registers_PC_TCP_Write.CALIB_FACTOR4 != Modbus_Registers_PC_TCP.CALIB_FACTOR4)){
//
//		Modbus_Registers_Detector_Write.CALIB_FACTOR1 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR1;
//		Modbus_Registers_Detector_Write.CALIB_FACTOR2 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR2;
//		Modbus_Registers_Detector_Write.CALIB_FACTOR3 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR3;
//		Modbus_Registers_Detector_Write.CALIB_FACTOR4 = Modbus_Registers_PC_TCP_Write.CALIB_FACTOR4;
//
//		calib_factor_reconfigure = true;
//	}

	if(addr <= FREQ_DTC_SWITCH && (addr +(reg_cnt - 1)) >= FREQ_DTC_SWITCH){
		dtc_freq_switch_val = (Modbus_Registers_PC_TCP_Write.FREQ_DTC == 1) ? 1:0;
		Modbus_Registers_PC_TCP_Write.FREQ_DTC = 0;
		dtc_freq_switch = true;
	}

	//	if(Modbus_Registers_PC_TCP_Write.FREQ_DTC != Modbus_Registers_PC_TCP.FREQ_DTC)
	//	{
	//		dtc_freq_switch_val = (Modbus_Registers_PC_TCP_Write.FREQ_DTC == 1) ? 1:0;
	//		dtc_freq_switch = true;
	//	}

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
		dtc_freq_switch_val =  Modbus_Registers_PC_TCP.FREQ_DTC;

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

		g_1s_flags.rtc_poll = false;
	}
}

void Fill_Record(void)
{

    sd_logs.WRITE_COUNT_MSB = (uint16_t)(g_write_count >> 16);
    sd_logs.WRITE_COUNT_LSB = (uint16_t)(g_write_count & 0xFFFF);

    g_write_count++;

	sd_logs.RTC_DAY         = Modbus_Registers.RTC_DAY;
	sd_logs.RTC_MONTH       = Modbus_Registers.RTC_MONTH;
	sd_logs.RTC_YEAR        = Modbus_Registers.RTC_YEAR;
	sd_logs.RTC_HOUR        = Modbus_Registers.RTC_HOUR;
	sd_logs.RTC_MIN         = Modbus_Registers.RTC_MIN;
    sd_logs.RTC_SEC         = Modbus_Registers.RTC_SEC;
    sd_logs.Overload        = Modbus_Registers.Overload;
    sd_logs.Overrun         = Modbus_Registers.Overrun;
    sd_logs.Alarm           = Modbus_Registers.Alarm;
    sd_logs.Hv_Fault        = Modbus_Registers.Hv_Fault;
    sd_logs.Dtc_Fault       = Modbus_Registers.Dtc_Fault;
    sd_logs.CPS_MSB         = Modbus_Registers.CPS_MSB;
    sd_logs.CPS_LSB         = Modbus_Registers.CPS_LSB;
    sd_logs.mR_MSB          = Modbus_Registers.mR_MSB;
    sd_logs.mR_LSB          = Modbus_Registers.mR_LSB;
}

bool Modbus_Check_Ack(void)
{
    if(rx_index_PC_sd != 8)
        return false;

    if(rx_buffer_dma_PC_sd[0] != Modbus_Slave_Id_PC)
        return false;

    if(rx_buffer_dma_PC_sd[1] != 0x10)
        return false;

    return true;
}

bool Modbus_Send_Record_And_WaitAck(void)
{
    tx_complete_PC_sd = 0;
    tx_complete_PC_sd = 0;
    frame_recv_PC_sd  = 0;

    Modbus_FC_10_sd_rs485();

    uint32_t start = HAL_GetTick();

    while(!tx_complete_PC_sd)
    {
        if((HAL_GetTick() - start) > 150)
        {
            printf("TX timeout\r\n");
            tx_complete_PC_sd = 0;
            return false;
        }
    }

    start = HAL_GetTick();

    while(!frame_recv_PC_sd)
    {
        if((HAL_GetTick() - start) > 150)
        {
            printf("Frame Recv timeout\r\n");
            frame_recv_PC_sd = 0;
            return false;
        }
    }

    bool status = Modbus_Check_Ack();
    if(!status)
    {
    	printf("ACK Corrupt Data\n");
    }
    Modbus_Restart_RX_DMA_PC_SD();
    return true;
}

void Modbus_FC_10_sd_rs485(void)
{
	static uint8_t  frame_sd[128];
	uint8_t idx = 0;
	memset(frame_sd, 0, sizeof(frame_sd));
	uint8_t  slave_id   = Modbus_Slave_Id_PC;
	uint16_t start_addr = 0x2;
	uint16_t reg_count  = 0xF;
	uint16_t *regs      = (uint16_t*)&read_sd_logs;

	frame_sd[idx++] = slave_id;
	frame_sd[idx++] = 0x10;
	frame_sd[idx++] = (start_addr >> 8) & 0xFF;
	frame_sd[idx++] =  start_addr       & 0xFF;
	frame_sd[idx++] = (reg_count  >> 8) & 0xFF;
	frame_sd[idx++] =  reg_count        & 0xFF;
	frame_sd[idx++] =  reg_count * 2;

	for (int i = 0; i < reg_count; i++) {

		uint16_t value = regs[start_addr + i];
		frame_sd[idx++] = (value >> 8) & 0xFF;
		frame_sd[idx++] = value & 0xFF;
	}

	uint16_t crc = Modbus_CRC16(frame_sd, idx);
	frame_sd[idx++] = crc & 0xFF;
	frame_sd[idx++] = crc >> 8;

	RS485_TX_MODE_PC();
	HAL_UART_Transmit_DMA(&huart8, (uint8_t*)frame_sd, idx);
}

void Modbus_Restart_RX_DMA_PC_SD(void)
{
    HAL_UART_DMAStop(&huart8);
    __HAL_UART_CLEAR_OREFLAG(&huart8);
    __HAL_UART_FLUSH_DRREGISTER(&huart8);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, rx_buffer_dma_PC_sd,BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
}

float get_float_from_regs(uint16_t msb, uint16_t lsb)
{
    uint32_t val = ((uint32_t)msb << 16) | lsb;
    return ((float)val / 100.0f);
}

bool validate_ip(uint16_t msb, uint16_t lsb)
{
    uint8_t ip1 = (msb >> 8) & 0xFF;
    uint8_t ip2 = msb & 0xFF;
    uint8_t ip3 = (lsb >> 8) & 0xFF;
    uint8_t ip4 = lsb & 0xFF;

    if(ip1 > 255 || ip2 > 255 || ip3 > 255 || ip4 > 255){
        return false;
    }

    if(ip1 == 0 || ip1 == 255){
        return false;
    }

    return true;
}

bool validate_gateway(uint16_t msb, uint16_t lsb)
{
	uint8_t ip1 = (msb >> 8) & 0xFF;
	uint8_t ip2 = msb & 0xFF;
	uint8_t ip3 = (lsb >> 8) & 0xFF;
	uint8_t ip4 = lsb & 0xFF;

	if(ip1 > 255 || ip2 > 255 || ip3 > 255 || ip4 > 255){
		return false;
	}

	return true;
}

bool validate_subnet(uint16_t msb, uint16_t lsb)
{
    uint32_t subnet =
        ((uint32_t)((msb >> 8) & 0xFF) << 24) |
        ((uint32_t)(msb & 0xFF) << 16) |
        ((uint32_t)((lsb >> 8) & 0xFF) << 8) |
        (uint32_t)(lsb & 0xFF);

    switch(subnet)
    {
        case 0xFF000000: //255.0.0.0
        case 0xFFFF0000: //255.255.0.0
        case 0xFFFFFF00: //255.255.255.0
        case 0xFFFFFF80:
        case 0xFFFFFFC0:
        case 0xFFFFFFE0:
        case 0xFFFFFFF0:
        case 0xFFFFFFF8:
        case 0xFFFFFFFC:
            return true;

        default:
            return false;
    }
}

bool validate_port(uint16_t port)
{
    return (port >= 1 && port <= 65535);
}

void validate_modbus_write_rs485(void)
{
	float value;

	/* RTC */
	if(Modbus_Registers_Write.RTC_DAY < 1 || Modbus_Registers_Write.RTC_DAY > 31){
		Modbus_Registers_Write.RTC_DAY = Modbus_Registers.RTC_DAY;
	}

	if(Modbus_Registers_Write.RTC_MONTH < 1 || Modbus_Registers_Write.RTC_MONTH > 12){
		Modbus_Registers_Write.RTC_MONTH = Modbus_Registers.RTC_MONTH;
	}

	if(Modbus_Registers_Write.RTC_YEAR > 3000){
		Modbus_Registers_Write.RTC_YEAR = Modbus_Registers.RTC_YEAR;
	}

	if(Modbus_Registers_Write.RTC_HOUR > 23){
		Modbus_Registers_Write.RTC_HOUR = Modbus_Registers.RTC_HOUR;
	}

	if(Modbus_Registers_Write.RTC_MIN > 59){
		Modbus_Registers_Write.RTC_MIN = Modbus_Registers.RTC_MIN;
	}

	if(Modbus_Registers_Write.RTC_SEC > 59){
		Modbus_Registers_Write.RTC_SEC = Modbus_Registers.RTC_SEC;
	}

	/* READ_SD */
	if(Modbus_Registers_Write.READ_SD > 1){
		Modbus_Registers_Write.READ_SD = Modbus_Registers.READ_SD;
	}

	/* SD MODE */
	if(Modbus_Registers_Write.SD_MODE > 1){
		Modbus_Registers_Write.SD_MODE = Modbus_Registers.SD_MODE;
	}

	/* ACK */
	if(Modbus_Registers_Write.ACK > 1){
		Modbus_Registers_Write.ACK = Modbus_Registers.ACK;
	}

	/* RESET */
	if(Modbus_Registers_Write.Reset > 1){
		Modbus_Registers_Write.Reset = Modbus_Registers.Reset;
	}

	/* AUDIO MODE */
	if(Modbus_Registers_Write.AUDIO_MODE > 1){
		Modbus_Registers_Write.AUDIO_MODE = Modbus_Registers.AUDIO_MODE;
	}

	/* PRIMARY UNIT */
	if(Modbus_Registers_Write.Primary_Unit > 3){
		Modbus_Registers_Write.Primary_Unit = Modbus_Registers.Primary_Unit;
	}

	/* AGM MODE */
	if(Modbus_Registers_Write.AGM_MODE > 1){
		Modbus_Registers_Write.AGM_MODE  = Modbus_Registers.AGM_MODE;
	}

	/* FREQ FLAGS */
	if(Modbus_Registers_Write.FREQ_RECV > 1){
		Modbus_Registers_Write.FREQ_RECV = Modbus_Registers.FREQ_RECV;
	}

	if(Modbus_Registers_Write.FREQ_DTC > 1){
		Modbus_Registers_Write.FREQ_DTC = Modbus_Registers.FREQ_DTC;
	}

	/* ALARM VALUE <= 999999 */
	uint32_t val =  (Modbus_Registers_Write.Alarm_Value_MSB << 16) | (Modbus_Registers_Write.Alarm_Value_LSB);

	if(val > 999999U){
		Modbus_Registers_Write.Alarm_Value_MSB = Modbus_Registers.Alarm_Value_MSB;
		Modbus_Registers_Write.Alarm_Value_LSB = Modbus_Registers.Alarm_Value_LSB;
	}

	if(Modbus_Registers_Write.Alarm_Unit > 3){
		Modbus_Registers_Write.Alarm_Unit = Modbus_Registers.Alarm_Unit;
	}

	/* RS485 */
	if(Modbus_Registers_Write.RS485_Slave_Id > 999){
		Modbus_Registers_Write.RS485_Slave_Id = Modbus_Registers.RS485_Slave_Id;
	}

	if(Modbus_Registers_Write.RS485_Baud_Rate > 7){
		Modbus_Registers_Write.RS485_Baud_Rate = Modbus_Registers.RS485_Baud_Rate;
	}

	/* PASSWORD */
	uint32_t pass_val = (Modbus_Registers_Write.PASSWORD_MSB << 16) | (Modbus_Registers_Write.PASSWORD_LSB);

	if(pass_val > 9999U){
		Modbus_Registers_Write.PASSWORD_MSB = Modbus_Registers.PASSWORD_MSB;
		Modbus_Registers_Write.PASSWORD_LSB = Modbus_Registers.PASSWORD_LSB;
	}

	/* Ethernet */
	if(!validate_ip(Modbus_Registers_Write.Ethernet_IP_MSB,Modbus_Registers_Write.Ethernet_IP_LSB)){
		Modbus_Registers_Write.Ethernet_IP_MSB = Modbus_Registers.Ethernet_IP_MSB;
		Modbus_Registers_Write.Ethernet_IP_LSB = Modbus_Registers.Ethernet_IP_LSB;
	}

	if(!validate_subnet(Modbus_Registers_Write.Ethernet_Subnet_MSB,Modbus_Registers_Write.Ethernet_Subnet_LSB)){
		Modbus_Registers_Write.Ethernet_Subnet_MSB = Modbus_Registers.Ethernet_Subnet_MSB;
		Modbus_Registers_Write.Ethernet_Subnet_LSB = Modbus_Registers.Ethernet_Subnet_LSB;
	}

	if(!validate_gateway(Modbus_Registers_Write.Ethernet_Gateway_MSB,Modbus_Registers_Write.Ethernet_Gateway_LSB))
	{
		Modbus_Registers_Write.Ethernet_Gateway_MSB = Modbus_Registers.Ethernet_Gateway_MSB;
		Modbus_Registers_Write.Ethernet_Gateway_LSB = Modbus_Registers.Ethernet_Gateway_LSB;
	}

	if(Modbus_Registers_Write.Ethernet_Slave_Id > 247){
		Modbus_Registers_Write.Ethernet_Slave_Id = Modbus_Registers.Ethernet_Slave_Id;
	}

	if(!validate_port(Modbus_Registers_Write.Ethernet_Port)){
		Modbus_Registers_Write.Ethernet_Port = Modbus_Registers.Ethernet_Port;
	}

	/* PC IP */
	if(!validate_ip(Modbus_Registers_Write.Ethernet_IP_MSB_PC,Modbus_Registers_Write.Ethernet_IP_LSB_PC))
	{
		Modbus_Registers_Write.Ethernet_IP_MSB_PC = Modbus_Registers.Ethernet_IP_MSB_PC;
		Modbus_Registers_Write.Ethernet_IP_LSB_PC = Modbus_Registers.Ethernet_IP_LSB_PC;
	}

	if(!validate_port(Modbus_Registers_Write.Ethernet_Port_PC)){
		Modbus_Registers_Write.Ethernet_Port_PC = Modbus_Registers.Ethernet_Port_PC;
	}

	/* OVERLOAD */
	val = (Modbus_Registers_Write.Overload_MSB << 16) | (Modbus_Registers_Write.Overload_LSB);

	if(val > 999999U){
		Modbus_Registers_Write.Overload_MSB = Modbus_Registers.Overload_MSB;
		Modbus_Registers_Write.Overload_LSB = Modbus_Registers.Overload_LSB;
	}

	if(Modbus_Registers_Write.Overload_Unit > 3){
		Modbus_Registers_Write.Overload_Unit = Modbus_Registers.Overload_Unit;
	}

	/* OVERRANGE */
	val = (Modbus_Registers_Write.Overrange_MSB << 16) | (Modbus_Registers_Write.Overrange_LSB);

	if(val > 999999U){
		Modbus_Registers_Write.Overrange_MSB = Modbus_Registers.Overrange_MSB;
		Modbus_Registers_Write.Overrange_LSB = Modbus_Registers.Overrange_LSB;
	}

	if(Modbus_Registers_Write.Overrange_Unit > 3){
		Modbus_Registers_Write.Overrange_Unit = Modbus_Registers.Overrange_Unit;
	}

	/* ANALOG MIN */
	val = (Modbus_Registers_Write.Analog_4_to_20mA_Min_MSB << 16) | (Modbus_Registers_Write.Analog_4_to_20mA_Min_LSB);

	if(val > 999999U){
		Modbus_Registers_Write.Analog_4_to_20mA_Min_MSB = Modbus_Registers.Analog_4_to_20mA_Min_MSB;
		Modbus_Registers_Write.Analog_4_to_20mA_Min_LSB = Modbus_Registers.Analog_4_to_20mA_Min_LSB;
	}

	if(Modbus_Registers_Write.Analog_4_to_20mA_Min_Unit > 3){
		Modbus_Registers_Write.Analog_4_to_20mA_Min_Unit = Modbus_Registers.Analog_4_to_20mA_Min_Unit;
	}

	/* ANALOG MAX */
	val = (Modbus_Registers_Write.Analog_4_to_20mA_Max_MSB << 16) | (Modbus_Registers_Write.Analog_4_to_20mA_Max_LSB);

	if(val > 999999U){
		Modbus_Registers_Write.Analog_4_to_20mA_Max_MSB = Modbus_Registers.Analog_4_to_20mA_Max_MSB;
		Modbus_Registers_Write.Analog_4_to_20mA_Max_LSB = Modbus_Registers.Analog_4_to_20mA_Max_LSB;
	}

	if(Modbus_Registers_Write.Analog_4_to_20mA_Max_Unit > 3){
		Modbus_Registers_Write.Analog_4_to_20mA_Max_Unit = Modbus_Registers.Analog_4_to_20mA_Max_Unit;
	}

	/* HV */
	value = get_float_from_regs(
			Modbus_Registers_Write.HV_MSB,
			Modbus_Registers_Write.HV_LSB);

	if(value > 1500.0f){
		Modbus_Registers_Write.HV_MSB = Modbus_Registers.HV_MSB;
		Modbus_Registers_Write.HV_LSB = Modbus_Registers.HV_LSB;
	}

	/* Calibration */
	if(Modbus_Registers_Write.CALIB_FACTOR1 > 9999){
		Modbus_Registers_Write.CALIB_FACTOR1 = Modbus_Registers.CALIB_FACTOR1;
	}

	if(Modbus_Registers_Write.CALIB_FACTOR2 > 9999){
		Modbus_Registers_Write.CALIB_FACTOR2 = Modbus_Registers.CALIB_FACTOR2;
	}

	if(Modbus_Registers_Write.CALIB_FACTOR3 > 9999){
		Modbus_Registers_Write.CALIB_FACTOR3 = Modbus_Registers.CALIB_FACTOR3;
	}

	if(Modbus_Registers_Write.CALIB_FACTOR4 > 9999){
		Modbus_Registers_Write.CALIB_FACTOR4 = Modbus_Registers.CALIB_FACTOR4;
	}

}

void validate_modbus_write_tcp(void)
{
	float value;
    /* RTC */
    if(Modbus_Registers_PC_TCP_Write.RTC_DAY < 1 || Modbus_Registers_PC_TCP_Write.RTC_DAY > 31){
    	Modbus_Registers_PC_TCP_Write.RTC_DAY = Modbus_Registers_PC_TCP.RTC_DAY;
    }

    if(Modbus_Registers_PC_TCP_Write.RTC_MONTH < 1 || Modbus_Registers_PC_TCP_Write.RTC_MONTH > 12){
        Modbus_Registers_PC_TCP_Write.RTC_MONTH = Modbus_Registers_PC_TCP.RTC_MONTH;
    }

    if(Modbus_Registers_PC_TCP_Write.RTC_YEAR > 3000){
        Modbus_Registers_PC_TCP_Write.RTC_YEAR = Modbus_Registers_PC_TCP.RTC_YEAR;
    }

    if(Modbus_Registers_PC_TCP_Write.RTC_HOUR > 23){
        Modbus_Registers_PC_TCP_Write.RTC_HOUR = Modbus_Registers_PC_TCP.RTC_HOUR;
    }

    if(Modbus_Registers_PC_TCP_Write.RTC_MIN > 59){
        Modbus_Registers_PC_TCP_Write.RTC_MIN = Modbus_Registers_PC_TCP.RTC_MIN;
    }

    if(Modbus_Registers_PC_TCP_Write.RTC_SEC > 59){
        Modbus_Registers_PC_TCP_Write.RTC_SEC = Modbus_Registers_PC_TCP.RTC_SEC;
    }

    /* READ_SD */
    if(Modbus_Registers_PC_TCP_Write.READ_SD > 1){
        Modbus_Registers_PC_TCP_Write.READ_SD = Modbus_Registers_PC_TCP.READ_SD;
    }

    /* SD MODE */
    if(Modbus_Registers_PC_TCP_Write.SD_MODE > 1){
        Modbus_Registers_PC_TCP_Write.SD_MODE = Modbus_Registers_PC_TCP.SD_MODE;
    }

    /* ACK */
    if(Modbus_Registers_PC_TCP_Write.ACK > 1){
        Modbus_Registers_PC_TCP_Write.ACK = Modbus_Registers_PC_TCP.ACK;
    }

    /* RESET */
    if(Modbus_Registers_PC_TCP_Write.Reset > 1){
        Modbus_Registers_PC_TCP_Write.Reset = Modbus_Registers_PC_TCP.Reset;
    }

    /* AUDIO MODE */
    if(Modbus_Registers_PC_TCP_Write.AUDIO_MODE > 1){
        Modbus_Registers_PC_TCP_Write.AUDIO_MODE = Modbus_Registers_PC_TCP.AUDIO_MODE;
    }

    /* PRIMARY UNIT */
    if(Modbus_Registers_PC_TCP_Write.Primary_Unit > 3){
        Modbus_Registers_PC_TCP_Write.Primary_Unit = Modbus_Registers_PC_TCP.Primary_Unit;
    }

    /* AGM MODE */
    if(Modbus_Registers_PC_TCP_Write.AGM_MODE > 1){
       Modbus_Registers_PC_TCP_Write.AGM_MODE  = Modbus_Registers_PC_TCP.AGM_MODE;
    }

    /* FREQ FLAGS */
    if(Modbus_Registers_PC_TCP_Write.FREQ_RECV > 1){
       Modbus_Registers_PC_TCP_Write.FREQ_RECV = Modbus_Registers_PC_TCP.FREQ_RECV;
    }

    if(Modbus_Registers_PC_TCP_Write.FREQ_DTC > 1){
       Modbus_Registers_PC_TCP_Write.FREQ_DTC = Modbus_Registers_PC_TCP.FREQ_DTC;
    }

    /* ALARM VALUE <= 9999.99 */
    value = get_float_from_regs(
            Modbus_Registers_Write.Alarm_Value_MSB,
            Modbus_Registers_Write.Alarm_Value_LSB);

    if(value > 9999.99f){
        Modbus_Registers_PC_TCP_Write.Alarm_Value_MSB = Modbus_Registers_PC_TCP.Alarm_Value_MSB;
        Modbus_Registers_PC_TCP_Write.Alarm_Value_LSB = Modbus_Registers_PC_TCP.Alarm_Value_LSB;
    }

    if(Modbus_Registers_PC_TCP_Write.Alarm_Unit > 3){
        Modbus_Registers_PC_TCP_Write.Alarm_Unit = Modbus_Registers_PC_TCP.Alarm_Unit;
    }

    /* RS485 */
    if(Modbus_Registers_PC_TCP_Write.RS485_Slave_Id > 999){
       Modbus_Registers_PC_TCP_Write.RS485_Slave_Id = Modbus_Registers_PC_TCP.RS485_Slave_Id;
    }

    if(Modbus_Registers_PC_TCP_Write.RS485_Baud_Rate > 7){
       Modbus_Registers_PC_TCP_Write.RS485_Baud_Rate = Modbus_Registers_PC_TCP.RS485_Baud_Rate;
    }

    /* PASSWORD */
    uint32_t pass_val = (Modbus_Registers_PC_TCP_Write.PASSWORD_MSB << 16) | (Modbus_Registers_PC_TCP_Write.PASSWORD_LSB);

    if(pass_val > 9999){
        Modbus_Registers_PC_TCP_Write.PASSWORD_MSB = Modbus_Registers_PC_TCP.PASSWORD_MSB;
        Modbus_Registers_PC_TCP_Write.PASSWORD_LSB = Modbus_Registers_PC_TCP.PASSWORD_LSB;
    }

    /* Ethernet */
    if(!validate_ip(Modbus_Registers_PC_TCP_Write.Ethernet_IP_MSB,Modbus_Registers_PC_TCP_Write.Ethernet_IP_LSB))
    {
        Modbus_Registers_PC_TCP_Write.Ethernet_IP_MSB = Modbus_Registers_PC_TCP.Ethernet_IP_MSB;
        Modbus_Registers_PC_TCP_Write.Ethernet_IP_LSB = Modbus_Registers_PC_TCP.Ethernet_IP_LSB;
    }

    if(!validate_subnet(Modbus_Registers_PC_TCP_Write.Ethernet_Subnet_MSB,Modbus_Registers_PC_TCP_Write.Ethernet_Subnet_LSB))
    {
        Modbus_Registers_PC_TCP_Write.Ethernet_Subnet_MSB = Modbus_Registers_PC_TCP.Ethernet_Subnet_MSB;
        Modbus_Registers_PC_TCP_Write.Ethernet_Subnet_LSB = Modbus_Registers_PC_TCP.Ethernet_Subnet_LSB;
    }

    if(!validate_gateway(Modbus_Registers_PC_TCP_Write.Ethernet_Gateway_MSB,Modbus_Registers_PC_TCP_Write.Ethernet_Gateway_LSB))
    {
        Modbus_Registers_PC_TCP_Write.Ethernet_Gateway_MSB = Modbus_Registers_PC_TCP.Ethernet_Gateway_MSB;
        Modbus_Registers_PC_TCP_Write.Ethernet_Gateway_LSB = Modbus_Registers_PC_TCP.Ethernet_Gateway_LSB;
    }

    if(Modbus_Registers_PC_TCP_Write.Ethernet_Slave_Id > 247){
        Modbus_Registers_PC_TCP_Write.Ethernet_Slave_Id = Modbus_Registers_PC_TCP.Ethernet_Slave_Id;
    }

    if(!validate_port(Modbus_Registers_PC_TCP_Write.Ethernet_Port)){
        Modbus_Registers_PC_TCP_Write.Ethernet_Port = Modbus_Registers_PC_TCP.Ethernet_Port;
    }

    /* PC IP */
    if(!validate_ip(Modbus_Registers_PC_TCP_Write.Ethernet_IP_MSB_PC,Modbus_Registers_PC_TCP_Write.Ethernet_IP_LSB_PC))
    {
        Modbus_Registers_PC_TCP_Write.Ethernet_IP_MSB_PC = Modbus_Registers_PC_TCP.Ethernet_IP_MSB_PC;
        Modbus_Registers_PC_TCP_Write.Ethernet_IP_LSB_PC = Modbus_Registers_PC_TCP.Ethernet_IP_LSB_PC;
    }

    if(!validate_port(Modbus_Registers_PC_TCP_Write.Ethernet_Port_PC)){
        Modbus_Registers_PC_TCP_Write.Ethernet_Port_PC = Modbus_Registers_PC_TCP.Ethernet_Port_PC;
    }

    /* OVERLOAD */
    value = get_float_from_regs(Modbus_Registers_PC_TCP_Write.Overload_MSB,Modbus_Registers_PC_TCP_Write.Overload_LSB);

    if(value > 9999.99f){
        Modbus_Registers_PC_TCP_Write.Overload_MSB = Modbus_Registers_PC_TCP.Overload_MSB;
        Modbus_Registers_PC_TCP_Write.Overload_LSB = Modbus_Registers_PC_TCP.Overload_LSB;
    }

    if(Modbus_Registers_PC_TCP_Write.Overload_Unit > 3){
       Modbus_Registers_PC_TCP_Write.Overload_Unit = Modbus_Registers_PC_TCP.Overload_Unit;
    }

    /* OVERRANGE */
    value = get_float_from_regs(Modbus_Registers_PC_TCP_Write.Overrange_MSB,Modbus_Registers_PC_TCP_Write.Overrange_LSB);

    if(value > 9999.99f){
       Modbus_Registers_PC_TCP_Write.Overrange_MSB = Modbus_Registers_PC_TCP.Overrange_MSB;
       Modbus_Registers_PC_TCP_Write.Overrange_LSB = Modbus_Registers_PC_TCP.Overrange_LSB;
    }

    if(Modbus_Registers_PC_TCP_Write.Overrange_Unit > 3){
       Modbus_Registers_PC_TCP_Write.Overrange_Unit = Modbus_Registers_PC_TCP.Overrange_Unit;
    }

    /* ANALOG MIN */
    value = get_float_from_regs(Modbus_Registers_PC_TCP_Write.Analog_4_to_20mA_Min_MSB,Modbus_Registers_PC_TCP_Write.Analog_4_to_20mA_Min_LSB);

    if(value > 9999.99f){
       Modbus_Registers_PC_TCP_Write.Analog_4_to_20mA_Min_MSB = Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_MSB;
       Modbus_Registers_PC_TCP_Write.Analog_4_to_20mA_Min_LSB = Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_LSB;
    }

    if(Modbus_Registers_PC_TCP_Write.Analog_4_to_20mA_Min_Unit > 3){
       Modbus_Registers_PC_TCP_Write.Analog_4_to_20mA_Min_Unit = Modbus_Registers_PC_TCP.Analog_4_to_20mA_Min_Unit;
    }

    /* ANALOG MAX */
    value = get_float_from_regs(Modbus_Registers_PC_TCP_Write.Analog_4_to_20mA_Max_MSB,Modbus_Registers_PC_TCP_Write.Analog_4_to_20mA_Max_LSB);

    if(value > 9999.99f){
       Modbus_Registers_PC_TCP_Write.Analog_4_to_20mA_Max_MSB = Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_MSB;
       Modbus_Registers_PC_TCP_Write.Analog_4_to_20mA_Max_LSB = Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_LSB;
    }

    if(Modbus_Registers_Write.Analog_4_to_20mA_Max_Unit > 3){
       Modbus_Registers_PC_TCP_Write.Analog_4_to_20mA_Max_Unit = Modbus_Registers_PC_TCP.Analog_4_to_20mA_Max_Unit;
    }

    /* HV */
    value = get_float_from_regs(
            Modbus_Registers_PC_TCP_Write.HV_MSB,
            Modbus_Registers_PC_TCP_Write.HV_LSB);

    if(value > 1500.0f){
        Modbus_Registers_PC_TCP_Write.HV_MSB = Modbus_Registers_PC_TCP.HV_MSB;
        Modbus_Registers_PC_TCP_Write.HV_LSB = Modbus_Registers_PC_TCP.HV_LSB;
    }

    /* Calibration */
    if(Modbus_Registers_PC_TCP_Write.CALIB_FACTOR1 > 9999){
      Modbus_Registers_PC_TCP_Write.CALIB_FACTOR1 = Modbus_Registers_PC_TCP.CALIB_FACTOR1;
    }

    if(Modbus_Registers_PC_TCP_Write.CALIB_FACTOR2 > 9999){
      Modbus_Registers_PC_TCP_Write.CALIB_FACTOR2 = Modbus_Registers_PC_TCP.CALIB_FACTOR2;
    }

    if(Modbus_Registers_PC_TCP_Write.CALIB_FACTOR3 > 9999){
      Modbus_Registers_PC_TCP_Write.CALIB_FACTOR3 = Modbus_Registers_PC_TCP.CALIB_FACTOR3;
    }

    if(Modbus_Registers_PC_TCP_Write.CALIB_FACTOR4 > 9999){
       Modbus_Registers_PC_TCP_Write.CALIB_FACTOR4 = Modbus_Registers_PC_TCP.CALIB_FACTOR4;
    }

}

bool validate_modbus_write(master_modbus_db_t *db)
{
    float value;

    /* RTC */

    if(db->RTC_DAY < 1 || db->RTC_DAY > 31){
        return false;
    }

    if(db->RTC_MONTH < 1 || db->RTC_MONTH > 12){
        return false;
    }

    if(db->RTC_YEAR > 3000){
        return false;
    }

    if(db->RTC_HOUR > 23){
        return false;
    }

    if(db->RTC_MIN > 59){
        return false;
    }

    if(db->RTC_SEC > 59){
        return false;
    }

    /* READ_SD */

    if(db->READ_SD > 1){
        return false;
    }

    /* SD MODE */

    if(db->SD_MODE > 1){
        return false;
    }
    /* ACK */

    if(db->ACK > 1){
        return false;
    }

    /* RESET */

    if(db->Reset > 1){
        return false;
    }

    /* AUDIO MODE */

    if(db->AUDIO_MODE > 1){
        return false;
    }

    /* PRIMARY UNIT */

    if(db->Primary_Unit > 3){
        return false;
    }

    /* AGM MODE */

    if(db->AGM_MODE > 1){
        return false;
    }

    /* FREQ FLAGS */

    if(db->FREQ_RECV > 1){
        return false;
    }

    if(db->FREQ_DTC > 1){
        return false;
    }

    /* ALARM VALUE <= 9999.99 */

    value = get_float_from_regs(
            db->Alarm_Value_MSB,
            db->Alarm_Value_LSB);

    if(value > 9999.99f){
        return false;
    }

    if(db->Alarm_Unit > 3){
        return false;
    }

    /* RS485 */

    if(db->RS485_Slave_Id > 999){
        return false;
    }

    if(db->RS485_Baud_Rate > 7){
        return false;
    }

    /* PASSWORD */

    uint32_t pass_val = (db->PASSWORD_MSB << 16) | (db->PASSWORD_LSB);

    if(pass_val > 9999){
        return false;
    }

    /* Ethernet */

    if(!validate_ip(db->Ethernet_IP_MSB,db->Ethernet_IP_LSB)){
        return false;
    }

    if(!validate_subnet(db->Ethernet_Subnet_MSB,db->Ethernet_Subnet_LSB)){
        return false;
    }

    if(!validate_gateway(db->Ethernet_Gateway_MSB,db->Ethernet_Gateway_LSB))
    {
    	return false;
    }

    if(db->Ethernet_Slave_Id > 247){
        return false;
    }

    if(!validate_port(db->Ethernet_Port)){
        return false;
    }

    /* PC IP */

    if(!validate_ip(db->Ethernet_IP_MSB_PC, db->Ethernet_IP_LSB_PC)){
        return false;
    }

    if(!validate_port(db->Ethernet_Port_PC)){
        return false;
    }

    /* OVERLOAD */

    value = get_float_from_regs(db->Overload_MSB, db->Overload_LSB);

    if(value > 9999.99f){
        return false;
    }

    if(db->Overload_Unit > 3){
        return false;
    }

    /* OVERRANGE */

    value = get_float_from_regs(db->Overrange_MSB,db->Overrange_LSB);

    if(value > 9999.99f){
        return false;
    }

    if(db->Overrange_Unit > 3){
        return false;
    }

    /* ANALOG MIN */

    value = get_float_from_regs(db->Analog_4_to_20mA_Min_MSB,db->Analog_4_to_20mA_Min_LSB);

    if(value > 9999.99f){
        return false;
    }

    if(db->Analog_4_to_20mA_Min_Unit > 3){
        return false;
    }

    /* ANALOG MAX */

    value = get_float_from_regs(db->Analog_4_to_20mA_Max_MSB,db->Analog_4_to_20mA_Max_LSB);

    if(value > 9999.99f){
        return false;
    }

    if(db->Analog_4_to_20mA_Max_Unit > 3){
        return false;
    }

    /* HV */

    value = get_float_from_regs(
            db->HV_MSB,
            db->HV_LSB);

    if(value > 1500.0f){
        return false;
    }

    /* Calibration */

    if(db->CALIB_FACTOR1 > 9999){
        return false;
    }

    if(db->CALIB_FACTOR2 > 9999){
        return false;
    }

    if(db->CALIB_FACTOR3 > 9999){
        return false;
    }

    if(db->CALIB_FACTOR4 > 9999){
        return false;
    }

    return true;
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

//void config_CalibFactors_Variables(void)
//{
//	uint32_t k = Modbus_Registers.CALIB_FACTOR1;
//	uint32_t int_part  = k / 100;
//	uint32_t frac_part = k % 100;
//	if (frac_part == 0) {
//		sprintf((char *)Factor1_Value, "%lu", int_part);
//	} else if (frac_part % 10 == 0) {
//		sprintf((char *)Factor1_Value, "%lu.%01lu", int_part, frac_part / 10);
//	} else {
//		sprintf((char *)Factor1_Value, "%lu.%02lu", int_part, frac_part);
//	}
//
//	k = Modbus_Registers.CALIB_FACTOR2;
//	int_part  = k / 100;
//	frac_part = k % 100;
//	if (frac_part == 0) {
//		sprintf((char *)Factor2_Value, "%lu", int_part);
//	} else if (frac_part % 10 == 0) {
//		sprintf((char *)Factor2_Value, "%lu.%01lu", int_part, frac_part / 10);
//	} else {
//		sprintf((char *)Factor2_Value, "%lu.%02lu", int_part, frac_part);
//	}
//
//	k = Modbus_Registers.CALIB_FACTOR3;
//	int_part  = k / 100;
//	frac_part = k % 100;
//	if (frac_part == 0) {
//		sprintf((char *)Factor3_Value, "%lu", int_part);
//	} else if (frac_part % 10 == 0) {
//		sprintf((char *)Factor3_Value, "%lu.%01lu", int_part, frac_part / 10);
//	} else {
//		sprintf((char *)Factor3_Value, "%lu.%02lu", int_part, frac_part);
//	}
//
//	k =  Modbus_Registers.CALIB_FACTOR4;
//	int_part  = k / 100;
//	frac_part = k % 100;
//	if (frac_part == 0) {
//		sprintf((char *)Factor4_Value, "%lu", int_part);
//	} else if (frac_part % 10 == 0) {
//		sprintf((char *)Factor4_Value, "%lu.%01lu", int_part, frac_part / 10);
//	} else {
//		sprintf((char *)Factor4_Value, "%lu.%02lu", int_part, frac_part);
//	}
//}
//
//double convert_cps_to_mr_h()
//{
//	double dose_mr,factor;
//	dose_mr = (float) CPS_VAL / GM_TUBE_SENSITIVITY;
//	if (dose_mr <= (MR_25_VAL * atof(Factor1_Value)))
//	{
//		dose_mr = dose_mr / atof(Factor1_Value);
//	}
//	else if (dose_mr <= (MR_50_VAL * atof(Factor2_Value)))
//	{
//		double raw_25  = (MR_25_VAL * atof(Factor1_Value));
//		double raw_50 = (MR_50_VAL * atof(Factor2_Value));
//
//		double f1 = 1/atof(Factor1_Value);
//		double f2 = 1/atof(Factor2_Value);
//		double r1 = (double)dose_mr - (double)raw_25;
//		double sub = (f2-f1);
//		double r2 = (raw_50 - raw_25);
//		factor = (f1+((r1*sub)/r2));
//
//		dose_mr = dose_mr * factor;
//	}
//	else if (dose_mr <= (MR_75_VAL * atof(Factor3_Value)))
//	{
//		double raw_50  = (MR_50_VAL * atof(Factor2_Value));
//		double raw_75 = (MR_75_VAL * atof(Factor3_Value));
//
//		double f1 = 1/atof(Factor2_Value);
//		double f2 = 1/atof(Factor3_Value);
//		double r1 = (double)dose_mr - (double)raw_50;
//		double sub = (f2-f1);
//		double r2 = (raw_75 - raw_50);
//		factor = (f1+((r1*sub)/r2));
//
//		dose_mr = dose_mr * factor;
//	}
//	else
//	{
//		dose_mr = dose_mr / atof(Factor4_Value);
//	}
//	return dose_mr;
//}

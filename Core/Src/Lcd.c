/*
 * Lcd.c
 *
 *  Created on: Jan 27, 2026
 *      Author: Ajit Bhosle
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "main_global.h"
#include "Lcd.h"
#include "AT24CM01_Eeprom.h"
#include "modbus.h"
#include "Fault_Handler.h"
#include "pc_modbus.h"
#include "tcp_server_registers.h"
#include "Detector_Modbus_registers.h"
#include "App_7_segment.h"
#include "DAC_Drvr.h"

#define EPSILON             0.0001f
#define SYSTEM_MENU_OPTION  20

typedef enum {
    CALIBRATION_MENU,
    CALIBRATION_FACTOR_EDIT
} calibration_screen_state_t;

typedef enum
{
    ETH_STATE_MENU = 0,
    ETH_STATE_SLAVE_ID,
    ETH_STATE_PORT,
    ETH_STATE_IP,
    ETH_STATE_SUBNET,
    ETH_STATE_GATEWAY,
	ETH_STATE_NONE
} eth_state_t;

typedef enum
{
    ETH_MENU_SLAVE_ID = 0,
    ETH_MENU_PORT,
    ETH_MENU_IP,
    ETH_MENU_SUBNET,
    ETH_MENU_GATEWAY,
    ETH_MENU_EXIT,
    ETH_MENU_MAX
} eth_menu_t;

typedef enum
{
  rs485_state_menu=0,
  rs485_state_slave_id,
  rs485_state_baud_rate,
  rs485_state_none,
}rs485_state_t;

typedef enum
{
   rs485_menu_slave_id=0,
   rs485_menu_baud_rate,
   rs485_menu_exit,
   rs485_menu_max
}rs485_menu_t;

typedef enum
{
	RTC_STATE_MENU = 0,
	RTC_STATE_DATE,
	RTC_STATE_TIME,
	RTC_STATE_NONE
}rtc_state_t;

typedef enum
{
    RTC_MENU_DATE=0,
	RTC_MENU_TIME,
	RTC_MENU_EXIT,
	RTC_MENU_MAX,
}rtc_menu_t;

char *eth_menu_str[] =
{
    "1. Slave ID",
    "2. Port",
    "3. IP",
    "4. Subnet",
    "5. Gateway",
    "6. Exit"
};

char *rs485_menu_str[] =
{
	"1. Slave ID",
	"2. Baud Rate",
	"3. Exit"
};

char *rtc_menu_str[] =
{
	"1. Date",
	"2. Time",
	"3. Exit"
};

eth_state_t eth_state = ETH_STATE_MENU;
eth_state_t prev_eth_state = ETH_STATE_NONE;

eth_menu_t eth_menu = ETH_MENU_SLAVE_ID;
eth_menu_t prev_eth_menu = ETH_MENU_MAX;

rs485_menu_t rs485_menu = rs485_menu_slave_id;
rs485_menu_t prev_rs485_menu = rs485_menu_max;

rs485_state_t rs485_state = rs485_state_menu;
rs485_state_t prev_rs485_state = rs485_state_none;

rtc_menu_t rtc_menu =  RTC_MENU_DATE;
rtc_menu_t prev_rtc_menu = RTC_MENU_MAX;

rtc_state_t rtc_state = RTC_STATE_MENU;
rtc_state_t prev_rtc_state = RTC_STATE_NONE;

System_Screen_t lcd_screen = Home_Screen;

SystemStateFlag last_system_state = (SystemStateFlag)0xFF;

bool sw2_press_flag = false;
bool sw3_press_flag = false;
bool sw4_press_flag = false;
bool sw1_press_flag = false;

float hv_val_dtc = 0.0f;
bool write_hv_val_dtc = false;

uint8_t edit_pos_index = 0;
uint8_t edit_pos_index_hv = 0;
const uint8_t editable_indices_hv[] = {0, 1, 2, 3, 5, 6};
const uint8_t editable_indices[] = {0,1,2,3,5,6,8};
static const uint8_t editable_indices_decimal[] = {0, 1, 2, 3, 5, 6, 8};
static const uint8_t editable_indices_integer[] = {0, 1, 2, 3, 4,    8};
const uint8_t cal_editable_indices[] = {0,1,3,4};

System_Screen_t prev_lcd_screen = 0xff;

uint16_t last_unit = 0xFFFF;
uint8_t pwd[4] = {0, 0, 0, 0};
uint8_t pwd_pos = 0;

uint8_t menu_options = 0;
uint8_t prev_menu_options = 0;

uint8_t temp_primary_unit = 0;
uint8_t temp_baud_rate=0;
char    temp_primary_unit_str[12];
char    temp_baud_rate_str[12];
char    prev_temp_primary_unit_str[12]="";
char    prev_temp_baud_rate_str[12]="";
uint8_t temp_alarm_unit = 0;
uint8_t prev_temp_alarm_unit = 0xff;
char    prev_temp_hv_write[8] = "";
char    temp_hv_write[8] = "0000.00";
char    temp_alarm_val[8] = "0000.00";
char    prev_temp_alarm_val[8]="";
uint8_t temp_overrange_unit = 0;
uint8_t prev_temp_overrange_unit = 0xff;
char    temp_overrange_val[8] = "0000.00";
char    prev_temp_overrange_val[8]="";
uint8_t temp_overload_unit = 0;
uint8_t prev_temp_overload_unit = 0xff;
char    temp_overload_val[8] = "0000.00";
char    prev_temp_overload_val[8]="";
uint8_t temp_4_20_min_unit = 0;
uint8_t prev_temp_4_20_min_unit = 0xff;
char    temp_4_20_min_val[8] = "0000.00";
char    prev_temp_4_20_min_val[8]="";
uint8_t temp_4_20_max_unit = 0;
uint8_t prev_temp_4_20_max_unit = 0xff;
char    temp_4_20_max_val[8] = "0000.00";
char    prev_temp_4_20_max_val[8]="";
char    prev_temp_4_20_min_cal_val[8]="";
char    temp_4_20_min_cal_val[8]="00.0000";
char    prev_temp_4_20_max_cal_val[8]="";
char    temp_4_20_max_cal_val[8]="00.0000";
char    temp_agm_mode_str[12] = "1. Manual";
char    prev_temp_agm_mode_str[12] ="";
char    temp_save_setting_str[12] = "1. Yes";
char    prev_temp_save_setting_str[12]="";

uint8_t temp_agm_mode = 0;
uint8_t temp_save_setting_mode = 0;

char temp_read_sd_str[8] = "1. No";
char prev_temp_read_sd_str[8]="";
uint8_t temp_sd_mode = 0;
uint8_t sd_mode = 0;

char temp_audio_mode_str[12] = "1. Disable";
char prev_temp_audio_mode_str[12]="";
uint8_t temp_audio_mode = 0;
uint8_t audio_mode = 0;

char temp_freq_recv_str[12] = "1. Disable";
char prev_temp_freq_recv_str[12] = "";
uint8_t temp_freq_recv_mode = 0;
uint8_t freq_recv_mode = 0;

char temp_freq_dtc_str[12] = "1. Disable";
char prev_temp_freq_dtc_str[12] = "";
uint8_t temp_freq_dtc_mode = 0;
uint8_t freq_dtc_mode = 0;

char temp_password[5] = "1234";
char correct_password[5] = "1234";
char entered_password[5];

static char* Unit_Str_Options[]={"mR/h ","uSv/h  ","cps  ","cpm  "};

// ------------------- ETHERNET SCREEN -------------------------------
char eth_slave_id[4]   = "001";
char rs485_slave_id[4] = "001";
char eth_port[5]       = "0000";
char eth_ip[16]        = "192.168.001.010";
char eth_subnet[16]    = "255.255.255.000";
char eth_gateway[16]   = "192.168.001.001";

uint8_t eth_num_cursor = 0;
uint8_t num_cursor = 0;
uint8_t eth_ip_cursor  = 0;
char prev_val[10] = "";
char prev_ip[20] = "";

// ------------------ CALIBRATION SCREEN ------------------------------
calibration_screen_state_t calibration_state = CALIBRATION_MENU;
uint8_t calibration_menu_index = 0;
uint8_t prev_calibration_menu_index = 0xff;
char factor_values[4][6] = {0};
char temp_factor_val[6]= "00.00";
char prev_temp_factor_val[6] = "";
uint8_t current_factor_index = 0;

// -----------------------SAVED VALUES-----------------------------------
uint16_t primary_unit = 0;
uint16_t last_primary_unit = 0xFFFF;
char saved_rs485_slave_id[6] = "001";
uint8_t baud_rate=0;
char primary_unit_str[12];
char baud_rate_str[12];
uint8_t alarm_unit = 0;
char alarm_val[8] = "0000.00";
uint8_t overrange_unit = 0;
char overrange_val[8] = "0000.00";
uint8_t overload_unit = 0;
char overload_val[8] = "0000.00";
uint8_t min_4_20_unit = 0;
char min_4_20_val[8] = "0000.00";
uint8_t max_4_20_unit = 0;
char max_4_20_val[8] = "0000.00";
uint8_t agm_mode = 0;

// ---------------Saved ETH---------------------
char saved_eth_slave_id[6] = "001";
char saved_eth_port[6]     = "0000";
char saved_eth_ip[16]      = "192.168.001.010";
char saved_eth_subnet[16]  = "255.255.255.000";
char saved_eth_gateway[16] = "192.168.001.001";

// --------------Saved Calibration--------------
char saved_factor_values[4][6] = {"00.00","00.00","00.00","00.00"};
char cal_4mA_val[8]  = "00.0000";
char cal_20mA_val[8] = "00.0000";

static uint32_t last_tick_time_rhv = 0;

char  temp_rtc_date_val[11] = "00.00.0000";
char  rtc_date_val[11] = "00.00.0000";
char  prev_temp_rtc_date_val[11]="";
char  temp_rtc_time_val[9] = "00.00.00";
char  rtc_time_val[9] = "00.00.00";
char  prev_temp_rtc_time_val[9]="";
uint8_t rtc_cursor = 0;

uint8_t unit_is_integer(uint8_t unit)
{
	return unit == 2 || unit == 3;
}

static const uint8_t* get_index_table(uint8_t unit, uint8_t *out_len)
{
    if (unit_is_integer(unit)) { *out_len = 6; return editable_indices_integer; }
    else                       { *out_len = 7; return editable_indices_decimal; }
}

bool check_calib_fact_change(void)
{
	float saved_vals[4] = {
			atof(Factor1_Value),
			atof(Factor2_Value),
			atof(Factor3_Value),
			atof(Factor4_Value)
	};

	float current_vals[4] = {
			atof(factor_values[0]),
			atof(factor_values[1]),
			atof(factor_values[2]),
			atof(factor_values[3])
	};

	bool changed = false;

	for(int i = 0; i < 4; i++)
	{
	    if(fabs(saved_vals[i] - current_vals[i]) > EPSILON)
	    {
	        changed = true;
	        break;
	    }
	}
	return changed;
}

void calib_factors_hv_write(void)
{
	bool is_freq_edit = false;
	bool is_calib_edit = false;
	bool is_hv_edit = false;
    bool calib_factor_change = false;

    calib_factor_change = check_calib_fact_change();

	if(calib_factor_change == true)
	{
		uint16_t scaled_val;
		scaled_val = (uint16_t)(atof(factor_values[0])*100);
		Modbus_Registers_Detector_Write.CALIB_FACTOR1 = scaled_val;
		scaled_val =  (uint16_t)(atof(factor_values[1])*100);
		Modbus_Registers_Detector_Write.CALIB_FACTOR2 = scaled_val;
		scaled_val =  (uint16_t)(atof(factor_values[2])*100);
		Modbus_Registers_Detector_Write.CALIB_FACTOR3 = scaled_val;
		scaled_val =  (uint16_t)(atof(factor_values[3])*100);
        Modbus_Registers_Detector_Write.CALIB_FACTOR4 = scaled_val;

		is_calib_edit = true;
		calib_factor_change = false;
	}

	if(hv_val_dtc > 0.0f)
	{
		uint32_t scaled_val = hv_val_dtc * 100;
		Modbus_Registers_Detector_Write.HV_MSB = (scaled_val >> 16 ) & 0xFFFF;
		Modbus_Registers_Detector_Write.HV_LSB = (scaled_val & 0xFFFF);

		is_hv_edit = true;
	}

	if(freq_dtc_mode  != temp_freq_dtc_mode)
	{
        dtc_freq_switch_val = (temp_freq_dtc_mode == 1) ? 1:0;
		is_freq_edit = true;
	}

	if(is_calib_edit == true && is_hv_edit == false && is_freq_edit == false)
	{
		modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = CALIB_WRITE_FLAG;
		is_calib_edit = false;
	}
	else if(is_calib_edit == false && is_hv_edit == true && is_freq_edit == false)
	{
		modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = HV_WRITE_FLAG;
		is_hv_edit = false;
	}
	else if(is_calib_edit == false &&  is_hv_edit == false && is_freq_edit == true)
	{
		modbus_write_flag = MODBUS_SINGLE_WRITE_FLAG;
		single_write_flag = DTC_FREQ_SWITCH_FLAG;
		is_freq_edit = false;
	}
	else if(is_calib_edit == true && is_hv_edit == true && is_freq_edit == false)
	{
		//Need to Update FREQ Switch
		dtc_freq_switch_val =  freq_dtc_mode;

	    modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
	    multiple_write_flag = HV_AND_CALIB_WRITE_FLAG;
	    is_calib_edit = false;
	    is_hv_edit = false;
	}
	else if(is_calib_edit == false && is_hv_edit == true && is_freq_edit == true)
	{
		modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
        multiple_write_flag = HV_AND_FREQ_WRITE_FLAG;
		is_hv_edit = false;
		is_freq_edit = false;
	}
	else if(is_calib_edit ==  true && is_hv_edit == false && is_freq_edit == true)
	{
		modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = ALL_CONFIG_WRITE_FLAG;
		is_calib_edit = false;
		is_hv_edit = false;
		is_freq_edit = false;
	}
	else if(is_calib_edit == true && is_hv_edit == true && is_freq_edit == true){
		modbus_write_flag = MODBUS_MULTIPLE_WRITE_FLAG;
		multiple_write_flag = ALL_CONFIG_WRITE_FLAG;
		is_calib_edit = false;
		is_hv_edit = false;
		is_freq_edit = false;
	}
}

void verify_tcp_rs485_conf(void)
{
	if(baud_rate != temp_baud_rate)
	{
		rs485_reconfig_required = true;
	}

	if(strcmp(eth_slave_id,saved_eth_slave_id) != 0 || strcmp(eth_port,saved_eth_port) !=0 || strcmp(saved_eth_ip,eth_ip) !=0
			|| strcmp(saved_eth_subnet , eth_subnet) != 0 || strcmp(saved_eth_gateway,eth_gateway))
	{
		tcp_reconfig_required = true;
	}

}

void dac_config(void){
	float a = atof(cal_4mA_val);
	float b = atof(cal_20mA_val);
	Two_Point_Calibrate_DAC(a,b);
}

bool Validate_Time_String()
{
    uint8_t hour, min, sec;
	hour   = (temp_rtc_time_val[0]-'0')*10 + (temp_rtc_time_val[1]-'0');
	min    = (temp_rtc_time_val[3]-'0')*10 + (temp_rtc_time_val[4]-'0');
	sec    = (temp_rtc_time_val[6]-'0')*10 + (temp_rtc_time_val[7]-'0');

    if (hour > 23)
        return false;

    if (min > 59)
        return false;

    if (sec > 59)
        return false;

    return true;
}

bool IsLeapYear(uint16_t year)
{
    return ((year % 4 == 0 && year % 100 != 0) ||
            (year % 400 == 0));
}

bool Validate_Date_String()
{
    uint8_t day, month;
    uint16_t year;

    day =
        (temp_rtc_date_val[0]-'0')*10 +
        (temp_rtc_date_val[1]-'0');

    month =
        (temp_rtc_date_val[3]-'0')*10 +
        (temp_rtc_date_val[4]-'0');

    year =
        (temp_rtc_date_val[6]-'0')*1000 +
        (temp_rtc_date_val[7]-'0')*100 +
        (temp_rtc_date_val[8]-'0')*10 +
        (temp_rtc_date_val[9]-'0');

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

uint8_t Get_RTC_WeekDay(uint8_t day, uint8_t month, uint16_t year)
{
    if(month < 3)
    {
        month += 12;
        year--;
    }

    uint16_t K = year % 100;
    uint16_t J = year / 100;

    uint8_t h = (day + (13 * (month + 1)) / 5 +
                 K + K / 4 + J / 4 + 5 * J) % 7;
    switch(h)
    {
        case 0: return RTC_WEEKDAY_SATURDAY;
        case 1: return RTC_WEEKDAY_SUNDAY;
        case 2: return RTC_WEEKDAY_MONDAY;
        case 3: return RTC_WEEKDAY_TUESDAY;
        case 4: return RTC_WEEKDAY_WEDNESDAY;
        case 5: return RTC_WEEKDAY_THURSDAY;
        case 6: return RTC_WEEKDAY_FRIDAY;
    }

    return RTC_WEEKDAY_MONDAY;
}

void config_rtc(void)
{
	bool time_changed = false;
	bool date_changed = false;

	if(strcmp(rtc_date_val,temp_rtc_date_val)!=0)
	{
		date_changed = true;
	}

	if(strcmp(rtc_time_val,temp_rtc_time_val)!=0)
	{
		time_changed = true;
	}

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	if(time_changed)
	{
		sTime.Hours   = (temp_rtc_time_val[0]-'0')*10 + (temp_rtc_time_val[1]-'0');
		sTime.Minutes = (temp_rtc_time_val[3]-'0')*10 + (temp_rtc_time_val[4]-'0');
		sTime.Seconds = (temp_rtc_time_val[6]-'0')*10 + (temp_rtc_time_val[7]-'0');
	}

	if(date_changed)
	{
	    sDate.Date  = (temp_rtc_date_val[0]-'0')*10 + (temp_rtc_date_val[1]-'0');
	    sDate.Month = (temp_rtc_date_val[3]-'0')*10 + (temp_rtc_date_val[4]-'0');

	    uint16_t year =
	            (temp_rtc_date_val[6]-'0')*1000 +
	            (temp_rtc_date_val[7]-'0')*100 +
	            (temp_rtc_date_val[8]-'0')*10 +
	            (temp_rtc_date_val[9]-'0');

	    sDate.Year = year - 2000;

	    sDate.WeekDay = Get_RTC_WeekDay(
	                        sDate.Date,
	                        sDate.Month,
	                        year);
	}

	if(date_changed && Validate_Date_String())
	{
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	}

	if(time_changed && Validate_Time_String())
	{
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	}
}

void set_configuration(void)
{
	config_rtc();
	calib_factors_hv_write();
	verify_tcp_rs485_conf();
	copy_temp_to_saved_values();
	dac_config();
	Update_Registers();
	cpy_reg_to_eeprom_reg();
	Save_Config_To_Eeprom();
	conf_rs485_tcp();
	if(temp_sd_mode == 1)
	{
		read_sd_flag = true;
	}
}

// --------------Saved Temp Variables-------------
void copy_temp_to_saved_values(void)
{
	hv_val_dtc   = atof(temp_hv_write);

	audio_mode = temp_audio_mode;
	agm_mode = temp_agm_mode;
	primary_unit = temp_primary_unit;
	freq_recv_mode = temp_freq_recv_mode;
	freq_dtc_mode  = temp_freq_dtc_mode;

	if(strcmp(rs485_slave_id,saved_rs485_slave_id)!=0)
	{
		Modbus_Slave_Id_PC = atoi(rs485_slave_id);
	}
	strcpy(saved_rs485_slave_id,rs485_slave_id);

	baud_rate = temp_baud_rate;

	alarm_unit = temp_alarm_unit;
	strcpy(alarm_val,temp_alarm_val);
	overrange_unit = temp_overrange_unit;
	strcpy(overrange_val,temp_overrange_val);
	overload_unit = temp_overload_unit;
	strcpy(overload_val,temp_overload_val);
	min_4_20_unit = temp_4_20_min_unit;
	strcpy(min_4_20_val,temp_4_20_min_val);
	max_4_20_unit = temp_4_20_max_unit;
	strcpy(max_4_20_val,temp_4_20_max_val);

	strcpy(saved_eth_slave_id,eth_slave_id);
	strcpy(saved_eth_port,eth_port);
	strcpy(saved_eth_ip,eth_ip);
	strcpy(saved_eth_subnet,eth_subnet);
	strcpy(saved_eth_gateway,eth_gateway);

	//  ---------- Passsword Saved--------------
	strcpy(correct_password,temp_password);

	strcpy(cal_4mA_val,temp_4_20_min_cal_val);
	strcpy(cal_20mA_val,temp_4_20_max_cal_val);
}

// --------------Copy To Temp Variables-------------
void copy_saved_values_to_temp(void)
{
	strcpy(temp_hv_write, "0000.00");

	strcpy(temp_4_20_min_cal_val,"00.0000");
	strcpy(temp_4_20_max_cal_val,"00.0000");

	temp_audio_mode = audio_mode;
	temp_freq_recv_mode = freq_recv_mode;
	temp_freq_dtc_mode = freq_dtc_mode;
	temp_agm_mode = agm_mode;
	temp_primary_unit = primary_unit;
	strcpy(rs485_slave_id,saved_rs485_slave_id);
	temp_baud_rate = baud_rate;
	temp_alarm_unit = alarm_unit;
	strcpy(temp_alarm_val,alarm_val);
	temp_overrange_unit = overrange_unit;
	strcpy(temp_overrange_val,overrange_val);
	temp_overload_unit = overload_unit;
	strcpy(temp_overload_val,overload_val);
	temp_4_20_min_unit = min_4_20_unit;
	strcpy(temp_4_20_min_val,min_4_20_val);
	temp_4_20_max_unit = max_4_20_unit;
	strcpy(temp_4_20_max_val,max_4_20_val);
	if(agm_mode == 0){
		strcpy(temp_agm_mode_str,"1. Manual");
	}
	else{
		strcpy(temp_agm_mode_str,"2. Auto");
	}
	strcpy(eth_slave_id,saved_eth_slave_id);
	strcpy(eth_port,saved_eth_port);
	strcpy(eth_ip,saved_eth_ip);
	strcpy(eth_subnet,saved_eth_subnet);
	strcpy(eth_gateway,saved_eth_gateway);

	sprintf(factor_values[0], "%05.2f", atof(Factor1_Value));
	sprintf(factor_values[1], "%05.2f", atof(Factor2_Value));
	sprintf(factor_values[2], "%05.2f", atof(Factor3_Value));
	sprintf(factor_values[3], "%05.2f", atof(Factor4_Value));

	//  ---------- Retain Passsword --------------
	strcpy(temp_password,correct_password);

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	snprintf(temp_rtc_time_val,
	         sizeof(temp_rtc_time_val),
	         "%02d.%02d.%02d",
	         sTime.Hours,
	         sTime.Minutes,
	         sTime.Seconds);
	snprintf(rtc_time_val,
		         sizeof(rtc_time_val),
		         "%02d.%02d.%02d",
		         sTime.Hours,
		         sTime.Minutes,
		         sTime.Seconds);

	snprintf(temp_rtc_date_val, sizeof(temp_rtc_date_val),
	         "%02d.%02d.%04d",
	         sDate.Date, sDate.Month, 2000 + sDate.Year);

	snprintf(rtc_date_val, sizeof(rtc_date_val),
	         "%02d.%02d.%04d",
	         sDate.Date, sDate.Month, 2000 + sDate.Year);

	temp_sd_mode = sd_mode;
}

void select_hv_val(char* val)
{
	if(sw1_press_flag == true)
	{
		sw1_press_flag = false;
		uint8_t str_index = editable_indices_hv[edit_pos_index_hv];
		if(val[str_index] == '9')
		{
			val[str_index] = '0';
		}
		else{
			val[str_index]++;
		}
	}

	if(sw2_press_flag == true)
	{
		sw2_press_flag = false;
		uint8_t str_index = editable_indices_hv[edit_pos_index_hv];
		if(val[str_index] == '0')
		{
			val[str_index] = '9';
		}
		else{
			val[str_index]--;
		}
	}

	if(sw4_press_flag == true)
	{
		sw4_press_flag = false;
		edit_pos_index_hv = (edit_pos_index_hv + 1) % 6;
	}

	uint8_t lcd_col=editable_indices_hv[edit_pos_index_hv];
	lcd_set_cursor(1, lcd_col+4);
	lcd_cursor_on();
}

void lcd_freq_dtc_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    DTC FREQ   ");
	lcd_set_cursor(1,3);
}

void lcd_freq_dtc_screen(void)
{
	if(prev_lcd_screen != System_Freq_Dtc_Screen)
	{
		prev_lcd_screen = System_Freq_Dtc_Screen;
		lcd_freq_dtc_screen_init();
		prev_temp_freq_dtc_str[0] = '\0';
	}
	check_switch_status();

	if(sw1_press_flag == true || sw2_press_flag == true)
	{
		sw1_press_flag = false;
		sw2_press_flag = false;
		temp_freq_dtc_mode ^= 1;
		strcpy(temp_freq_dtc_str,
				temp_freq_dtc_mode ? "2. Enable" : "1. Disable");
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = System_Freq_Dtc_Screen;
		lcd_screen = Setup_Menu_Screen;
	}

	if(strcmp(temp_freq_dtc_str,prev_temp_freq_dtc_str)!=0)
	{
		lcd_set_cursor(1,3);
		lcd_print("                ");
		lcd_set_cursor(1,3);
		lcd_print(temp_freq_dtc_str);
		strcpy(prev_temp_freq_dtc_str,temp_freq_dtc_str);
	}
}

void lcd_freq_recv_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    RECV FREQ   ");
	lcd_set_cursor(1,3);
}

void lcd_freq_recv_screen(void)
{
	if(prev_lcd_screen != System_Freq_Recv_Screen)
	{
		prev_lcd_screen = System_Freq_Recv_Screen;
		lcd_freq_recv_screen_init();
		prev_temp_freq_recv_str[0] = '\0';
	}
	check_switch_status();

	if(sw1_press_flag == true || sw2_press_flag == true)
	{
		sw1_press_flag = false;
		sw2_press_flag = false;
		temp_freq_recv_mode ^= 1;
		strcpy(temp_freq_recv_str,
				temp_freq_recv_mode ? "2. Enable" : "1. Disable");
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = System_Freq_Recv_Screen;
		lcd_screen = Setup_Menu_Screen;
	}

	if(strcmp(temp_freq_recv_str,prev_temp_freq_recv_str)!=0)
	{
		lcd_set_cursor(1,3);
		lcd_print("                ");
		lcd_set_cursor(1,3);
		lcd_print(temp_freq_recv_str);
		strcpy(prev_temp_freq_recv_str,temp_freq_recv_str);
	}
}

void lcd_read_sd_data_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    READ   ");
	lcd_set_cursor(1,4);
}

void lcd_read_sd_data(void)
{
	if(prev_lcd_screen != System_Read_Data)
	{
		prev_lcd_screen = System_Read_Data;
		lcd_read_sd_data_init();
		prev_temp_read_sd_str[0] = '\0';
	}
	check_switch_status();

	if(sw1_press_flag == true || sw2_press_flag == true)
	{
		sw1_press_flag = false;
		sw2_press_flag = false;
		temp_sd_mode ^= 1;
		strcpy(temp_read_sd_str,
				temp_sd_mode ? "2. Yes" : "1. No");
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = System_Read_Data;
		lcd_screen = Setup_Menu_Screen;
	}

	if(strcmp(temp_read_sd_str,prev_temp_read_sd_str)!=0)
	{
		lcd_set_cursor(1,4);
		lcd_print("                ");
		lcd_set_cursor(1,4);
		lcd_print(temp_read_sd_str);
		strcpy(prev_temp_read_sd_str,temp_read_sd_str);
	}
}

void lcd_audio_mode_screen_init(void)
{
    lcd_clear();
    lcd_cursor_off();
    lcd_set_cursor(0,0);
    lcd_print("    AUDIO MODE   ");
    lcd_set_cursor(1,4);
}

void lcd_audio_mode_screen(void)
{
    if(prev_lcd_screen != System_Audio_Mode_Screen)
    {
    	prev_lcd_screen = System_Audio_Mode_Screen;
    	lcd_audio_mode_screen_init();
    	prev_temp_audio_mode_str[0] = '\0';
    }
    check_switch_status();

    if(sw1_press_flag == true || sw2_press_flag == true)
    {
       sw1_press_flag = false;
       sw2_press_flag = false;
       temp_audio_mode ^= 1;
       strcpy(temp_audio_mode_str,
    		   temp_audio_mode ? "2. Enable" : "1. Disable");
    }

    if(sw3_press_flag == true)
    {
    	sw3_press_flag = false;
    	prev_lcd_screen = System_Audio_Mode_Screen;
    	lcd_screen = Setup_Menu_Screen;
    }

    if(strcmp(temp_audio_mode_str,prev_temp_audio_mode_str)!=0)
    {
    	lcd_set_cursor(1,4);
    	lcd_print("                ");
    	lcd_set_cursor(1,4);
    	lcd_print(temp_audio_mode_str);
    	strcpy(prev_temp_audio_mode_str,temp_audio_mode_str);
    }
}

void lcd_hv_write_screen_init(void)
{
    lcd_clear();
    lcd_cursor_off();
    lcd_set_cursor(0, 0);
    lcd_print("    HV Write   ");
    strcpy(prev_temp_hv_write, "");
    lcd_set_cursor(1, 4);
    lcd_print(temp_hv_write);
    edit_pos_index_hv = 0;
}

void lcd_hv_write(void)
{
    if(prev_lcd_screen != System_Hv_Read_Screen)
    {
        prev_lcd_screen = System_Hv_Read_Screen;
        lcd_hv_write_screen_init();
    }

    if(strcmp(prev_temp_hv_write, temp_hv_write) != 0)
    {
        lcd_set_cursor(1, 4);
        lcd_print(temp_hv_write);
        strcpy(prev_temp_hv_write, temp_hv_write);
    }

    check_switch_status();
    select_hv_val(temp_hv_write);

    if(sw5_press_flag == true)
    {
        sw5_press_flag = false;
    }

    if(sw3_press_flag == true)
    {
        sw3_press_flag = false;
        prev_lcd_screen = System_Hv_Read_Screen;
        lcd_screen = Setup_Menu_Screen;
    }
}

void lcd_hv_read_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0, 0);
	lcd_print("   HV Monitor   ");
}

void lcd_hv_read(void)
{
	if(prev_lcd_screen != System_Hv_Read_Screen)
	{
		prev_lcd_screen = System_Hv_Read_Screen;
		lcd_hv_read_screen_init();
	}

	check_switch_status();

	if((HAL_GetTick()-last_tick_time_rhv>1000)){
		char line[18];
		uint32_t hv_display = 0;
		if(Hv_Voltage<= 10.0f)
		{
			hv_display = 0;
		}
		else{
			hv_display =(uint32_t) Hv_Voltage;
		}
		snprintf(line, sizeof(line), "%-4u V", (unsigned int)hv_display);
		lcd_set_cursor(1,6);
		lcd_print(line);
		last_tick_time_rhv=HAL_GetTick();
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = System_Hv_Read_Screen;
		lcd_screen = Setup_Menu_Screen;
	}

	if(sw1_press_flag == true)
	{
		sw1_press_flag = false;
	}

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw2_press_flag == true)
	{
		sw2_press_flag = false;
	}

	if(sw4_press_flag == true)
	{
		sw4_press_flag = false;
	}
}

// ------------------ CALIBRATION MENU SCREEN ------------------------
void lcd_calibration_menu_init(void)
{
    lcd_clear();
    lcd_cursor_off();

    calibration_menu_index = 0;
    prev_calibration_menu_index = 0xff;
    edit_pos_index = 0;
}

void lcd_calibration_menu_screen(void)
{
	if(prev_calibration_menu_index != calibration_menu_index)
	{
		if(calibration_menu_index < 4)  // Factor1 to Factor20
		{
			lcd_set_cursor(1, 0);
			lcd_print("    Factor");
			if(calibration_menu_index < 9)  //For Two Digit e.g. 12
			{
				lcd_print(" ");
				lcd_print_int(calibration_menu_index + 1);
			}
			else
			{
				lcd_print_int(calibration_menu_index + 1);
			}
			lcd_print("    ");
		}
		else
		{
			lcd_set_cursor(0, 0);
			lcd_print("  CALIBRATION   ");
			lcd_set_cursor(1, 0);
			lcd_print("      Exit      ");
		}

		prev_calibration_menu_index = calibration_menu_index;
	}

	check_switch_status();

	if(sw4_press_flag == true)
	{
		sw4_press_flag = false;
	}

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	// -------SW1 Move down the menu------
	if(sw1_press_flag == true)
	{
		sw1_press_flag = false;
		if(calibration_menu_index < 4)
		{
			calibration_menu_index++;
		}
		else
		{
			calibration_menu_index = 0;
		}
	}

	// -------SW2 Move up the menu----------
	if(sw2_press_flag == true)
	{
		sw2_press_flag = false;
		if(calibration_menu_index > 0)
		{
			calibration_menu_index--;
		}
		else
		{
			calibration_menu_index = 4;
		}
	}

	// ------SW3 Enter selected option------
	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;

		if(calibration_menu_index < 4)
		{
			current_factor_index = calibration_menu_index;
			strcpy(temp_factor_val, factor_values[current_factor_index]);
			strcpy(prev_temp_factor_val, "");
			calibration_state = CALIBRATION_FACTOR_EDIT;
			lcd_factor_edit_screen_init();
		}
		else  // ---------Exit option-----------
		{
			lcd_screen = Setup_Menu_Screen;
			prev_lcd_screen = System_Calibration_Screen;
		}
	}
}

// --------------------- FACTOR EDIT SCREEN -----------------------------------
void lcd_factor_edit_screen_init(void)
{
    lcd_clear();
    lcd_cursor_off();
    edit_pos_index = 0;

    // Display header
    lcd_set_cursor(0, 0);
    lcd_print("    FACTOR");
    if(current_factor_index < 9)
    {
        lcd_print(" ");
        lcd_print_int(current_factor_index + 1);
    }
    else
    {
        lcd_print_int(current_factor_index + 1);
    }
    lcd_print("    ");
}

void lcd_factor_edit_screen(void)
{
    if(strcmp(prev_temp_factor_val, temp_factor_val) != 0)
    {
        lcd_set_cursor(1, 5);
        lcd_print(temp_factor_val);
        strcpy(prev_temp_factor_val, temp_factor_val);
    }

    uint8_t cursor_pos = edit_pos_index;
    if(edit_pos_index >= 2)
        cursor_pos++;

    lcd_set_cursor(1, 5 + cursor_pos);
    lcd_cursor_on();

    check_switch_status();

    if(sw5_press_flag == true)
    {
        sw5_press_flag = false;
    }

    // --------------SW1 Increment digit-----------------
    if(sw1_press_flag == true)
    {
        sw1_press_flag = false;
        increment_digit_at_position(temp_factor_val, edit_pos_index);
    }

    // -------------SW2 Decrement digit at cursor---------
    if(sw2_press_flag == true)
    {
        sw2_press_flag = false;
        decrement_digit_at_position(temp_factor_val, edit_pos_index);
    }

    // ----------------SW4 Move cursor position------------
    if(sw4_press_flag == true)
    {
        sw4_press_flag = false;
        edit_pos_index++;
        if(edit_pos_index >= 4)
            edit_pos_index = 0;
    }

    // --------------SW3 Exit back to calibration menu-----
    if(sw3_press_flag == true)
    {
        sw3_press_flag = false;
        lcd_cursor_off();

        strcpy(factor_values[current_factor_index], temp_factor_val);

        calibration_state = CALIBRATION_MENU;
        prev_calibration_menu_index = 0xff;
    }
}

// ------------------- MAIN CALIBRATION SCREEN FUNCTION --------------------------
void lcd_calibration_screen(void)
{
    if(prev_lcd_screen != System_Calibration_Screen)
    {
        prev_lcd_screen = System_Calibration_Screen;
        calibration_state = CALIBRATION_MENU;
        lcd_calibration_menu_init();
    }

    if(calibration_state == CALIBRATION_MENU)
    {
    	lcd_set_cursor(0, 0);
    	lcd_print("  CALIBRATION   ");

        lcd_calibration_menu_screen();
    }
    else
    {
        lcd_factor_edit_screen();
    }
}

void increment_digit_at_position(char* str, uint8_t pos)
{
    uint8_t actual_pos = pos;
    if(pos >= 2)
        actual_pos++;

    if(str[actual_pos] >= '0' && str[actual_pos] < '9')
        str[actual_pos]++;
    else if(str[actual_pos] == '9')
        str[actual_pos] = '0';
}

void decrement_digit_at_position(char* str, uint8_t pos)
{
    uint8_t actual_pos = pos;
    if(pos >= 2)
        actual_pos++;

    if(str[actual_pos] > '0' && str[actual_pos] <= '9')
        str[actual_pos]--;
    else if(str[actual_pos] == '0')
        str[actual_pos] = '9';
}

void load_all_factors(void)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        strcpy(factor_values[i], "00.00");
    }
}

void lcd_print_int(uint8_t num)
{
    if(num < 10)
    {
    	lcd_print_char('0' + num);
    }
    else
    {
    	lcd_print_char('0' + (num / 10));
    	lcd_print_char('0' + (num % 10));
    }
}

void lcd_save_setting_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("  SAVE SETTING ");
	lcd_set_cursor(1,4);
}

void lcd_save_setting_screen(void)
{
	if(prev_lcd_screen != System_Save_Setting_Screen)
	{
		prev_lcd_screen = System_Save_Setting_Screen;
		lcd_save_setting_screen_init();
		prev_temp_save_setting_str[0] = '\0';
	}

	check_switch_status();

	if(sw4_press_flag == true)
	{
		sw4_press_flag = false;
	}

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw1_press_flag == true || sw2_press_flag == true)
	{
		sw1_press_flag = false;
		sw2_press_flag = false;
		temp_save_setting_mode ^= 1;
		strcpy(temp_save_setting_str,
				temp_save_setting_mode ? "2. No" : "1. Yes");

	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		if(temp_save_setting_mode == 0)     // save Setting : Yes
		{
			set_configuration();
		}
		prev_lcd_screen = System_Save_Setting_Screen;
		lcd_screen = Setup_Menu_Screen;
	}

	if(strcmp(temp_save_setting_str,prev_temp_save_setting_str) != 0)
	{
		lcd_set_cursor(1,4);
		lcd_print("                ");
		lcd_set_cursor(1,4);
		lcd_print(temp_save_setting_str);
		strcpy(prev_temp_save_setting_str,temp_save_setting_str);
	}
}

void lcd_modify_password_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    PASSWORD    ");
	lcd_set_cursor(1,0);
	lcd_cursor_on();
}

void lcd_modify_password_screen(void)
{
	if(prev_lcd_screen != Modify_Password_Screen)
	{
		prev_lcd_screen = Modify_Password_Screen;
		lcd_modify_password_screen_init();
		num_cursor = 0;
		strcpy(prev_val,"");
	}

	check_switch_status();
	select_value(temp_password);

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = Modify_Password_Screen;
		lcd_screen = Setup_Menu_Screen;
	}
}

void lcd_overrange_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    OVERRANGE    ");
	lcd_set_cursor(1,0);
	prev_temp_overrange_unit = 0xff;
	strcpy(prev_temp_overrange_val ,"");
	edit_pos_index = 0;
}

void lcd_overrange_screen(void)
{
	if(prev_lcd_screen != Overrange_Setting_Screen)
	{
		prev_lcd_screen = Overrange_Setting_Screen;
		lcd_overrange_screen_init();
	}

	if (strcmp(prev_temp_overrange_val,temp_overrange_val) != 0) {
		lcd_set_cursor(1,2);
		lcd_print(temp_overrange_val);
		strcpy(prev_temp_overrange_val,temp_overrange_val);
	}

	if(prev_temp_overrange_unit != temp_overrange_unit)
	{
		lcd_set_cursor(1,10);
		lcd_print(Unit_Str_Options[temp_overrange_unit]);
		prev_temp_overrange_unit=temp_overrange_unit;
	}

	check_switch_status();
	select_alarm_format_val(temp_overrange_val,&temp_overrange_unit);

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = Overrange_Setting_Screen;
		lcd_screen = Setup_Menu_Screen;
	}
}

void lcd_overload_screen_init()
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    OVERLOAD    ");
	lcd_set_cursor(1,0);
	prev_temp_overload_unit = 0xff;
	strcpy(prev_temp_overload_val ,"");
	edit_pos_index = 0;
}

void lcd_overload_screen(void)
{
	if(prev_lcd_screen != Overload_Setting_Screen)
	{
		prev_lcd_screen = Overload_Setting_Screen;
		lcd_overload_screen_init();
	}

	if (strcmp(prev_temp_overload_val,temp_overload_val) != 0) {
		lcd_set_cursor(1,2);
		lcd_print(temp_overload_val);
		strcpy(prev_temp_overload_val,temp_overload_val);
	}

	if(prev_temp_overload_unit != temp_overload_unit)
	{
		lcd_set_cursor(1,10);
		lcd_print(Unit_Str_Options[temp_overload_unit]);
		prev_temp_overload_unit=temp_overload_unit;
	}

	check_switch_status();
	select_alarm_format_val(temp_overload_val,&temp_overload_unit);

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = Overload_Setting_Screen;
		lcd_screen = Setup_Menu_Screen;
	}
}

void lcd_4_20_max_cal_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    20mA CAL    ");
	lcd_set_cursor(1,4);
	edit_pos_index = 0;
	strcpy(prev_temp_4_20_max_cal_val,"");
}

void lcd_4_20_max_cal_screen(void)
{
	if(prev_lcd_screen != System_4_20_Max_Cal_Screen){
		prev_lcd_screen = System_4_20_Max_Cal_Screen;
		lcd_4_20_max_cal_screen_init();
	}

	if (strcmp(prev_temp_4_20_max_cal_val,temp_4_20_max_cal_val) != 0) {
		lcd_set_cursor(1,4);
		lcd_print(temp_4_20_max_cal_val);
		strcpy(prev_temp_4_20_max_cal_val,temp_4_20_max_cal_val);
	}

	select_4_20_cal_format_val(temp_4_20_max_cal_val);
	check_switch_status();

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = System_4_20_Max_Cal_Screen;
		lcd_screen = Setup_Menu_Screen;
	}
}

void lcd_4_20_min_cal_screen_init(void)
{
    lcd_clear();
    lcd_cursor_off();
    lcd_set_cursor(0,0);
    lcd_print("    4mA CAL    ");
    lcd_set_cursor(1,4);
    edit_pos_index = 0;
    strcpy(prev_temp_4_20_min_cal_val,"");
}

void lcd_4_20_min_cal_screen(void)
{
	if(prev_lcd_screen != System_4_20_Min_Cal_Screen){
		prev_lcd_screen = System_4_20_Min_Cal_Screen;
		lcd_4_20_min_cal_screen_init();
	}

	if (strcmp(prev_temp_4_20_min_cal_val,temp_4_20_min_cal_val) != 0) {
		lcd_set_cursor(1,4);
		lcd_print(temp_4_20_min_cal_val);
		strcpy(prev_temp_4_20_min_cal_val,temp_4_20_min_cal_val);
	}

	select_4_20_cal_format_val(temp_4_20_min_cal_val);
	check_switch_status();

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = System_4_20_Min_Cal_Screen;
		lcd_screen = Setup_Menu_Screen;
	}
}

void select_4_20_cal_format_val(char *val)
{
	if(sw1_press_flag == true)
	{
		sw1_press_flag = false;
		uint8_t str_index = cal_editable_indices[edit_pos_index];
		if(val[str_index] == '9')
		{
			val[str_index] = '0';
		}
		else{
			val[str_index]++;
		}
	}

	if(sw2_press_flag == true)
	{
		sw2_press_flag = false;
		uint8_t str_index = cal_editable_indices[edit_pos_index];
		if(val[str_index] == '0')
		{
			val[str_index] = '9';
		}
		else{
			val[str_index]--;
		}
	}

	if(sw4_press_flag == true)
	{
		sw4_press_flag = false;
		edit_pos_index = (edit_pos_index + 1) % 7;
	}

	uint8_t lcd_col = cal_editable_indices[edit_pos_index];
	lcd_set_cursor(1, lcd_col+4);
	lcd_cursor_on();
}

void lcd_4_20_min_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    4_20 MIN    ");
	lcd_set_cursor(1,0);
	prev_temp_4_20_min_unit = 0xff;
	strcpy(prev_temp_4_20_min_val ,"");
	edit_pos_index = 0;
}

void lcd_4_20_min_screen(void)
{
	if(prev_lcd_screen != System_4_20_Min_Screen)
	{
		prev_lcd_screen = System_4_20_Min_Screen;
		lcd_4_20_min_screen_init();
	}

	if (strcmp(prev_temp_4_20_min_val,temp_4_20_min_val) != 0) {
		lcd_set_cursor(1,2);
		lcd_print(temp_4_20_min_val);
		strcpy(prev_temp_4_20_min_val,temp_4_20_min_val);
	}

	if(prev_temp_4_20_min_unit != temp_4_20_min_unit)
	{
		lcd_set_cursor(1,10);
		lcd_print(Unit_Str_Options[temp_4_20_min_unit]);
		prev_temp_4_20_min_unit=temp_4_20_min_unit;
	}

	check_switch_status();
	select_alarm_format_val(temp_4_20_min_val,&temp_4_20_min_unit);

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = System_4_20_Min_Screen;
		lcd_screen = Setup_Menu_Screen;
	}
}

void lcd_4_20_max_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    4_20 MAX    ");
	lcd_set_cursor(1,0);
	prev_temp_4_20_max_unit = 0xff;
	strcpy(prev_temp_4_20_max_val ,"");
	edit_pos_index = 0;
}

void lcd_4_20_max_screen(void)
{
	if(prev_lcd_screen != System_4_20_Max_Screen)
	{
		prev_lcd_screen = System_4_20_Max_Screen;
		lcd_4_20_max_screen_init();
	}

	if (strcmp(prev_temp_4_20_max_val,temp_4_20_max_val) != 0) {
		lcd_set_cursor(1,2);
		lcd_print(temp_4_20_max_val);
		strcpy(prev_temp_4_20_max_val,temp_4_20_max_val);
	}

	if(prev_temp_4_20_max_unit != temp_4_20_max_unit)
	{
		lcd_set_cursor(1,10);
		lcd_print(Unit_Str_Options[temp_4_20_max_unit]);
		prev_temp_4_20_max_unit=temp_4_20_max_unit;
	}

	check_switch_status();
	select_alarm_format_val(temp_4_20_max_val,&temp_4_20_max_unit);

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = System_4_20_Max_Screen;
		lcd_screen = Setup_Menu_Screen;
	}
}

void lcd_agm_mode_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    AGM MODE    ");
	lcd_set_cursor(1,4);
}

void lcd_agm_mode_screen(void)
{
	if(prev_lcd_screen != AGM_Mode_Screen)
	{
		prev_lcd_screen = AGM_Mode_Screen;
		lcd_agm_mode_screen_init();
		prev_temp_agm_mode_str[0] = '\0';
	}

	check_switch_status();

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw1_press_flag == true || sw2_press_flag == true)
	{
		sw1_press_flag = false;
		sw2_press_flag = false;
		temp_agm_mode ^= 1;
		strcpy(temp_agm_mode_str,
				temp_agm_mode ? "2. Auto" : "1. Manual");

	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		prev_lcd_screen = AGM_Mode_Screen;
		lcd_screen = Setup_Menu_Screen;
	}

	if(strcmp(temp_agm_mode_str,prev_temp_agm_mode_str) != 0)
	{
		lcd_set_cursor(1,4);
		lcd_print("                ");
		lcd_set_cursor(1,4);
		lcd_print(temp_agm_mode_str);
		strcpy(prev_temp_agm_mode_str,temp_agm_mode_str);
	}

}

void select_value(char *val)
{
    uint8_t len = strlen((char *)val);

    while(val[num_cursor] &&
         (val[num_cursor] < '0' || val[num_cursor] > '9'))
    {
        num_cursor = (num_cursor + 1) % len;
    }

    uint8_t changed = 0;

    if(sw1_press_flag)
    {
        sw1_press_flag = false;
        val[num_cursor] = (val[num_cursor] == '9') ? '0' : val[num_cursor] + 1;
        changed = 1;
    }

    if(sw2_press_flag)
    {
        sw2_press_flag = false;
        val[num_cursor] = (val[num_cursor] == '0') ? '9' : val[num_cursor] - 1;
        changed = 1;
    }

    if(sw4_press_flag)
    {
        sw4_press_flag = false;
        do {
            num_cursor = (num_cursor + 1) % len;
        } while(val[num_cursor] < '0' || val[num_cursor] > '9');
    }

    if(strcmp((char*)prev_val, (char*)val) != 0)
    {
        lcd_set_cursor(1,6);
        lcd_print(val);
        strcpy((char*)prev_val, (char*)val);
    }

    lcd_set_cursor(1, num_cursor + 6);
}

void select_ip_value(char *ip)
{
    uint8_t len = strlen((char *)ip);
    uint8_t changed = 0;

    if(ip[eth_ip_cursor] == '.')
        eth_ip_cursor = (eth_ip_cursor + 1) % len;

    if(sw1_press_flag)
    {
        sw1_press_flag = false;
        ip[eth_ip_cursor] = (ip[eth_ip_cursor] == '9') ? '0' : ip[eth_ip_cursor] + 1;
        changed = 1;
    }

    if(sw2_press_flag)
    {
        sw2_press_flag = false;
        ip[eth_ip_cursor] = (ip[eth_ip_cursor] == '0') ? '9' : ip[eth_ip_cursor] - 1;
        changed = 1;
    }

    if(sw4_press_flag)
    {
        sw4_press_flag = false;
        do {
            eth_ip_cursor = (eth_ip_cursor + 1) % len;
        } while(ip[eth_ip_cursor] == '.');
    }

    if(strcmp((char*)prev_ip, (char*)ip) != 0)
    {
        lcd_set_cursor(1,0);
        lcd_print(ip);
        strcpy((char*)prev_ip, (char*)ip);
    }

    lcd_set_cursor(1, eth_ip_cursor);
}

void lcd_ip_edit_screen(char *title, char *ip)
{
	static uint8_t init = 0;

	if(!init)
	{
		lcd_clear();
		lcd_set_cursor(0,0);
		if(strcmp(title,"      IP    ")==0)
		{
			eth_state = ETH_STATE_IP;
		}
		else if(strcmp(title,"     SUBNET  ")==0)
		{
			eth_state = ETH_STATE_SUBNET;
		}
		else
		{
			eth_state = ETH_STATE_GATEWAY;
		}
		lcd_print(title);
		init = 1;
		eth_ip_cursor = 0;

		lcd_cursor_on();

		strcpy(prev_ip,"");
	}

	check_switch_status();
	select_ip_value(ip);

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw3_press_flag)
	{
		sw3_press_flag = false;
		init = 0;
		if(eth_state == ETH_STATE_IP)
		{
			prev_eth_state = ETH_STATE_IP;
		}
		else if(eth_state == ETH_STATE_SUBNET)
		{
			prev_eth_state = ETH_STATE_SUBNET;
		}
		else
		{
			prev_eth_state = ETH_STATE_GATEWAY;
		}
		eth_state = ETH_STATE_MENU;

		lcd_cursor_off();
	}
}

void lcd_numeric_edit_screen(char *title, char *value)
{
	static uint8_t init = 0;

	if(!init)
	{
		lcd_clear();
		lcd_set_cursor(0,0);
		if(strcmp(title,"    SLAVE ID ")==0)
		{
			eth_state = ETH_STATE_SLAVE_ID;
		}
		else{
			eth_state = ETH_STATE_PORT;
		}
		lcd_print(title);
		init = 1;
		num_cursor = 0;

		lcd_cursor_on();

		strcpy(prev_val,"");
	}

	check_switch_status();
	select_value(value);

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw3_press_flag)
	{
		sw3_press_flag = false;
		init = 0;
		if(eth_state == ETH_STATE_SLAVE_ID)
		{
			prev_eth_state = ETH_STATE_SLAVE_ID;
		}
		else{
			prev_eth_state = ETH_STATE_PORT;
		}
		eth_state = ETH_STATE_MENU;

		lcd_cursor_off();
	}
}

void lcd_ethernet_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("     ETH SET   ");
	lcd_cursor_off();
	prev_eth_menu = ETH_MENU_MAX;
}

void eth_menu_screen(void)
{
	if(prev_eth_state != eth_state)
	{
		lcd_ethernet_screen_init();
		prev_eth_state = eth_state;
	}

	check_switch_status();

	if(sw4_press_flag == true)
	{
		sw4_press_flag = false;
	}

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw1_press_flag)
	{
		sw1_press_flag = false;
		eth_menu = (eth_menu + 1) % ETH_MENU_MAX;
	}

	if(sw2_press_flag)
	{
		sw2_press_flag = false;
		eth_menu = (eth_menu == 0) ? ETH_MENU_MAX - 1 : eth_menu - 1;
	}

	if(sw3_press_flag)
	{
		sw3_press_flag = false;

		switch(eth_menu)
		{
		case ETH_MENU_SLAVE_ID: eth_state = ETH_STATE_SLAVE_ID; break;
		case ETH_MENU_PORT:     eth_state = ETH_STATE_PORT;    break;
		case ETH_MENU_IP:       eth_state = ETH_STATE_IP;      break;
		case ETH_MENU_SUBNET:   eth_state = ETH_STATE_SUBNET;  break;
		case ETH_MENU_GATEWAY:  eth_state = ETH_STATE_GATEWAY; break;

		case ETH_MENU_EXIT:
			eth_state = ETH_STATE_MENU;
			eth_menu = ETH_MENU_SLAVE_ID;
			prev_eth_state = ETH_STATE_NONE;

			lcd_screen = Setup_Menu_Screen;
			prev_lcd_screen = Ethernet_Setting_Screen;
			return;
		}

	}

	if(prev_eth_menu != eth_menu)
	{
		lcd_set_cursor(1,0);
		lcd_print("                ");
		lcd_set_cursor(1,0);
		lcd_print(eth_menu_str[eth_menu]);
		prev_eth_menu = eth_menu;
	}
}

void lcd_ethernet_screen(void)
{
    switch(eth_state)
    {
        case ETH_STATE_MENU:
            eth_menu_screen();
            break;

        case ETH_STATE_SLAVE_ID:
            lcd_numeric_edit_screen("    SLAVE ID ", eth_slave_id);
            break;

        case ETH_STATE_PORT:
            lcd_numeric_edit_screen("      PORT   ", eth_port);
            break;

        case ETH_STATE_IP:
            lcd_ip_edit_screen("      IP    ", eth_ip);
            break;

        case ETH_STATE_SUBNET:
            lcd_ip_edit_screen("     SUBNET  ", eth_subnet);
            break;

        case ETH_STATE_GATEWAY:
            lcd_ip_edit_screen("    GATEWAY ", eth_gateway);
            break;
    }
}

void rtc_menu_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    RTC SET   ");
	prev_rtc_menu = RTC_MENU_MAX;
}

void rtc_menu_screen(void)
{
	if(rtc_state != prev_rtc_state)
	{
		prev_rtc_state = rtc_state;
		rtc_menu_screen_init();
	}
	check_switch_status();

	if(sw4_press_flag == true)
	{
		sw4_press_flag = false;
	}

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw1_press_flag)
	{
		sw1_press_flag = false;
		rtc_menu = (rtc_menu + 1) % RTC_MENU_MAX;
	}

	if(sw2_press_flag)
	{
		sw2_press_flag = false;
		rtc_menu = (rtc_menu == 0) ? RTC_MENU_MAX - 1 : rtc_menu - 1;
	}

	if(sw3_press_flag)
	{
		sw3_press_flag = false;
		switch(rtc_menu)
		{
		case RTC_MENU_DATE :        rtc_state = RTC_STATE_DATE;     break;
		case RTC_MENU_TIME:         rtc_state = RTC_STATE_TIME;     break;
		case RTC_MENU_EXIT:

			rtc_state = RTC_STATE_MENU;
			prev_rtc_state = RTC_STATE_NONE;
			rtc_menu = RTC_MENU_DATE;

			lcd_screen = Setup_Menu_Screen;
			prev_lcd_screen = System_Rtc_Screen;
			return;
		}
	}

	if(rtc_menu != prev_rtc_menu)
	{
		lcd_set_cursor(1,0);
		lcd_print("                ");
		lcd_set_cursor(1,0);
		lcd_print(rtc_menu_str[rtc_menu]);
		prev_rtc_menu = rtc_menu;
	}
}

void lcd_rtc_date_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("     DATE      ");
	lcd_set_cursor(1,2);

	strcpy(prev_temp_rtc_date_val ,"");
	rtc_cursor = 0;
}

void rtc_date_screen(void)
{
	if (rtc_state != prev_rtc_state)
	{
		prev_rtc_state = rtc_state;
		lcd_rtc_date_screen_init();
	}

	check_switch_status();
	select_rtc_format_val(temp_rtc_date_val);

	if (strcmp(prev_temp_rtc_date_val, temp_rtc_date_val) != 0 )
	{
		lcd_set_cursor(1, 2);
		lcd_print(temp_rtc_date_val);
		strcpy(prev_temp_rtc_date_val, temp_rtc_date_val);
	}
	lcd_set_cursor(1,rtc_cursor+2);
	lcd_cursor_on();

	if (sw5_press_flag) { sw5_press_flag = false; }

	if (sw3_press_flag)
	{
		if(!Validate_Date_String())
		{
			lcd_clear();
			lcd_cursor_off();
			lcd_set_cursor(0,0);
			lcd_print(" Invalid Format");
			HAL_Delay(1000);
		}
		sw3_press_flag = false;
		rtc_state = RTC_STATE_MENU;
		prev_rtc_state = RTC_STATE_DATE;

	}
}

void lcd_rtc_time_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("     TIME      ");
	lcd_set_cursor(1,3);

	strcpy(prev_temp_rtc_time_val ,"");
	rtc_cursor = 0;
}

void rtc_time_screen(void)
{
	if(rtc_state != prev_rtc_state)
	{
		prev_rtc_state = rtc_state;
		lcd_rtc_time_screen_init();
	}

	check_switch_status();
	select_rtc_format_val(temp_rtc_time_val);

	if (strcmp(prev_temp_rtc_time_val, temp_rtc_time_val) != 0 )
	{
		lcd_set_cursor(1, 3);
		lcd_print(temp_rtc_time_val);
		strcpy(prev_temp_rtc_time_val, temp_rtc_time_val);
	}
	lcd_set_cursor(1,rtc_cursor+3);
	lcd_cursor_on();

	if (sw5_press_flag) { sw5_press_flag = false; }

	if (sw3_press_flag)
	{
		if(!Validate_Time_String())
		{
			lcd_clear();
			lcd_cursor_off();
			lcd_set_cursor(0,0);
			lcd_print(" Invalid Format");
			HAL_Delay(1000);
		}
		sw3_press_flag = false;
		rtc_state = RTC_STATE_MENU;
		prev_rtc_state = RTC_STATE_TIME;
	}
}

void lcd_rtc_screen(void)
{
	switch(rtc_state)
	{
	case RTC_STATE_MENU:
		rtc_menu_screen();
		break;
	case RTC_STATE_DATE:
		rtc_date_screen();
		break;
	case RTC_STATE_TIME:
		rtc_time_screen();
		break;
	}
}

void rs485_menu_screen_init(void)
{
	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0,0);
	lcd_print("    RS485 SET   ");
	prev_rs485_menu = rs485_menu_max;
}

void rs485_menu_screen(void)
{
	if(rs485_state != prev_rs485_state)
	{
		prev_rs485_state = rs485_state;
		rs485_menu_screen_init();
	}
	check_switch_status();

	if(sw4_press_flag == true)
	{
		sw4_press_flag = false;
	}

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw1_press_flag)
	{
		sw1_press_flag = false;
		rs485_menu = (rs485_menu + 1) % rs485_menu_max;
	}

	if(sw2_press_flag)
	{
		sw2_press_flag = false;
		rs485_menu = (rs485_menu == 0) ? rs485_menu_max - 1 : rs485_menu - 1;
	}

	if(sw3_press_flag)
	{
		sw3_press_flag = false;
		switch(rs485_menu)
		{
		case rs485_menu_slave_id :        rs485_state = rs485_state_slave_id;     break;
		case rs485_menu_baud_rate:        rs485_state = rs485_state_baud_rate;    break;
		case rs485_menu_exit:

			rs485_state = rs485_state_menu;
			prev_rs485_state = rs485_state_none;
			rs485_menu = rs485_menu_slave_id;

			lcd_screen = Setup_Menu_Screen;
			prev_lcd_screen = RS485_Setting_Screen;
			return;
		}
	}

	if(rs485_menu != prev_rs485_menu)
	{
		lcd_set_cursor(1,0);
		lcd_print("                ");
		lcd_set_cursor(1,0);
		lcd_print(rs485_menu_str[rs485_menu]);
		prev_rs485_menu = rs485_menu;
	}
}

void rs485_slave_id_screen(void)
{
	static uint8_t init = 0;

	if(!init)
	{
		lcd_clear();
		lcd_set_cursor(0,0);
		lcd_print("    SLAVE ID    ");
		init = 1;
		num_cursor = 0;

		lcd_cursor_on();

		strcpy(prev_val,"");
	}

	check_switch_status();
	select_value(rs485_slave_id);

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw3_press_flag)
	{
		sw3_press_flag = false;
		init = 0;
		lcd_cursor_off();

		rs485_state =  rs485_state_menu;
		prev_rs485_state = rs485_state_slave_id;
	}
}

void rs485_baud_rate_screen_init(void)
{
    lcd_clear();
    lcd_cursor_off();
    lcd_set_cursor(0,0);
    lcd_print("   BAUD RATE    ");
    lcd_set_cursor(1,4);
}

void getbaudrate_print(uint8_t baud, char *dest)
{
    switch(baud){
        case 0:  strcpy(dest, "1. 2400");    break;
        case 1:  strcpy(dest, "2. 4800");    break;
        case 2:  strcpy(dest, "3. 9600");    break;
        case 3:  strcpy(dest, "4. 14400");   break;
        case 4:  strcpy(dest, "5. 19200");   break;
        case 5:  strcpy(dest, "6. 38400");   break;
        case 6:  strcpy(dest, "7. 57600");   break;
        case 7:  strcpy(dest, "8. 115200");  break;
        default: strcpy(dest, "3. 9600");    break;
    }
}

void rs485_baud_rate_screen(void)
{

	if(rs485_state != prev_rs485_state)
	{
		prev_rs485_state = rs485_state;
		rs485_baud_rate_screen_init();
		strcpy(prev_temp_baud_rate_str, "");
	}

	getbaudrate_print(temp_baud_rate,temp_baud_rate_str);

	check_switch_status();

	if(sw5_press_flag  == true)
	{
		sw5_press_flag = false;
	}

	if(sw1_press_flag == true)
	{
		sw1_press_flag = false;
		if(temp_baud_rate == 7)
		{
			temp_baud_rate = 0;
		}
		else{
			temp_baud_rate += 1;
		}
	}

	if(sw2_press_flag == true)
	{
		sw2_press_flag = false;
		if(temp_baud_rate == 0)
		{
			temp_baud_rate = 7;
		}
		else
		{
			temp_baud_rate -= 1;
		}
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		rs485_state =  rs485_state_menu;
		prev_rs485_state = rs485_state_baud_rate;
	}

	if(strcmp(temp_baud_rate_str,prev_temp_baud_rate_str) != 0){
		lcd_set_cursor(1,4);
		lcd_print("                ");
		lcd_set_cursor(1,4);
		lcd_print(temp_baud_rate_str);
		strcpy(prev_temp_baud_rate_str,temp_baud_rate_str);
	}
}

void lcd_rs485_screen(void)
{
      switch(rs485_state)
      {
         case rs485_state_menu:
    	      rs485_menu_screen();
    	      break;
         case rs485_state_slave_id:
        	  rs485_slave_id_screen();
        	  break;
         case rs485_state_baud_rate:
        	  rs485_baud_rate_screen();
        	  break;
      }
}

static void convert_val_to_integer(char* val)
{
    val[4] = val[5];
    val[5] = ' ';
    val[6] = ' ';
    val[7] = '\0';
}

static void convert_val_to_decimal(char* val)
{
    val[6] = '0';
    val[5] = val[4];
    val[4] = '.';
    val[7] = '\0';
}

void select_rtc_format_val(char *val)
{
	uint8_t len = strlen((char *)val);

	if(val[rtc_cursor] == '.')
		rtc_cursor = (rtc_cursor + 1) % len;

	if(sw1_press_flag)
	{
		sw1_press_flag = false;
		val[rtc_cursor] = (val[rtc_cursor] == '9') ? '0' : val[rtc_cursor] + 1;
	}

	if(sw2_press_flag)
	{
		sw2_press_flag = false;
		val[rtc_cursor] = (val[rtc_cursor] == '0') ? '9' : val[rtc_cursor] - 1;
	}

	if(sw4_press_flag)
	{
		sw4_press_flag = false;
		do {
			rtc_cursor = (rtc_cursor + 1) % len;
		} while(val[rtc_cursor] == '.');
	}
}

void select_alarm_format_val(char* val, uint8_t* unit)
{
    uint8_t tbl_len;
    const uint8_t* tbl = get_index_table(*unit, &tbl_len);

    if (sw1_press_flag)
    {
        sw1_press_flag = false;
        uint8_t str_index = tbl[edit_pos_index];

        if (str_index == 8)
        {
            uint8_t old_is_int = unit_is_integer(*unit);
            *unit = (*unit == 3) ? 0 : (*unit + 1);
            uint8_t new_is_int = unit_is_integer(*unit);

            if (!old_is_int && new_is_int) convert_val_to_integer(val);
            if ( old_is_int && !new_is_int) convert_val_to_decimal(val);

            tbl = get_index_table(*unit, &tbl_len);
            edit_pos_index = tbl_len - 1;
        }
        else
        {
            val[str_index] = (val[str_index] == '9') ? '0' : val[str_index] + 1;
        }
    }

    if (sw2_press_flag)
    {
        sw2_press_flag = false;
        uint8_t str_index = tbl[edit_pos_index];

        if (str_index == 8)
        {
            uint8_t old_is_int = unit_is_integer(*unit);
            *unit = (*unit == 0) ? 3 : (*unit - 1);
            uint8_t new_is_int = unit_is_integer(*unit);

            if (!old_is_int && new_is_int) convert_val_to_integer(val);
            if ( old_is_int && !new_is_int) convert_val_to_decimal(val);

            tbl = get_index_table(*unit, &tbl_len);
            edit_pos_index = tbl_len - 1;
        }
        else
        {
            val[str_index] = (val[str_index] == '0') ? '9' : val[str_index] - 1;
        }
    }

    if (sw4_press_flag)
    {
        sw4_press_flag = false;
        edit_pos_index = (edit_pos_index + 1) % tbl_len;
    }

    tbl = get_index_table(*unit, &tbl_len);
    uint8_t str_index = tbl[edit_pos_index];

    uint8_t lcd_col;
    if (str_index == 8)
    {
        lcd_col = 10;
    }
    else
    {
        lcd_col = str_index + 2;
    }

    lcd_set_cursor(1, lcd_col);
    lcd_cursor_on();
}

void lcd_alarm_screen_init(void)
{
    lcd_clear();
    lcd_cursor_off();
    lcd_set_cursor(0,0);
    lcd_print("     ALARM      ");
    lcd_set_cursor(1,0);
    prev_temp_alarm_unit = 0xff;
    strcpy(prev_temp_alarm_val ,"");
    edit_pos_index = 0;
}

void lcd_alarm_screen(void)
{
    if (prev_lcd_screen != Set_Alarm_Screen)
    {
        prev_lcd_screen = Set_Alarm_Screen;
        lcd_alarm_screen_init();
    }

    if (strcmp(prev_temp_alarm_val, temp_alarm_val) != 0 ||
        prev_temp_alarm_unit != temp_alarm_unit)
    {
        lcd_set_cursor(1, 2);
        lcd_print(temp_alarm_val);
        lcd_set_cursor(1, 10);
        lcd_print(Unit_Str_Options[temp_alarm_unit]);

        strcpy(prev_temp_alarm_val, temp_alarm_val);
        prev_temp_alarm_unit = temp_alarm_unit;
    }

    check_switch_status();
    select_alarm_format_val(temp_alarm_val, &temp_alarm_unit);

    if (sw5_press_flag) { sw5_press_flag = false; }

    if (sw3_press_flag)
    {
        sw3_press_flag = false;
        prev_lcd_screen = Set_Alarm_Screen;
        lcd_screen = Setup_Menu_Screen;
    }
}

void lcd_unit_screen_init(void)
{
    lcd_clear();
    lcd_cursor_off();
    lcd_set_cursor(0,0);
    lcd_print("    SET UNIT   ");
    lcd_set_cursor(1,4);
}

void GetUnitString_print(uint8_t unit, char *dest)
{
	switch(unit){
	case 0:  strcpy(dest, "1. mR/h");  break;
	case 1:  strcpy(dest, "2. uSv/h"); break;
	case 2:  strcpy(dest, "3. cps");   break;
	case 3:  strcpy(dest, "4. cpm");   break;
	default: strcpy(dest, "1. mR/h");  break;
	}
}

void lcd_unit_screen(void)
{
   if(prev_lcd_screen != Set_Unit_Screen)
   {
	   prev_lcd_screen = Set_Unit_Screen;
	   lcd_unit_screen_init();
	   strcpy(prev_temp_primary_unit_str, "");
   }

   GetUnitString_print(temp_primary_unit,temp_primary_unit_str);

   check_switch_status();

   if(sw5_press_flag  == true)
   {
	   sw5_press_flag = false;
   }

   if(sw1_press_flag == true)
   {
	   sw1_press_flag = false;
	   if(temp_primary_unit == 3)
	   {
		   temp_primary_unit = 0;
	   }
	   else{
	       temp_primary_unit += 1;
	   }
   }

   if(sw2_press_flag == true)
   {
	   sw2_press_flag = false;
	   if(temp_primary_unit == 0)
	   {
		   temp_primary_unit = 3;
	   }
	   else
	   {
	       temp_primary_unit -= 1;
	   }
   }

   if(sw3_press_flag == true)
   {
	   sw3_press_flag = false;
	   prev_lcd_screen = Set_Unit_Screen;
	   lcd_screen = Setup_Menu_Screen;
   }

   if(strcmp(temp_primary_unit_str,prev_temp_primary_unit_str) != 0){
	   lcd_set_cursor(1,4);
	   lcd_print("                ");
	   lcd_set_cursor(1,4);
       lcd_print(temp_primary_unit_str);
       strcpy(prev_temp_primary_unit_str,temp_primary_unit_str);
   }
}

void lcd_setup_menu_screen_init(){

	lcd_clear();
	lcd_cursor_off();
	lcd_set_cursor(0, 0);
	lcd_print("   SETUP MENU   ");
	lcd_set_cursor(1, 0);
	prev_menu_options = 0xFF;

}

void select_lcd_screen(void)
{
	switch(menu_options){

		case 0:  lcd_screen = Set_Unit_Screen;            break;
		case 1:  lcd_screen = Set_Alarm_Screen;           break;
		case 2:  lcd_screen = RS485_Setting_Screen;       break;
		case 3:  lcd_screen = Ethernet_Setting_Screen;    break;
		case 4:  lcd_screen = AGM_Mode_Screen;            break;
		case 5:  lcd_screen = System_4_20_Min_Screen;     break;
		case 6:  lcd_screen = System_4_20_Max_Screen;     break;
		case 7:  lcd_screen = System_4_20_Min_Cal_Screen; break;
		case 8:  lcd_screen = System_4_20_Max_Cal_Screen; break;
		case 9:  lcd_screen = Overrange_Setting_Screen;   break;
		case 10: lcd_screen = Overload_Setting_Screen;    break;
		case 11: lcd_screen = System_Calibration_Screen;  break;
		case 12: lcd_screen = System_Hv_Read_Screen;      break;
		case 13: lcd_screen = System_Hv_Write_Screen;     break;
		case 14: lcd_screen = System_Audio_Mode_Screen;   break;
		case 15: lcd_screen = System_Freq_Recv_Screen;    break;
		case 16: lcd_screen = System_Freq_Dtc_Screen;     break;
		case 17: lcd_screen = System_Rtc_Screen;          break;
		case 18: lcd_screen = Modify_Password_Screen;     break;
		case 19: lcd_screen = System_Read_Data;           break;
		case 20: lcd_screen = System_Save_Setting_Screen; break;
		default: lcd_screen = Set_Unit_Screen;            break;
	}
}

void lcd_setup_menu_screen(void)
{
	if (prev_lcd_screen != Setup_Menu_Screen)
	{
		prev_lcd_screen = Setup_Menu_Screen;
		lcd_setup_menu_screen_init();
	}

	check_switch_status();

	if(sw4_press_flag == true)
	{
		sw4_press_flag = false;
	}

	if(sw1_press_flag == true)
	{
		sw1_press_flag = false;
		if(menu_options == SYSTEM_MENU_OPTION)
		{
			menu_options = 0;
		}
		else
		{
			menu_options += 1;
		}
	}

	if(sw2_press_flag == true)
	{
		sw2_press_flag = false;
		if(menu_options == 0)
		{
			menu_options = SYSTEM_MENU_OPTION;
		}
		else{
			menu_options -= 1;
		}
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		select_lcd_screen();
		prev_lcd_screen = Setup_Menu_Screen;
	}

	if(sw5_press_flag == true)
	{
		sw5_press_flag = false;
		menu_options =  0;
		lcd_screen = Home_Screen;
		prev_lcd_screen = Setup_Menu_Screen;
	}

	if(prev_menu_options != menu_options){
		prev_menu_options = menu_options;
		lcd_set_cursor(1, 0);
		lcd_print("                ");
		lcd_set_cursor(1, 0);
		switch(menu_options){
		case 0:   lcd_print("1. Set Unit");       break;
		case 1:   lcd_print("2. Alarm   ");       break;
		case 2:   lcd_print("3. RS485 Set ");     break;
		case 3:   lcd_print("4. Ethernet Set");   break;
		case 4:   lcd_print("5. AGM Mode Set");   break;
		case 5:   lcd_print("6. 4_20 Min Set");   break;
		case 6:   lcd_print("7. 4_20 Max Set");   break;
		case 7:   lcd_print("8. 4mA Cal Set");    break;
		case 8:   lcd_print("9. 20mA Cal Set");   break;
		case 9:   lcd_print("10. Overrange ");    break;
		case 10:  lcd_print("11. Overload ");     break;
		case 11:  lcd_print("12. Calibration");   break;
		case 12:  lcd_print("13. Hv Read");       break;
		case 13:  lcd_print("14. Hv Write");      break;
		case 14:  lcd_print("15. Audio");         break;
		case 15:  lcd_print("16. Recv Freq");     break;
		case 16:  lcd_print("17. Dtc Freq");      break;
		case 17:  lcd_print("18. Set Time");      break;
		case 18:  lcd_print("19. Set Password");  break;
		case 19:  lcd_print("20. Read Data");     break;
		case 20:  lcd_print("21. Save Set");      break;
		}
	}
}

bool verify_password(void)
{
	bool status = false;
    build_password_string();

    if (strcmp(correct_password, entered_password) == 0)
    {
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print(" Access Granted ");
        HAL_Delay(1000);
        lcd_screen = Setup_Menu_Screen;
        prev_lcd_screen = Password_Screen;
        status = true;
    }
    else
    {
    	lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print(" Access Denied  ");
        HAL_Delay(1000);
        prev_lcd_screen = 0xff;
        status = false;
    }
    return status;
}

void build_password_string(void)
{
    for (int i = 0; i < 4; i++)
    {
        entered_password[i] = pwd[i] + '0';
    }
    entered_password[4] = '\0';
}

void lcd_password_screen_init(void)
{
    lcd_clear();

    lcd_set_cursor(0,4);
    lcd_print("PASSWORD");

    lcd_set_cursor(1,6);
    lcd_print("0000");

    pwd[0] = pwd[1] = pwd[2] = pwd[3] = 0;
    pwd_pos = 0;

    lcd_set_cursor(1,6);
    lcd_cursor_on();
}

void lcd_update_pwd_digit(uint8_t pos)
{
    char c = '0' + pwd[pos];

    lcd_set_cursor(1, 6 + pos);
    lcd_print_char(c);
    lcd_set_cursor(1, 6 + pwd_pos);
}

void pwd_sw1_increment(void)
{
    pwd[pwd_pos] = (pwd[pwd_pos] + 1) % 10;
    lcd_update_pwd_digit(pwd_pos);
}

void pwd_sw2_decrement(void)
{
    if (pwd[pwd_pos] == 0)
        pwd[pwd_pos] = 9;
    else
        pwd[pwd_pos]--;

    lcd_update_pwd_digit(pwd_pos);
}

void pwd_sw4_next_digit(void)
{
    pwd_pos = (pwd_pos + 1) % 4;
    lcd_set_cursor(1, 6 + pwd_pos);
}

void lcd_password_screen(void)
{
	if (prev_lcd_screen != Password_Screen)
	{
		prev_lcd_screen = Password_Screen;
		lcd_password_screen_init();
	}

	check_switch_status();

	if(sw1_press_flag == true)
	{
		sw1_press_flag = false;
		pwd_sw1_increment();
	}
	if(sw2_press_flag == true)
	{
		sw2_press_flag = false;
		pwd_sw2_decrement();
	}
	if(sw4_press_flag == true)
	{
		sw4_press_flag = false;
		pwd_sw4_next_digit();
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
		bool check=verify_password();
		if(check){
			copy_saved_values_to_temp();
		}
	}

	if(sw5_press_flag == true)
	{
		sw5_press_flag = false;
		lcd_screen = Home_Screen;
		prev_lcd_screen = Password_Screen;
	}
}

void lcd_home_screen_init(void)
{
	lcd_clear();
	lcd_set_cursor(0, 4);
	lcd_print("DOSE RATE");
	lcd_cursor_off();

	last_unit = 0xFF;
	menu_options = 0;

	last_system_state = (SystemStateFlag)0xFF;
}

void lcd_home_screen_update(uint32_t cps, uint16_t unit_mode)
{
	char str[16];
	char unit[7] = "";
	float dose = 0.0;
	char headline[14] = "";

	// ---------------- Update HEADLINE ----------------
	if (State_Flag != last_system_state)
	{
		switch (State_Flag)
		{
		case Normal_Flag:
			strcpy(headline, "DOSE RATE");
			break;
		case Alarm_Flag:
			strcpy(headline, "ALARM");
			break;
		case Overrun_Flag:
			strcpy(headline, "   OR");
			break;
		case Overload_Flag:
			strcpy(headline, "   OL");
			break;
		case Hv_Fault_Flag:
			strcpy(headline, "HV FAIL  ");
			break;
		case Dtc_Failed_Flag:
			strcpy(headline, "DET FAIL");
			break;
		case Ack_Flag_Alarm:
			strcpy(headline, "ACK ALARM");
			break;
		case Ack_Flag_Overrun:
			strcpy(headline, "ACK OR");
			break;
		case Ack_Flag_OverLoad:
			strcpy(headline, "ACK OL");
			break;
		default:
			strcpy(headline, "DOSE RATE");
			break;
		}

		lcd_set_cursor(0, 0);
		lcd_print("              ");
		lcd_set_cursor(0, 4);
		lcd_print(headline);

		last_system_state = State_Flag;
	}

	// ---------- Update UNIT only ----------
	if (unit_mode != last_unit)
	{
		switch (unit_mode)
		{
		case 0: strcpy(unit, "mR/h "); break;
		case 1: strcpy(unit, "uSv/h"); break;
		case 2: strcpy(unit, "cps  "); break;
		case 3: strcpy(unit, "cpm  "); break;
		}
		lcd_set_cursor(1, 10);
		lcd_print("     ");
		lcd_set_cursor(1, 10);
		lcd_print(unit);

		last_unit = unit_mode;
	}

	// ---------- Update VALUE ----------

	switch (unit_mode)
	{
	case 0: // ----------mR/h----------
		dose = dose_mRh;
		break;
	case 1: // ---------µSv/h----------
		dose = dose_mRh * 10.0f;
		break;
	case 2: // ---------CPS--------------
		break;
	case 3: // ---------CPM--------------
		break;
	}

	if(unit_mode == 2)       //cps
	{
		snprintf(str, sizeof(str), "%06ld", cps);
	}
	else if(unit_mode == 3)  //cpm
	{
		snprintf(str, sizeof(str), "%06ld", cpm);
	}
	else{
		//			snprintf(str, sizeof(str), "%07.2f", dose);

		uint32_t dose_x100 = (uint32_t)(dose * 100.0f + 0.5f);
		snprintf(str, sizeof(str),
				"%04lu.%02lu",
				dose_x100 / 100,
				dose_x100 % 100);
	}

	lcd_set_cursor(1,2);
	lcd_print(str);

	// ----------Update 7_Segment------------------
	uint32_t scaled_val;
	if(unit_mode == 2)        //cps
	{
		scaled_val = cps;
		MAX7219_DisplayFloat(scaled_val,0);
	}
	else if(unit_mode == 3)   //cpm
	{
		scaled_val = cpm;
		MAX7219_DisplayFloat(scaled_val,0);
	}
	else{
		scaled_val = (uint32_t)(dose * 100 + 0.5f);
		MAX7219_DisplayFloat(scaled_val,2);
	}

}

void lcd_home_screen(uint32_t cps, uint16_t unit_mode)
{
	if (prev_lcd_screen != Home_Screen)
	{
		prev_lcd_screen = Home_Screen;
		lcd_home_screen_init();
	}

	lcd_home_screen_update(cps, unit_mode);

	if(sw5_press_flag == true)
	{
		sw5_press_flag = false;
		lcd_screen = Password_Screen;
		prev_lcd_screen = Home_Screen;
	}

	if(sw1_press_flag == true)
	{
		sw1_press_flag = false;
	}

	if(sw2_press_flag == true)
	{
		sw2_press_flag = false;
	}

	if(sw3_press_flag == true)
	{
		sw3_press_flag = false;
	}

	if(sw4_press_flag == true)
	{
		sw4_press_flag = false;
	}

	check_switch_status();
}

void check_switch_status(void)
{
	const uint32_t debounce_delay = 200;
	static uint32_t last_sw3_tick = 0;
	static uint32_t last_sw4_tick = 0;
	static uint32_t last_sw2_tick = 0;
	static uint32_t last_sw1_tick = 0;
	uint32_t now = HAL_GetTick();

	// --- Handle Increment ---
	if (HAL_GPIO_ReadPin(SW1_PORT, SW1_PIN) == GPIO_PIN_SET &&
			(now - last_sw1_tick > 300)) {
		last_sw1_tick = now;
		sw1_press_flag = true;
	}

	// --- Handle Enter  ---
	if (HAL_GPIO_ReadPin(SW2_PORT, SW2_PIN) == GPIO_PIN_SET &&
			(now - last_sw2_tick > debounce_delay)) {

		last_sw2_tick = now;
		sw2_press_flag = true;
	}

	// --- Handle Decrement ---
	if (HAL_GPIO_ReadPin(SW3_PORT, SW3_PIN) == GPIO_PIN_SET &&
			(now - last_sw3_tick > debounce_delay)) {

		last_sw3_tick = now;
		sw3_press_flag = true;
	}

	// ---To move cursor ---
	if (HAL_GPIO_ReadPin(SW4_PORT, SW4_PIN) == GPIO_PIN_SET &&
			(now - last_sw4_tick > debounce_delay)) {

		last_sw4_tick = now;
		sw4_press_flag = true;
	}

}

void system_lcd_screen(void)
{
  switch(lcd_screen)
  {
   case Home_Screen:
        lcd_home_screen(CPS_VAL,primary_unit);
        break;
   case Password_Screen:
	    lcd_password_screen();
	    break;
   case Setup_Menu_Screen:
	    lcd_setup_menu_screen();
	    break;
   case Set_Unit_Screen:
	    lcd_unit_screen();
	    break;
   case Set_Alarm_Screen:
	    lcd_alarm_screen();
	    break;
   case RS485_Setting_Screen:
	    lcd_rs485_screen();
	    break;
   case Ethernet_Setting_Screen:
	    lcd_ethernet_screen();
	    break;
   case AGM_Mode_Screen:
	    lcd_agm_mode_screen();
	    break;
   case System_4_20_Min_Screen:
	    lcd_4_20_min_screen();
	    break;
   case System_4_20_Max_Screen:
	    lcd_4_20_max_screen();
	    break;
   case System_4_20_Min_Cal_Screen:
	    lcd_4_20_min_cal_screen();
	    break;
   case System_4_20_Max_Cal_Screen:
	    lcd_4_20_max_cal_screen();
	    break;
   case Overload_Setting_Screen:
	    lcd_overload_screen();
	    break;
   case Overrange_Setting_Screen:
	    lcd_overrange_screen();
	    break;
   case System_Calibration_Screen:
	    lcd_calibration_screen();
	    break;
   case System_Hv_Read_Screen:
	    lcd_hv_read();
	    break;
   case System_Hv_Write_Screen:
	    lcd_hv_write();
	    break;
   case System_Audio_Mode_Screen:
	    lcd_audio_mode_screen();
	    break;
   case System_Freq_Recv_Screen:
	    lcd_freq_recv_screen();
	    break;
   case System_Freq_Dtc_Screen:
	    lcd_freq_dtc_screen();
	    break;
   case System_Rtc_Screen:
	    lcd_rtc_screen();
	    break;
   case Modify_Password_Screen:
	    lcd_modify_password_screen();
	    break;
   case System_Read_Data:
	    lcd_read_sd_data();
	    break;
   case System_Save_Setting_Screen:
	    lcd_save_setting_screen();
	    break;
   default:
	    lcd_home_screen(CPS_VAL,primary_unit);
	    break;
  }
}

void LCD_SetData(uint8_t data) {

    HAL_GPIO_WritePin(LCD_D0_Port, LCD_D0_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D1_Port, LCD_D1_Pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D2_Port, LCD_D2_Pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D3_Port, LCD_D3_Pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D4_Port, LCD_D4_Pin, (data & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D5_Port, LCD_D5_Pin, (data & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D6_Port, LCD_D6_Pin, (data & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D7_Port, LCD_D7_Pin, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LCD_WriteCmd(uint8_t cmd) {

    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_RESET);  // RS = 0
    HAL_GPIO_WritePin(LCD_RW_Port, LCD_RW_Pin, GPIO_PIN_RESET);  // RW = 0
    LCD_SetData(cmd);
    LCD_EnablePulse();
    HAL_Delay(2);

}

void lcd_print_char(uint8_t data) {

    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_SET);    // RS = 1
    HAL_GPIO_WritePin(LCD_RW_Port, LCD_RW_Pin, GPIO_PIN_RESET);  // RW = 0
    LCD_SetData(data);
    LCD_EnablePulse();
    HAL_Delay(2);
}

void lcd_init(void) {

    HAL_Delay(50);      // Wait for LCD power-up

    LCD_WriteCmd(0x30);
    HAL_Delay(3);       // Function set (8-bit)
    LCD_WriteCmd(0x30);
    HAL_Delay(3);
    LCD_WriteCmd(0x30);
    HAL_Delay(3);

    LCD_WriteCmd(0x38); // Function set: 8-bit, 2-line, 5x8 dots
    HAL_Delay(3);
    LCD_WriteCmd(0x01);
    HAL_Delay(1);
    LCD_WriteCmd(0x0C); // Display ON, cursor OFF
    HAL_Delay(3);
    LCD_WriteCmd(0x06); // Entry mode set
    HAL_Delay(3);
    lcd_clear();        // Clear display

}

void lcd_set_cursor(uint8_t row, uint8_t col) {

    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    LCD_WriteCmd(0x80 | (addr + col));

}

void lcd_cursor_on() {
	LCD_WriteCmd(0x0E); // Display ON, Cursor ON, Blink OFF
}

void lcd_cursor_off() {
	LCD_WriteCmd(0x0C); // Display ON, Cursor OFF, Blink OFF
}

void lcd_print(char *str) {
    while (*str) {
    	lcd_print_char(*str++);
    }
}

void lcd_clear(void) {
    LCD_WriteCmd(0x01); // Clear display
    HAL_Delay(2);
}

void LCD_EnablePulse() {

    HAL_GPIO_WritePin(LCD_E_Port, LCD_E_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_E_Port, LCD_E_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_8) {

		Ack_Button = 1;
	}

	if( GPIO_Pin == GPIO_PIN_9)
	{
		Reset_Button = 1;
	}
}

void refresh_lcd(void)
{
	// --------for refreshing lcd------
	copy_saved_values_to_temp();
	strcpy(prev_temp_primary_unit_str, "");
	strcpy(prev_temp_agm_mode_str,"");
	strcpy(prev_temp_audio_mode_str,"");
    strcpy(prev_temp_freq_recv_str,"");
    strcpy(prev_temp_freq_dtc_str,"");


	// ---------Newly Added------------
	prev_temp_alarm_unit = 0xff;
	strcpy(prev_temp_alarm_val ,"");
	edit_pos_index = 0;
	prev_temp_4_20_min_unit = 0xff;
	strcpy(prev_temp_4_20_min_val ,"");
	prev_temp_4_20_max_unit = 0xff;
	strcpy(prev_temp_4_20_max_val ,"");
	strcpy(prev_temp_4_20_min_cal_val,"");
	strcpy(prev_temp_4_20_max_cal_val,"");
	prev_temp_overload_unit = 0xff;
	strcpy(prev_temp_overload_val ,"");
	prev_temp_overrange_unit = 0xff;
	strcpy(prev_temp_overrange_val ,"");
	strcpy(prev_temp_hv_write, "");
	edit_pos_index_hv = 0;
	num_cursor = 0;
	strcpy(prev_val,"");

	strcpy(prev_temp_baud_rate_str, "");
	eth_ip_cursor = 0;
	strcpy(prev_ip,"");

	strcpy(prev_temp_rtc_date_val,"");
	strcpy(prev_temp_rtc_time_val,"");
}

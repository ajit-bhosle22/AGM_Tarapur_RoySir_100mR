/*
 * Lcd.h
 *
 *  Created on: Jan 27, 2026
 *      Author: Ajit Bhosle
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "main.h"
#include "modbus.h"

#define LCD_RS_Pin  GPIO_PIN_5
#define LCD_RW_Pin  GPIO_PIN_4
#define LCD_E_Pin   GPIO_PIN_6

#define LCD_RS_Port GPIOC
#define LCD_RW_Port GPIOC
#define LCD_E_Port  GPIOA

// LCD data pins D0–D7
#define LCD_D0_Pin  GPIO_PIN_3
#define LCD_D1_Pin  GPIO_PIN_2
#define LCD_D2_Pin  GPIO_PIN_1
#define LCD_D3_Pin  GPIO_PIN_0
#define LCD_D4_Pin  GPIO_PIN_3
#define LCD_D5_Pin  GPIO_PIN_2
#define LCD_D6_Pin  GPIO_PIN_1
#define LCD_D7_Pin  GPIO_PIN_0

#define LCD_D0_Port GPIOA
#define LCD_D1_Port GPIOA
#define LCD_D2_Port GPIOA
#define LCD_D3_Port GPIOA
#define LCD_D4_Port GPIOC
#define LCD_D5_Port GPIOC
#define LCD_D6_Port GPIOC
#define LCD_D7_Port GPIOC

// Switches Data Pins
#define SW1_PIN   GPIO_PIN_10    //PE10
#define SW1_PORT  GPIOE

#define SW2_PIN   GPIO_PIN_13    //PE13
#define SW2_PORT  GPIOE

#define SW3_PIN   GPIO_PIN_10    //PB10
#define SW3_PORT  GPIOB

#define SW4_PIN   GPIO_PIN_11    //PB11
#define SW4_PORT  GPIOB

#define SW5_PIN   GPIO_PIN_5     //PA5
#define SW5_PORT  GPIOA


typedef enum{
	Normal_Flag,
	Dtc_Failed_Flag,
	Hv_Fault_Flag,
	Overrun_Flag,
	Overload_Flag,
	Alarm_Flag,
	Ack_Flag_Alarm,
	Ack_Flag_Overrun,
	Ack_Flag_OverLoad,
}SystemStateFlag;

typedef enum
{
   Home_Screen,
   Password_Screen,
   Setup_Menu_Screen,
   Set_Unit_Screen,
   Set_Alarm_Screen,
   RS485_Setting_Screen,
   Ethernet_Setting_Screen,
   AGM_Mode_Screen,
   Overload_Setting_Screen,
   Overrange_Setting_Screen,
   System_4_20_Min_Screen,
   System_4_20_Max_Screen,
   System_4_20_Min_Cal_Screen,
   System_4_20_Max_Cal_Screen,
   System_Calibration_Screen,
   System_Hv_Read_Screen,
   System_Hv_Write_Screen,
   System_Audio_Mode_Screen,
   System_Freq_Recv_Screen,
   System_Freq_Dtc_Screen,
   System_Rtc_Screen,
   Modify_Password_Screen,
   PC_Ethernet_Setting_Screen,
   System_Read_Data,
   System_Read_Mode,
   System_Save_Setting_Screen
}System_Screen_t;

// -----------------------SAVED VALUES-----------------------------------
extern uint16_t primary_unit;
extern char saved_rs485_slave_id[6];
extern uint8_t baud_rate;
extern char primary_unit_str[12];
extern char baud_rate_str[12];
extern uint8_t alarm_unit;
extern char alarm_val[8];
extern uint8_t overrange_unit;
extern char overrange_val[8];
extern uint8_t overload_unit;
extern char overload_val[8];
extern uint8_t min_4_20_unit;
extern char min_4_20_val[8];
extern uint8_t max_4_20_unit;
extern char max_4_20_val[8];
extern uint8_t agm_mode;
extern char temp_agm_mode_str[12];
extern uint8_t audio_mode;
extern char temp_audio_mode_str[12];
extern uint8_t freq_recv_mode;
extern char temp_freq_recv_str[12];
extern uint8_t freq_dtc_mode;
extern char temp_freq_dtc_str[12];
extern uint8_t sd_mode;
extern char temp_sdmode_str[12];

// ---------------Saved ETH  (FOR DEVICE)---------------------
extern char saved_eth_slave_id[6];
extern char saved_eth_port[6];
extern char saved_eth_ip[16];
extern char saved_eth_subnet[16];
extern char saved_eth_gateway[16];

// --------------Saved ETH (FOR PC)--------------------------
extern char saved_eth_port_pc[6];
extern char saved_eth_ip_pc[16];

extern char correct_password[5] ;
// --------------Saved Calibration--------------
extern char saved_factor_values[4][6];
extern char cal_4mA_val[8];
extern char cal_20mA_val[8];

extern System_Screen_t lcd_screen;
extern System_Screen_t prev_lcd_screen;
extern SystemStateFlag last_system_state;
extern SystemStateFlag State_Flag;
extern SystemStateFlag prevState;

extern float hv_val_dtc ;
extern bool write_hv_val_dtc ;
extern uint16_t last_unit;
extern volatile uint8_t saved_sn_ir;

void LCD_SetData(uint8_t);
void LCD_WriteCmd(uint8_t);
void lcd_print_char(uint8_t);
void lcd_init(void);
void lcd_set_cursor(uint8_t, uint8_t);
void lcd_cursor_on();
void lcd_cursor_off();
void lcd_print(char*);
void lcd_clear(void);
void lcd_print_float(float);
void LCD_EnablePulse();
void system_lcd_screen(void);
void lcd_unit_screen(void);
void lcd_alarm_screen(void);
void lcd_rs485_screen(void);
void lcd_ethernet_screen(void);
void lcd_agm_mode_screen(void);
void lcd_overload_screen(void);
void lcd_overrange_screen(void);
void lcd_4_20_max_cal_screen(void);
void lcd_4_20_min_cal_screen(void);
void lcd_modify_password_screen(void);
void lcd_read_sd_data_init(void);
void lcd_read_sd_data(void);
void pwd_sw4_next_digit(void);
void pwd_sw2_decrement(void);
void pwd_sw1_increment(void);
void lcd_password_screen_init(void);
void check_switch_status(void);
void build_password_string(void);
void lcd_setup_menu_screen_init();
void lcd_alarm_screen_unit(void);
void select_lcd_screen(void);
void lcd_save_setting_screen(void);
void select_unit(uint8_t);
void select_value(char*);
void select_ip_value(char*);
void select_unit_generic(uint8_t*, uint8_t);
void select_value_generic(uint8_t*);
void lcd_4_20_min_screen_init(void);
void GetUnitString_print(uint8_t, char*);
void lcd_agm_mode_screen_init(void);
void lcd_numeric_edit_screen(char*, char*);
void lcd_ip_edit_screen(char*, char*);
void rs485_menu_screen(void);
void rs485_slave_id_screen(void);
void rs485_baud_rate_screen(void);
void rs485_menu_screen_init(void);
void rs485_baud_rate_screen_init(void);
void select_alarm_format_val(char*,uint8_t*);
void select_4_20_cal_format_val(char*);
void lcd_save_setting_screen(void);
void copy_saved_values(void);
void copy_saved_values_to_temp(void);
void load_all_factors(void);
void lcd_factor_edit_screen_init(void);
void lcd_print_int(uint8_t);
void lcd_update_pwd_digit(uint8_t);
void lcd_home_screen(uint32_t, uint16_t);
void lcd_password_screen(void);
void lcd_setup_menu_screen(void);
void increment_digit_at_position(char*, uint8_t);
void decrement_digit_at_position(char*, uint8_t);
void unit_panel_led(void);
void select_hv_val(char* val);
uint8_t unit_is_integer(uint8_t unit);
void copy_temp_to_saved_values(void);
void calib_factors_hv_write(void);
void lcd_home_screen_update(uint32_t, uint16_t);
bool check_calib_fact_change(void);
bool verify_password(void);
void refresh_lcd(void);
void lcd_freq_recv_screen(void);
void lcd_freq_recv_screen_init(void);
void lcd_freq_dtc_screen(void);
void lcd_freq_dtc_screen_init(void);
void select_rtc_format_val(char *val);
void select_rtc_time_format_val(char *val);
void select_rtc_date_format_val(char *val);
uint8_t Get_RTC_WeekDay(uint8_t, uint8_t, uint16_t);
void lcd_numeric_edit_screen_pc(char *title, char *value);
void lcd_ip_edit_screen_pc(char *title, char *ip);
void display_server_failed_status(void);
void display_data_writing(void);
void display_data_writing_done(void);
void display_server_wait(void);
bool IsLeapYear(uint16_t year);

#endif /* INC_LCD_H_ */

#ifndef FAULT_HANDLER_H
#define FAULT_HANDLER_H

#include <stdbool.h>
#include "stm32h7xx_hal.h"

#define HV_MIN 400.0f
#define HV_MAX 700.0f

typedef enum
{
    UNIT_uSV_H,
    UNIT_mSV_H,
    UNIT_SV_H,
    UNIT_mR_H,
    UNIT_R_H,
	UNIT_CPS,
	UNIT_CPM,
    UNIT_INVALID
} unit_t;


extern volatile uint8_t Alarm_flag;
extern volatile uint8_t Detector_failure_flag;
extern volatile uint8_t OverLoad_flag;

extern volatile bool Dtc_Failed_Status;
extern volatile bool Prev_Dtc_Failed_Status;
extern volatile bool Pc_RTU_Failed_Status;
extern volatile bool Pc_TCP_Failed_Status;

extern volatile uint16_t Reset_Button;
extern volatile uint16_t Ack_Button;
extern volatile uint8_t Dtc_Failed_Timeout;
extern volatile uint8_t Pc_RTU_Failed_Timeout;
extern volatile uint8_t Pc_TCP_Failed_Timeout;

extern float Hv_Voltage;
extern uint32_t Hv_Val;
extern uint32_t CPS_VAL;
extern volatile float radiation_uSv;
extern uint32_t boot_up_time_tick_hv;
extern const float factor_thresholds[20];

void Check_Fault_Conditions(void);
void Handle_AlarmState(void);
void Handle_HvFailState(void);
void Handle_DtcFailState(void);
void Handle_OverRunState(void);
void Handle_OverLoadState(void);
void Handle_NormalState(void);
void Handle_AckState(void);
void ProcessSystem(void);
void Buzzer_On(void);
void Buzzer_Off(void);
void LED_Red_On(void) ;
void LED_Red_Off(void);
void LED_Red_Toggle(void);
void LED_Green_On(void);
void LED_Green_Off(void);
void LED_Yellow_On(void);
void LED_Yellow_Off(void);
void LED_Yellow_Toggle(void);
void Relay1_On(void);
void Relay1_Off(void);

void Panel_LED_mRh_On(void);
void Panel_LED_mRh_Off(void);
void Panel_LED_uSv_On(void);
void Panel_LED_uSv_Off(void);
void Panel_LED_CPS_On(void);
void Panel_LED_CPS_Off(void);
void Panel_LED_CPM_On(void);
void Panel_LED_CPM_Off(void);
void Panel_LED_Off(void);
void monitor_panel_unit_leds(void);

void Check_RS485_TCP_Communication(void);
void toggle_buzzer_red_yellow_leds(void);
float Convert_To_uSv(float value, unit_t unit);
unit_t Get_Unit(uint8_t unit);

#endif


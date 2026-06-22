#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "main_global.h"
#include "Fault_Handler.h"
#include "Modbus.h"
#include "lcd.h"
#include "tcp_server_registers.h"
#include "Detector_Modbus_registers.h"
#include "App_7_segment.h"

#define SET_REG(field, val) \
    do { \
        Modbus_Registers.field = (val); \
        Modbus_Registers_PC_TCP.field = (val); \
    } while(0)

// ------------------Default System State Normal---------------------------
SystemStateFlag State_Flag = Normal_Flag;
SystemStateFlag prevState  = Normal_Flag;

volatile bool Dtc_Failed_Status = false;
volatile bool Prev_Dtc_Failed_Status = false;
volatile uint8_t Dtc_Failed_Timeout = 0;
volatile bool Pc_RTU_Failed_Status = true;
volatile bool Pc_TCP_Failed_Status = true;
volatile bool Prev_Pc_RTU_Failed_Status = false;
volatile bool Prev_Pc_TCP_Failed_Status = false;
volatile uint8_t Pc_RTU_Failed_Timeout = 0;
volatile uint8_t Pc_TCP_Failed_Timeout = 0;
volatile bool Ack_Status = false;
volatile bool Reset_Status = false;

volatile uint16_t Reset_Button            =  0;
volatile uint16_t Ack_Button              =  0;

uint32_t Hv_Val = 0;
float Hv_Voltage = 0.0f;

uint32_t CPS_VAL;

char Current_Agm_Mode[10] = "";
volatile float radiation_uSv;

void Buzzer_On(void)          { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET); }
void Buzzer_Off(void)         { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); }

void LED_Red_On(void)         { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); }
void LED_Red_Off(void)        { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); }
void LED_Red_Toggle(void)     { HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); }

void LED_Green_On(void)       { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); }
void LED_Green_Off(void)      { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); }

void LED_Yellow_On(void)      { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); }
void LED_Yellow_Off(void)     { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); }
void LED_Yellow_Toggle(void)  { HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);}

// -------------------Panel LED's-----------------------------------------
void Panel_LED_CPM_On(void)   { HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);}   //LED4
void Panel_LED_CPM_Off(void)  { HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);}
void Panel_LED_uSv_On(void)   { HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_RESET);} //LED2
void Panel_LED_uSv_Off(void)  { HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_SET);}
void Panel_LED_CPS_On(void)   { HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);}   //LED3
void Panel_LED_CPS_Off(void)  { HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);}
void Panel_LED_mRh_On(void)   { HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_RESET);} //LED1
void Panel_LED_mRh_Off(void)  { HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_SET);}

// --------------Relay Monitoring-----------------
void Relay1_On(void)          {	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_SET);}
void Relay1_Off(void)         {	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_RESET);}


unit_t Get_Unit(uint8_t unit)
{
	switch(unit)
	{
	  case 0:
		 return UNIT_mR_H;
	  case 1:
		 return UNIT_uSV_H;
	  case 2:
		 return UNIT_CPS;
	  case 3:
		 return UNIT_CPM;
	  default:
		 return UNIT_mR_H;
	}
}

float Convert_To_uSv(float value, unit_t unit)
{
	switch(unit)
	{
	case UNIT_uSV_H:
		return value;
	case UNIT_mR_H:
		return value * 10.0f;
	case UNIT_CPS:
		 return (value / GM_TUBE_SENSITIVITY) * 10.0f;
	case UNIT_CPM:
		 return ((cpm/60) / GM_TUBE_SENSITIVITY) * 10.0f;
	default:
		return 0.0f;
	}
}

void Check_Fault_Conditions(void)
{
	__disable_irq();

	uint8_t ack_pressed   = Ack_Button;
	uint8_t reset_pressed = Reset_Button;

	Ack_Button   = 0;
	Reset_Button = 0;

	__enable_irq();

	static uint32_t last_tick_time_ack=0;
	static uint32_t boot_up_time_tick_hv = 0;
	double alarm_threshold;
	double overload_threshold;
	double overrun_threshold;
    if(boot_up_time_tick_hv == 0)
    {
    	boot_up_time_tick_hv = HAL_GetTick();
    }

	// --------------Convert Dose_mR to uSv/h------------
	radiation_uSv = dose_mRh * 10.0f;

	// --------------Alarm Fault Condition----------------
	unit_t alarm_unit_t = Get_Unit(alarm_unit);
	alarm_threshold = Convert_To_uSv(atof(alarm_val), alarm_unit_t);

	// --------------Overload Fault Condition-------------
	unit_t overload_unit_t = Get_Unit(overload_unit);
	overload_threshold = Convert_To_uSv(atof(overload_val),overload_unit_t);

	// --------------Overrange Fault Condition------------
	unit_t overrun_unit_t = Get_Unit(overrange_unit);
	overrun_threshold = Convert_To_uSv(atof(overrange_val),overrun_unit_t);

	// ---------------Check Detector Failed --------------
	if(Dtc_Failed_Timeout >= 3)
	{
		Dtc_Failed_Status = true;
	}

	if(Dtc_Failed_Status!=Prev_Dtc_Failed_Status)
	{
		if(Dtc_Failed_Status==true){

			State_Flag = Dtc_Failed_Flag;

			SET_REG(Dtc_Fault,1);
			SET_REG(Reset,0);
			SET_REG(ACK,0);
			SET_REG(Overload,0);
			SET_REG(Overrun,0);
			SET_REG(Alarm,0);
			SET_REG(Hv_Fault,0);
		}
		if(Dtc_Failed_Status==false){

			State_Flag = Normal_Flag;

			SET_REG(Reset,1);
			SET_REG(Dtc_Fault,0);
			SET_REG(ACK,0);
			SET_REG(Overload,0);
			SET_REG(Overrun,0);
			SET_REG(Alarm,0);
			SET_REG(Hv_Fault,0);

			if(relay_status == 1)
			{
				prev_relay_status = 0;
			}
			else
			{
				prev_relay_status = 1;
			}

			boot_up_time_tick_hv = HAL_GetTick();
		}
		Prev_Dtc_Failed_Status=Dtc_Failed_Status;
	}

	if(Dtc_Failed_Status == true)
	{
		// -----DTC Failed Reset Dose Rate----
		CPS_VAL = 0;
		radiation_uSv = 0;

		// ------Reset CPS/HV Voltage---------
		Modbus_Registers_Detector.CPS_MSB = 0;
		Modbus_Registers_Detector.CPS_LSB = 0;
		Modbus_Registers_Detector.HV_MSB  = 0;
		Modbus_Registers_Detector.HV_LSB  = 0;
	}

	if(radiation_uSv >= overload_threshold && State_Flag != Overload_Flag && State_Flag != Ack_Flag_OverLoad
	&& State_Flag != Dtc_Failed_Flag)
	{
		State_Flag=Overload_Flag;

		SET_REG(Overload,1);
		SET_REG(Alarm,0);
		SET_REG(Overrun,0);
		SET_REG(Reset,0);
		SET_REG(ACK,0);
		SET_REG(Hv_Fault,0);

		// ----To Auto Ack After One Min----
		last_tick_time_ack=HAL_GetTick();
	}
	else if(radiation_uSv >= overrun_threshold && State_Flag !=Overrun_Flag  && State_Flag != Overload_Flag
	&& State_Flag != Ack_Flag_OverLoad && State_Flag != Ack_Flag_Overrun  && State_Flag != Dtc_Failed_Flag)
	{
		State_Flag=Overrun_Flag;

		SET_REG(Overrun,1);
		SET_REG(Alarm,0);
		SET_REG(Reset,0);
		SET_REG(ACK,0);
		SET_REG(Hv_Fault,0);

		// -----To Auto Ack After One Min----
		last_tick_time_ack=HAL_GetTick();
	}
	else if(radiation_uSv >= alarm_threshold && State_Flag != Alarm_Flag
	&& State_Flag != Ack_Flag_Alarm && State_Flag != Overload_Flag
	&& State_Flag != Ack_Flag_OverLoad && State_Flag != Overrun_Flag
	&& State_Flag != Ack_Flag_Overrun && State_Flag != Dtc_Failed_Flag)
	{
		State_Flag=Alarm_Flag;

		SET_REG(Alarm,1);
		SET_REG(Reset,0);
		SET_REG(Hv_Fault,0);

		// -----To Auto Ack After One Min-----
		last_tick_time_ack=HAL_GetTick();
	}
	else if((Hv_Voltage < HV_MIN || Hv_Voltage > HV_MAX) && State_Flag == Normal_Flag
	&& (HAL_GetTick()-boot_up_time_tick_hv>5000) && State_Flag != Dtc_Failed_Flag)
	{
		State_Flag = Hv_Fault_Flag;

		SET_REG(Hv_Fault,1);
		SET_REG(Reset,0);

		boot_up_time_tick_hv = 0;
	}

	if(State_Flag == Hv_Fault_Flag && ((Hv_Voltage > HV_MIN) && (Hv_Voltage < HV_MAX)))
	{
		State_Flag = Normal_Flag;

		SET_REG(Hv_Fault,0);
		SET_REG(Reset,1);
	}


	/* check ACK condition
	 * agm_mode = 0 --> Manual , agm_mode = 1 ---> auto
	 */
	if((State_Flag == Overload_Flag||State_Flag == Overrun_Flag||State_Flag == Alarm_Flag)
	&& (ack_pressed == 1 || (agm_mode == 1 && HAL_GetTick() - last_tick_time_ack > 60000))
	&& State_Flag != Dtc_Failed_Flag)
	{
		if(State_Flag == Alarm_Flag)
		{
			State_Flag = Ack_Flag_Alarm;
		}

		if(State_Flag == Overrun_Flag)
		{
			State_Flag = Ack_Flag_Overrun;
		}

		if(State_Flag == Overload_Flag)
		{
			State_Flag = Ack_Flag_OverLoad;
		}

		SET_REG(ACK,1);

		// To Auto Ack After One Min
		last_tick_time_ack = HAL_GetTick();

	}

	// -------------------Normal Condition---------------------------------
	if(((State_Flag == Ack_Flag_OverLoad && radiation_uSv < overload_threshold)||
	(State_Flag == Ack_Flag_Alarm && radiation_uSv < alarm_threshold )
	||(State_Flag == Ack_Flag_Overrun && radiation_uSv < overrun_threshold)) &&
	(reset_pressed == 1 || agm_mode == 1) && State_Flag != Dtc_Failed_Flag)
	{
		State_Flag = Normal_Flag;

		SET_REG(Reset,1);
		SET_REG(ACK,0);
		SET_REG(Overload,0);
		SET_REG(Overrun,0);
		SET_REG(Alarm,0);
		SET_REG(Hv_Fault,0);
	}

	ProcessSystem();
	toggle_buzzer_red_yellow_leds();
}

void toggle_buzzer_red_yellow_leds(void)
{
	static uint32_t last_tick_time_ms=0;
	static uint32_t last_tick_time_yellow=0;
	static uint32_t last_tick_time_red=0;
	if ((prevState == Alarm_Flag || prevState == Overload_Flag ||  prevState == Overrun_Flag) && (HAL_GetTick()-last_tick_time_ms>500))
	{
		last_tick_time_ms=HAL_GetTick();
		if(Alarm_OneSec_Generated == false)
		{
			Buzzer_Off();
		}
		else
		{
			Buzzer_On();
			Alarm_OneSec_Generated = false;
		}
	}

	if ((prevState == Alarm_Flag || prevState == Overload_Flag ||  prevState == Overrun_Flag) && (HAL_GetTick()-last_tick_time_red>200))
	{
		last_tick_time_red = HAL_GetTick();
		LED_Red_Toggle();
	}

	if((prevState == Dtc_Failed_Flag || prevState == Hv_Fault_Flag) && (HAL_GetTick()-last_tick_time_yellow>200))
	{
		last_tick_time_yellow = HAL_GetTick();
		LED_Yellow_Toggle();
	}
}

void ProcessSystem(void)
{
    if (State_Flag != prevState)
    {
        prevState = State_Flag;
        switch (State_Flag)
        {
            case Alarm_Flag:          Handle_AlarmState(); break;
            case Dtc_Failed_Flag:     Handle_DtcFailState(); break;
            case Overload_Flag:       Handle_OverLoadState(); break;
            case Overrun_Flag:        Handle_OverRunState();  break;
            case Ack_Flag_OverLoad:   Handle_AckState(); break;
            case Ack_Flag_Alarm:      Handle_AckState(); break;
            case Ack_Flag_Overrun:    Handle_AckState(); break;
            case Normal_Flag:         Handle_NormalState(); break;
            case Hv_Fault_Flag:       Handle_HvFailState();break;
        }
    }
}

void Handle_AlarmState(void)
{
    LED_Red_On();
	LED_Green_Off();
	LED_Yellow_Off();
	Buzzer_On();
	Relay1_Off();
}

void Handle_HvFailState(void)
{
	LED_Red_Off();
	LED_Green_Off();
	LED_Yellow_On();
	Buzzer_Off();
	Relay1_Off();
}

void Handle_DtcFailState(void)
{
	LED_Red_Off();
	LED_Green_Off();
	LED_Yellow_On();
	Buzzer_Off();
	Relay1_Off();
}

void Handle_OverLoadState(void)
{
	LED_Red_On();
	LED_Green_Off();
	LED_Yellow_Off();
	Buzzer_On();
	Relay1_Off();
}

void Handle_OverRunState(void)
{
	LED_Red_On();
	LED_Green_Off();
	LED_Yellow_Off();
	Buzzer_On();
	Relay1_Off();
}

void Handle_AckState(void)
{
	LED_Red_On();
	LED_Green_Off();
	LED_Yellow_Off();
	Buzzer_Off();
	Relay1_Off();
}

void Handle_NormalState(void)
{
	LED_Red_Off();
	LED_Green_On();
	LED_Yellow_Off();
	Buzzer_Off();
	Relay1_On();
}

void Panel_LED_Off(void)
{
	Panel_LED_mRh_Off();
	Panel_LED_uSv_Off();
	Panel_LED_CPS_Off();
	Panel_LED_CPM_Off();
}

void monitor_panel_unit_leds(void)
{
	static uint8_t prev_primary_unit = 0xf;
	if(prev_primary_unit != primary_unit){
		if(primary_unit == 0)       //mR/h
		{
			Panel_LED_mRh_On();
			Panel_LED_uSv_Off();
			Panel_LED_CPS_Off();
			Panel_LED_CPM_Off();
		}
		else if(primary_unit == 1)  //uSv
		{
			Panel_LED_mRh_Off();
			Panel_LED_uSv_On();
			Panel_LED_CPS_Off();
			Panel_LED_CPM_Off();
		}
		else if(primary_unit == 2)  //cps
		{
			Panel_LED_mRh_Off();
			Panel_LED_uSv_Off();
			Panel_LED_CPS_On();
			Panel_LED_CPM_Off();
		}
		else                        //cpm
		{
			Panel_LED_mRh_Off();
			Panel_LED_uSv_Off();
			Panel_LED_CPS_Off();
			Panel_LED_CPM_On();
		}

		prev_primary_unit = primary_unit;
	}
}

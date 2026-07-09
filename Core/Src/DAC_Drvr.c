
#include <stdlib.h>
#include <stdbool.h>
#include "main.h"
#include "main_global.h"
#include "modbus.h"
#include "lcd.h"
#include "AT24CM01_Eeprom.h"
#include "Detector_Modbus_registers.h"
#include "Fault_Handler.h"
#include "pc_modbus.h"
#include "DAC_Drvr.h"

float g_cal_a = 0.0f;   // counts/mA
float g_cal_b = 0.0f;   // offset

#define DAC_4MA_CODE    718
#define DAC_20MA_CODE   3607

void Two_Point_Calibrate_DAC(float measured_4mA, float measured_20mA)
{
	// Realistic hardware drift range
	if (measured_4mA  < 3.5f || measured_4mA  > 4.5f)   return ;  // ±0.5mA around 4mA
	if (measured_20mA < 19.0f || measured_20mA > 21.0f) return ;  // ±1.0mA around 20mA

    bool has_4ma  = (measured_4mA  > 0.0f);
    bool has_20ma = (measured_20mA > 0.0f);

    float code1 = DAC_4MA_CODE;
    float code2 = DAC_20MA_CODE;
    float i1    = has_4ma  ? measured_4mA  : 4.0f;
    float i2    = has_20ma ? measured_20mA : 20.0f;

    g_cal_a = (code2 - code1) / (i2 - i1);
    g_cal_b = code1 - (g_cal_a * i1);
}

void Set_4to20MA(void)
{
    float min_uSv = Convert_To_uSv(atof(min_4_20_val), Get_Unit(min_4_20_unit));
    float max_uSv = Convert_To_uSv(atof(max_4_20_val), Get_Unit(max_4_20_unit));

    // Clamp
    if (radiation_uSv < min_uSv) radiation_uSv = min_uSv;
    if (radiation_uSv > max_uSv) radiation_uSv = max_uSv;

    // Calculate target mA (4-20mA linear)
    float percent  = (radiation_uSv - min_uSv) / (max_uSv - min_uSv);
    float target_mA = 4.0f + (percent * 16.0f);

    // Convert mA to DAC using calibrated line
    uint32_t dac_val = (uint32_t)(g_cal_a * target_mA + g_cal_b);

    // Fault condition  22.5mA
    if(State_Flag != Normal_Flag)
    {
    	 dac_val = (uint32_t)(g_cal_a * 22.5f + g_cal_b);
    }
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,dac_val);
}

void e4_20mA_task(void)
{
	if(g_1s_flags.current_4_20mA == true){
		g_1s_flags.current_4_20mA = false;
		Set_4to20MA();
	}
}

/*
 * DAC_Drvr.h
 *
 */

#ifndef INC_FOURTOTWENTY_DRVR_DAC_DRVR_H_
#define INC_FOURTOTWENTY_DRVR_DAC_DRVR_H_

extern float g_cal_a ;  // counts/mA
extern float g_cal_b ;  // offset

void Set_4to20MA_Dtc1(float mr_hr);
void Set_4to20MA_Dtc2(float mr_hr);
void Set_4to20MA(void);
void Two_Point_Calibrate_DAC(float measured_4mA, float measured_20mA);
void e4_20mA_task(void);

#endif /* INC_FOURTOTWENTY_DRVR_DAC_DRVR_H_ */

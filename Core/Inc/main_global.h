/*
 * main_global.h
 *
 *  Created on: Apr 25, 2026
 *      Author: admin
 */

#ifndef INC_MAIN_GLOBAL_H_
#define INC_MAIN_GLOBAL_H_

#include <stdbool.h>
#include "stm32h7xx_hal.h"

typedef struct
{
    bool dtc_poll;
    bool eth_status;
    bool current_4_20mA;
    bool pc_poll;
    bool rtc_poll;
    bool log_sd_card;
} one_sec_flags_t;

extern volatile one_sec_flags_t g_1s_flags;
extern volatile uint8_t relay_status ;
extern volatile uint8_t prev_relay_status ;
extern volatile bool Alarm_OneSec_Generated;
extern bool sw5_press_flag ;
extern RTC_HandleTypeDef hrtc;
extern DAC_HandleTypeDef hdac1;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c4;

void RTC_SetDefaultTimeDate(void);

#endif /* INC_MAIN_GLOBAL_H_ */

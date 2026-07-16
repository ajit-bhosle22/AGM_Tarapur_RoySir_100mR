#ifndef APP_7_segment_H__
#define APP_7_segment_H__

#include "stm32h7xx_hal.h"

#define MAX7219_CS_PORT       GPIOE
#define MAX7219_CS_PIN        GPIO_PIN_11  // CS

#define MAX7219_CS_LOW()      HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_RESET)
#define MAX7219_CS_HIGH()     HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET)

void Error_Handler(void);
void MAX7219_DisplayNumber(uint32_t number);
void MAX7219_Init(void);
void Test_SPI_Send(void);
void MAX7219_DisplayFloat(uint32_t number, int decimal_pos);
uint8_t MAX7219_CharToSeg(char c);
void MAX7219_DisplayString(const char *str);

#endif

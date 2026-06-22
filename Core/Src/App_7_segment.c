
#include <stdint.h>
#include <string.h>
#include "App_7_segment.h"
#include "main_global.h"

HAL_StatusTypeDef MAX7219_Send(uint8_t address, uint8_t data)
{
    uint8_t tx[2] = {address, data};
    MAX7219_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi4, tx, 2, 100);
    MAX7219_CS_HIGH();
    return status;
}

void MAX7219_Init(void)
{
    if (MAX7219_Send(0x0F, 0x00) != HAL_OK) Error_Handler(); // Display test off
    if (MAX7219_Send(0x0C, 0x01) != HAL_OK) Error_Handler(); // Normal mode
    if (MAX7219_Send(0x0B, 0x05) != HAL_OK) Error_Handler(); // Scan limit = 6 digits
    if (MAX7219_Send(0x09, 0xFF) != HAL_OK) Error_Handler(); // Decode all digits
    if (MAX7219_Send(0x0A, 0x04) != HAL_OK) Error_Handler(); // Medium intensity
}

void MAX7219_DisplayNumber(uint32_t number)
{

	    //clear all 8 digits
	    for (int i = 1; i <= 6; i++) {
	        MAX7219_Send(i, 0xF);
	    }

	    for (int pos = 6; pos >= 1; pos--) {
	        uint8_t digit = number % 10;
	        MAX7219_Send(pos, digit);
	        number /= 10;
	    }
}

void MAX7219_DisplayFloat(uint32_t number, int decimal_pos)
{
    // Clear all Digits
    for (int i = 1; i <= 6; i++) {
        MAX7219_Send(i, 0xF);
    }

    for (int pos = 6; pos >= 1; pos--) {
        uint8_t digit = number % 10;
        uint8_t data = digit;

        // Add decimal point if this is the position
        if ((6 - pos) == decimal_pos && decimal_pos > 0) {
            data |= 0x80;
        }

        MAX7219_Send(pos, data);
        number /= 10;
    }
}

void Test_SPI_Send(void)
 {
     uint8_t test_data[2] = {0x0F, 0x01};                    // Display test off

     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);

     HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi4, test_data, 2, 100);

     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);

     if (status == HAL_OK)
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);

 }




#include <stdint.h>
#include <string.h>
#include "App_7_segment.h"
#include "main_global.h"

static HAL_StatusTypeDef MAX7219_Send(uint8_t address, uint8_t data);

void MAX7219_Init(void)
{
    if (MAX7219_Send(0x0F, 0x00) != HAL_OK) Error_Handler(); // Display test off
    if (MAX7219_Send(0x0C, 0x01) != HAL_OK) Error_Handler(); // Normal mode
    if (MAX7219_Send(0x0B, 0x05) != HAL_OK) Error_Handler(); // Scan limit = 6 digits
    if (MAX7219_Send(0x09, 0x00) != HAL_OK) Error_Handler(); // Decode OFF (raw segments)
    if (MAX7219_Send(0x0A, 0x04) != HAL_OK) Error_Handler(); // Medium intensity
}

// Bit order: DP A B C D E F G
uint8_t MAX7219_CharToSeg(char c)
{
    switch (c) {
        // Digits
        case '0': return 0x7E;
        case '1': return 0x30;
        case '2': return 0x6D;
        case '3': return 0x79;
        case '4': return 0x33;
        case '5': return 0x5B;
        case '6': return 0x5F;
        case '7': return 0x70;
        case '8': return 0x7F;
        case '9': return 0x7B;

        // Letters needed for ALARM / OVR / OFL / HV FL / DTC FL
        case 'A': case 'a': return 0x77;
        case 'L': case 'l': return 0x0E;
        case 'R': case 'r': return 0x05;   // shown as lowercase "r"
        case 'M': case 'm': return 0x15;   // ⚠ approximation, see note below
        case 'O': case 'o': return 0x7E;   // same shape as '0'
        case 'V': case 'v': return 0x3E;   // ⚠ looks identical to 'U', see note
        case 'F': case 'f': return 0x47;
        case 'H': case 'h': return 0x37;
        case 'D': case 'd': return 0x3D;   // shown as lowercase "d"
        case 'T': case 't': return 0x0F;
        case 'C': case 'c': return 0x4E;
        case 'K': case 'k': return 0x27;    // B + E + F + G

        case '-': return 0x01;
        case ' ': default: return 0x00;    // blank
    }
}

// Bit order: DP A B C D E F G
//uint8_t MAX7219_CharToSeg(char c)
//{
//    switch (c) {
//        case '0': return 0x7E;
//        case '1': return 0x30;
//        case '2': return 0x6D;
//        case '3': return 0x79;
//        case '4': return 0x33;
//        case '5': return 0x5B;
//        case '6': return 0x5F;
//        case '7': return 0x70;
//        case '8': return 0x7F;
//        case '9': return 0x7B;
//
//        case 'D': case 'd': return 0x3D;
//        case 'T': case 't': return 0x0F;
//        case 'C': case 'c': return 0x4E;
//
//        case '-': return 0x01;
//        case ' ': default: return 0x00; // blank
//    }
//}

void MAX7219_DisplayString(const char *str)
{
    // clear all 6 digits first
    for (int i = 1; i <= 6; i++) {
        MAX7219_Send(i, 0x00);
    }

    int len = strlen(str);
    // right-align like your number function did (pos 6 = rightmost)
    int pos = 6;
    for (int i = len - 1; i >= 0 && pos >= 1; i--) {
        MAX7219_Send(pos, MAX7219_CharToSeg(str[i]));
        pos--;
    }
}

void MAX7219_DisplayNumber(uint32_t number)
{
    for (int i = 1; i <= 6; i++) {
        MAX7219_Send(i, 0x00); // clear
    }

    for (int pos = 6; pos >= 1; pos--) {
        uint8_t digit = number % 10;
        MAX7219_Send(pos, MAX7219_CharToSeg('0' + digit));
        number /= 10;
    }
}

static HAL_StatusTypeDef MAX7219_Send(uint8_t address, uint8_t data)
{
    uint8_t tx[2] = {address, data};
    MAX7219_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi4, tx, 2, 100);
    MAX7219_CS_HIGH();
    return status;
}

void MAX7219_DisplayFloat(uint32_t number, int decimal_pos)
{
    // Clear all Digits
    for (int i = 1; i <= 6; i++) {
        MAX7219_Send(i, 0x00);
    }

    for (int pos = 6; pos >= 1; pos--) {
        uint8_t digit = number % 10;
        uint8_t data = digit;

        // Add decimal point if this is the position
        if ((6 - pos) == decimal_pos && decimal_pos > 0) {
            data |= 0x80;
        }

        MAX7219_Send(pos, MAX7219_CharToSeg('0'+data));
        number /= 10;
    }
}

//
//void MAX7219_Init(void)
//{
//    if (MAX7219_Send(0x0F, 0x00) != HAL_OK) Error_Handler(); // Display test off
//    if (MAX7219_Send(0x0C, 0x01) != HAL_OK) Error_Handler(); // Normal mode
//    if (MAX7219_Send(0x0B, 0x05) != HAL_OK) Error_Handler(); // Scan limit = 6 digits
//    if (MAX7219_Send(0x09, 0xFF) != HAL_OK) Error_Handler(); // Decode all digits
//    if (MAX7219_Send(0x0A, 0x04) != HAL_OK) Error_Handler(); // Medium intensity
//}
//
//void MAX7219_DisplayNumber(uint32_t number)
//{
//
//	    //clear all 8 digits
//	    for (int i = 1; i <= 6; i++) {
//	        MAX7219_Send(i, 0xF);
//	    }
//
//	    for (int pos = 6; pos >= 1; pos--) {
//	        uint8_t digit = number % 10;
//	        MAX7219_Send(pos, digit);
//	        number /= 10;
//	    }
//}

//void MAX7219_DisplayFloat(uint32_t number, int decimal_pos)
//{
//    // Clear all Digits
//    for (int i = 1; i <= 6; i++) {
//        MAX7219_Send(i, 0xF);
//    }
//
//    for (int pos = 6; pos >= 1; pos--) {
//        uint8_t digit = number % 10;
//        uint8_t data = digit;
//
//        // Add decimal point if this is the position
//        if ((6 - pos) == decimal_pos && decimal_pos > 0) {
//            data |= 0x80;
//        }
//
//        MAX7219_Send(pos, data);
//        number /= 10;
//    }
//}

void Test_SPI_Send(void)
 {
     uint8_t test_data[2] = {0x0F, 0x01};                    // Display test off

     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);

     HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi4, test_data, 2, 100);

     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);

     if (status == HAL_OK)
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);

 }



/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_1_Pin GPIO_PIN_2
#define LED_1_GPIO_Port GPIOE
#define LED_2_Pin GPIO_PIN_3
#define LED_2_GPIO_Port GPIOE
#define SPK_1_Pin GPIO_PIN_5
#define SPK_1_GPIO_Port GPIOE
#define SPK_2_Pin GPIO_PIN_6
#define SPK_2_GPIO_Port GPIOE
#define LCD_D7_Pin GPIO_PIN_0
#define LCD_D7_GPIO_Port GPIOC
#define LCD_D6_Pin GPIO_PIN_1
#define LCD_D6_GPIO_Port GPIOC
#define LCD_D6C2_Pin GPIO_PIN_2
#define LCD_D6C2_GPIO_Port GPIOC
#define LCD_D4_Pin GPIO_PIN_3
#define LCD_D4_GPIO_Port GPIOC
#define LCD_D3_Pin GPIO_PIN_0
#define LCD_D3_GPIO_Port GPIOA
#define LCD_D2_Pin GPIO_PIN_1
#define LCD_D2_GPIO_Port GPIOA
#define LCD_D1_Pin GPIO_PIN_2
#define LCD_D1_GPIO_Port GPIOA
#define LCD_D0_Pin GPIO_PIN_3
#define LCD_D0_GPIO_Port GPIOA
#define SW_5_Pin GPIO_PIN_5
#define SW_5_GPIO_Port GPIOA
#define LCD_E_Pin GPIO_PIN_6
#define LCD_E_GPIO_Port GPIOA
#define W5500_INT_Pin GPIO_PIN_7
#define W5500_INT_GPIO_Port GPIOA
#define LCD_RW_Pin GPIO_PIN_4
#define LCD_RW_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_5
#define LCD_RS_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_0
#define LED3_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_2
#define LCD_RST_GPIO_Port GPIOB
#define UART7_RE_Pin GPIO_PIN_9
#define UART7_RE_GPIO_Port GPIOE
#define SW1_Pin GPIO_PIN_10
#define SW1_GPIO_Port GPIOE
#define SPI4_CS_Pin GPIO_PIN_11
#define SPI4_CS_GPIO_Port GPIOE
#define SW_2_Pin GPIO_PIN_13
#define SW_2_GPIO_Port GPIOE
#define SW_3_Pin GPIO_PIN_10
#define SW_3_GPIO_Port GPIOB
#define SW_4_Pin GPIO_PIN_11
#define SW_4_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_12
#define LED4_GPIO_Port GPIOB
#define MC_OP6_Pin GPIO_PIN_13
#define MC_OP6_GPIO_Port GPIOB
#define MC_OP5_Pin GPIO_PIN_9
#define MC_OP5_GPIO_Port GPIOD
#define MC_OP4_Pin GPIO_PIN_10
#define MC_OP4_GPIO_Port GPIOD
#define MC_OP3_Pin GPIO_PIN_11
#define MC_OP3_GPIO_Port GPIOD
#define MC_OP2_Pin GPIO_PIN_14
#define MC_OP2_GPIO_Port GPIOD
#define MC_OP1_Pin GPIO_PIN_15
#define MC_OP1_GPIO_Port GPIOD
#define IP3_Pin GPIO_PIN_9
#define IP3_GPIO_Port GPIOC
#define IP2_Pin GPIO_PIN_8
#define IP2_GPIO_Port GPIOA
#define IP2_EXTI_IRQn EXTI9_5_IRQn
#define IP1_Pin GPIO_PIN_9
#define IP1_GPIO_Port GPIOA
#define IP1_EXTI_IRQn EXTI9_5_IRQn
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define SD_MOSI_Pin GPIO_PIN_7
#define SD_MOSI_GPIO_Port GPIOD
#define SD_SCK_Pin GPIO_PIN_3
#define SD_SCK_GPIO_Port GPIOB
#define SD_MISO_Pin GPIO_PIN_4
#define SD_MISO_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_5
#define SD_CS_GPIO_Port GPIOB
#define PC_UART_RE_Pin GPIO_PIN_9
#define PC_UART_RE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

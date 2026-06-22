#ifndef __WIZCHIP_PORT_H__
#define __WIZCHIP_PORT_H__

#include "main.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "stdbool.h"
#include "stdint.h"

/* ----------- Config Macros ---------------- */
#define USE_DHCP    0

/* SPI & UART Handles ---------------------------------- */
//extern SPI_HandleTypeDef W5500_SPI;
//extern UART_HandleTypeDef LOG_UART;

/* ----------- Network Info Structure ------------------ */
extern wiz_NetInfo netInfo;

/* ----------- GPIO Control Macros ---------------------- */
//#define W5500_CS_LOW()      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6, GPIO_PIN_RESET)
//#define W5500_CS_HIGH()     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6, GPIO_PIN_SET)
//#define W5500_RST_LOW()     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, GPIO_PIN_RESET)
//#define W5500_RST_HIGH()    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, GPIO_PIN_SET)

/* ----------- SPI Interface ---------------------------- */
void W5500_Select(void);
void W5500_Unselect(void);

uint8_t W5500_ReadByte(void);
void W5500_WriteByte(uint8_t byte);

/* ----------- DHCP Callback Prototypes ---------------- */
#if USE_DHCP
extern volatile bool ip_assigned;

void Callback_IPAssigned(void);
void Callback_IPConflict(void);
#endif

/* ----------- Initialization API ---------------------- */
int W5500_init(void);

/* Retarget printf */
int _write(int fd, unsigned char *buf, int len);
int W5500_init(void);
#endif  // __W5500_DRIVER_H__

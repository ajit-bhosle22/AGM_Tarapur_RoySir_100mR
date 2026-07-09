
#ifndef TCP_SERVER_REGISTERS_H
#define TCP_SERVER_REGISTERS_H

#include <stdbool.h>
#include "stm32h7xx_hal.h"

#define SOCKET_NUM 0
#define MODBUS_TCP_PORT 5000
#define MAX_SOCK_NUM 8

/*--------------------------------- Config ---------------------------------*/
#define MODBUS_TCP_SOCKET      0
#define MODBUS_TCP_PORT        5000
#define MODBUS_MAX_PDU_SIZE    253   // Modbus spec: max PDU size = 253 bytes
#define MODBUS_MAX_TCP_SIZE    (7 + MODBUS_MAX_PDU_SIZE) // MBAP(7) + PDU
#define HOLDING_REGS_SIZE      128   // number of 16-bit holding registers available

/*---------------------------- Modbus exceptions ----------------------------*/
#define MB_EXCEPTION_ILLEGAL_FUNCTION     0x01
#define MB_EXCEPTION_ILLEGAL_DATA_ADDRESS 0x02
#define MB_EXCEPTION_ILLEGAL_DATA_VALUE   0x03
#define MB_EXCEPTION_SLAVE_DEVICE_FAILURE 0x04

/*---------------------------MODBUS TCP tMIEOUT------------------------------*/
#define SOCKET_CREATE_FAIL_THRESHOLD        3
#define MODBUS_IDLE_TIMEOUT_MS              3000


typedef enum {
    PHY_DOWN = 0,
    PHY_UP
} phy_state_t;


extern volatile phy_state_t  phy_state;
extern volatile bool ntw_alive;
extern volatile uint16_t tcp_slave_id;
extern volatile uint16_t tcp_current_port;
extern volatile uint8_t boot_up_link;
extern volatile uint8_t pending_baud_change;
extern uint32_t pending_new_baud;
extern uint16_t sock_is_sending;
extern volatile bool W5500_Process_Interrupts_flag;

void Update_Registers_TCP(void);
void W5500_Process_Interrupts(void);
void W5500_Apply_New_Settings(uint8_t *ip, uint8_t *sn,uint8_t *gw,uint16_t new_port);
void handle_fc03(uint8_t sn, const uint8_t *pdu, uint16_t pdu_len);
void handle_fc10(uint8_t sn, const uint8_t *pdu, uint16_t pdu_len);
void W5500_Link_Task(void);
void W5500_Health_Monitor(void);
void modbus_tcp_server_stop(void);
void W5500_Network_Task(void);
void W5500_Process_Interrupts(void);
void W5500_Link_Monitor(void);
void modbus_tcp_server_init(void);
void Reconfigure_TCP(void);
bool W5500_Network_Alive(void);
void tcp_task(void);
void monitor_tcp_handle_write(void);
void handle_socket_state_sd(void);
HAL_StatusTypeDef SPI3_ReInit(void);

#endif

/*
 * pc_modbus.h
 *
 *  Created on: Feb 2, 2026
 *  Author: Atharva Margale
 */

#ifndef INC_PC_MODBUS_H_
#define INC_PC_MODBUS_H_

#include <stdbool.h>

extern volatile uint16_t Modbus_Slave_Id_PC;
extern volatile uint8_t PC_UART_Error_Flag;

extern uint8_t rx_buffer_dma_PC[255];
extern uint8_t tx_buffer_dma_PC[255];
extern volatile uint8_t Is_Tx_Done_PC;
extern volatile uint8_t  frame_recv_PC;
extern volatile uint16_t rx_index_PC;

extern volatile uint8_t pending_baud_change;
extern uint32_t pending_new_baud;

bool Modbus_Process_Request_PC(uint8_t *rx_buffer, uint16_t Size);
void UART8_Start_RX(UART_HandleTypeDef *huart);
void Update_Registers_PC(void);
void PC_UART_ReInit(uint32_t new_baud);
void Modbus_Restart_RX_DMA_PC(void);
void modbus_task_pc(void);

#endif /* INC_PC_MODBUS_H_ */

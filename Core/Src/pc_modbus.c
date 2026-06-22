/*
 * pc_modbus.c
 *
 *  Created on: Feb 2, 2026
 *      Author: Ajit Bhosle
 */

#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "main_global.h"
#include "modbus.h"
#include "pc_modbus.h"
#include "lcd.h"
#include "Fault_Handler.h"
#include "AT24CM01_Eeprom.h"
#include "tcp_server_registers.h"

volatile uint8_t pending_baud_change = 0;
uint32_t pending_new_baud = 9600;

volatile uint8_t PC_UART_Error_Flag=0;
volatile uint16_t Modbus_Slave_Id_PC = 1;
uint8_t rx_buffer_dma_PC[BUFFER_SIZE];
uint8_t tx_buffer_dma_PC[BUFFER_SIZE];
volatile uint8_t  frame_recv_PC = 0;
volatile uint16_t rx_index_PC = 0;

static uint8_t response_pc[256];

void modbus_task_pc(void)
{
	if( g_1s_flags.pc_poll == true)
	{
		g_1s_flags.pc_poll = false;
		Pc_RTU_Failed_Timeout++;
	}

	if(frame_recv_PC)
	{
		frame_recv_PC = 0;
		memcpy(tx_buffer_dma_PC, rx_buffer_dma_PC,rx_index_PC);
		memset(rx_buffer_dma_PC, 0, sizeof(rx_buffer_dma_PC));
		bool status = true;
		status = Modbus_Process_Request_PC(tx_buffer_dma_PC,rx_index_PC);
		if(status == false)
		{
			Modbus_Restart_RX_DMA_PC();
		}
		rx_index_PC = 0;
	}

	if (pc_state.single_write)
	{
		pc_state.single_write = 0;
		uint16_t addr = pc_state.last_write_addr;
		uint16_t reg_cnt = pc_state.last_reg_cnt;

		if (addr == REG_ACK) {
			Ack_Button = Modbus_Registers_Write.ACK;
			Modbus_Registers_Write.ACK = 0;
		}
		else if (addr == REG_RESET) {
			Reset_Button = Modbus_Registers_Write.Reset;
			Modbus_Registers_Write.Reset = 0;
		}
		else{

			is_reconfigure_rs485_tcp();
			is_reconfigure_hv_rs485();
			is_reconfigure_rtc_rs485();
			is_sd_card_read_rs485();

			Modbus_Registers = Modbus_Registers_Write;
			Modbus_Registers_PC_TCP = Modbus_Registers;
			conf_rs485_tcp();
			config_variables();
			cpy_reg_to_eeprom_reg();
			Save_Config_To_Eeprom();
			refresh_lcd();
		}

		pc_state.last_write_addr = 0;
		pc_state.last_reg_cnt = 0;
	}

	if(pc_state.multiple_write)
	{
		pc_state.multiple_write = 0;
		uint16_t addr = pc_state.last_write_addr;
		uint16_t reg_cnt = pc_state.last_reg_cnt;

		is_reconfigure_rs485_tcp();
		is_reconfigure_hv_rs485();
		is_reconfigure_rtc_rs485();
		is_sd_card_read_rs485();

		Modbus_Registers = Modbus_Registers_Write;
		Modbus_Registers_PC_TCP = Modbus_Registers;
		conf_rs485_tcp();
		config_variables();
		e4_20mA_calib();
		cpy_reg_to_eeprom_reg();
		Save_Config_To_Eeprom();
        refresh_lcd();

		pc_state.last_write_addr = 0;
		pc_state.last_reg_cnt = 0;
	}

	if(PC_UART_Error_Flag)
	{
		PC_UART_Error_Flag = 0;
		int code = Modbus_Registers.RS485_Baud_Rate;
		pending_new_baud = GetBaudRate(code);
		PC_UART_ReInit(pending_new_baud);
	}
}

void Modbus_Restart_RX_DMA_PC(void)
{
    HAL_UART_DMAStop(&huart8);
    __HAL_UART_CLEAR_OREFLAG(&huart8);
    __HAL_UART_FLUSH_DRREGISTER(&huart8);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, rx_buffer_dma_PC,BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
}

bool Modbus_Process_Request_PC(uint8_t *rx_buffer, uint16_t Size)
{
	uint8_t slave_addr = rx_buffer[0];
	uint8_t func_code  = rx_buffer[1];
	uint16_t start_addr = (rx_buffer[2] << 8) | rx_buffer[3];
	uint16_t reg_count  = (rx_buffer[4] << 8) | rx_buffer[5];
	uint16_t crc_received = (rx_buffer[Size - 1] << 8) | rx_buffer[Size - 2];
	uint16_t crc_calc = Modbus_CRC16(rx_buffer, Size - 2);

	uint16_t* data_ptr = (uint16_t*)&Modbus_Registers;
    bool status = true;
	if (crc_received != crc_calc) {
		status = false;
	}

	if (slave_addr != Modbus_Slave_Id_PC) {
		status = false;
	}

	if (func_code == 0x03)
	{
		if (start_addr + reg_count > MAX_HOLDING_REGS) {
			status = false;
		}

		response_pc[0] = Modbus_Slave_Id_PC;
		response_pc[1] = 0x03;
		response_pc[2] = reg_count * 2;

		for (uint16_t i = 0; i < reg_count; i++)
		{
			uint16_t reg_addr = start_addr + i;
			uint16_t value = data_ptr[reg_addr];
			response_pc[3 + i * 2] = (value >> 8) & 0xFF;
			response_pc[4 + i * 2] = value & 0xFF;
		}

		uint16_t crc = Modbus_CRC16(response_pc, 3 + reg_count * 2);
		response_pc[3 + reg_count * 2] = crc & 0xFF;
		response_pc[4 + reg_count * 2] = (crc >> 8) & 0xFF;

		RS485_TX_MODE_PC();
		HAL_UART_Transmit_DMA(&huart8, response_pc, 5 + reg_count * 2);

		Pc_RTU_Failed_Timeout = 0;
		Pc_RTU_Failed_Status = false;
	}
	else if (func_code == 0x06)
	{
		if (start_addr >= MAX_HOLDING_REGS) {
			status = false;
		}

		Modbus_Registers_Write = Modbus_Registers;
		uint16_t* data_ptr_write = (uint16_t*)&Modbus_Registers_Write;

		uint16_t reg_value = (rx_buffer[4] << 8) | rx_buffer[5];
		data_ptr_write[start_addr] = reg_value;

		RS485_TX_MODE_PC();
		HAL_UART_Transmit_DMA(&huart8, rx_buffer, 8);

		Pc_RTU_Failed_Timeout = 0;
		Pc_RTU_Failed_Status = false;

		pc_state.single_write    = 1;
		pc_state.last_write_addr = start_addr;
		pc_state.last_reg_cnt = 1;
	}
	else if (func_code == 0x10)
	{
		if (Size < 9) {
			status = false;
		}

		uint8_t byte_count = rx_buffer[6];
		if (start_addr + reg_count > MAX_HOLDING_REGS || byte_count != reg_count * 2) {
			status = false;
		}

		Modbus_Registers_Write = Modbus_Registers;
		uint16_t* data_ptr_write = (uint16_t*)&Modbus_Registers_Write;

		for (uint16_t i = 0; i < reg_count; i++)
		{
			uint16_t value = (rx_buffer[7 + i * 2] << 8) | rx_buffer[8 + i * 2];
			data_ptr_write[start_addr + i] = value;
		}

		response_pc[0] = Modbus_Slave_Id_PC;
		response_pc[1] = 0x10;
		response_pc[2] = rx_buffer[2];
		response_pc[3] = rx_buffer[3];
		response_pc[4] = rx_buffer[4];
		response_pc[5] = rx_buffer[5];

		uint16_t crc = Modbus_CRC16(response_pc, 6);
		response_pc[6] = crc & 0xFF;
		response_pc[7] = (crc >> 8) & 0xFF;

		RS485_TX_MODE_PC();
		HAL_UART_Transmit_DMA(&huart8, response_pc, 8);

		Pc_RTU_Failed_Timeout = 0;
		Pc_RTU_Failed_Status = false;

		pc_state.multiple_write  = 1;
		pc_state.last_write_addr = start_addr;
		pc_state.last_reg_cnt = reg_count;
	}
	else {
		status = false;
	}
    return status;
}

void PC_UART_ReInit(uint32_t new_baud)
{
	printf("PC UART ERROR \r\n");

	// -----Disable UART------
	HAL_UART_DeInit(&huart8);

	// ----Update baud rate---
	huart8.Init.BaudRate = new_baud;

	// -----Re-init UART------
	if (HAL_UART_Init(&huart8) != HAL_OK)
	{
		printf("PC UART ReInit Failed\r\n");
	}

	UART8_Start_RX(&huart8);
}

void UART8_Start_RX(UART_HandleTypeDef *huart) {
    RS485_RX_MODE_PC();
    HAL_Delay(2);

    // --------------Clear all flags--------------------
    __HAL_UART_CLEAR_OREFLAG(&huart8);
    __HAL_UART_CLEAR_FEFLAG(&huart8);
    __HAL_UART_CLEAR_NEFLAG(&huart8);
    __HAL_UART_CLEAR_PEFLAG(&huart8);
    __HAL_UART_CLEAR_IDLEFLAG(&huart8);

    // ---------------Flush RX FIFO---------------------
    while (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_RXNE)) {
        volatile uint8_t d = huart8.Instance->RDR;
        (void)d;
    }

    // -----Force abort any pending operation------------
    HAL_UART_Abort(&huart8);

    // -----Small delay for abort to complete------------
    HAL_Delay(1);

    // ----Reset UART state machine manually if needed---
    huart->RxState = HAL_UART_STATE_READY;
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    // -------Disable and re-enable UART to reset it-----
    __HAL_UART_DISABLE(&huart8);
    HAL_Delay(1);
    __HAL_UART_ENABLE(&huart8);

    HAL_UART_DMAStop(&huart8);
    __HAL_UART_CLEAR_FLAG(&huart8, UART_CLEAR_IDLEF);

    // -------------Restart DMA RX----------------------
    HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(&huart8, rx_buffer_dma_PC,BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);

    if(st != HAL_OK) {
    	printf("PC_Modbus Error: Init Error Receive Interrupt \n");
    }

    printf("PC UART Re-Init Done \r\n");
}

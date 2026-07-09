
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "main_global.h"
#include "modbus.h"
#include "lcd.h"
#include "Detector_Modbus_registers.h"
#include "Fault_Handler.h"

#define MAX_REGISTERS_DTC             13
#define DTC_HV_VAL_ADDR               4
#define DTC_HV_REG_COUNT              2
#define DTC_CALIB_FACT_ADDR           9
#define DTC_CALIB_FACT_REG_COUNT      4
#define HV_AND_CALIB_FACT_ADDR        4
#define HV_AND_CALIB_FACT_REG_COUNT   9
#define HV_AND_FREQ_ADDR              4
#define HV_AND_FREQ_REG_COUNT         5

#define DTC_ALLCONF_ADDR              4
#define DTC_ALLCONF_REG_COUNT         9

volatile dtc_rx_t dtc_rx_frame = {0};
uint32_t request_time          = 0;

volatile uint8_t DTC_UART_Error_Flag = 0;
uint8_t  rx_buffer_dma_dtc[BUFFER_SIZE];
uint8_t  tx_buffer_dma_dtc[BUFFER_SIZE];
volatile uint8_t  frame_recv_dtc     = 0;
volatile uint16_t rx_index_dtc       = 0;

uint16_t hv_switch_val;
uint16_t dtc_freq_switch_val;
volatile bool boot_freq_dtc_write_enable = false;
volatile bool boot_freq_dtc_write_done   = false;

DTC_Modbus_State_t           Modbus_State_DTC    = MODBUS_IDLE_DTC;
Modbus_Write_Flag_t          modbus_write_flag   = MODBUS_POLLING_FLAG;
Modbus_Multiple_Write_Flag_t multiple_write_flag = MULTIPLE_WRITE_DONE;
Modbus_Single_Write_Flag_t   single_write_flag   = SINGLE_WRITE_DONE;

master_modbus_db_dtc_t Modbus_Registers_Detector       = {0};
master_modbus_db_dtc_t Modbus_Registers_Detector_Write = {0};

void modbus_task_dtc(void)
{
	if(prev_relay_status != relay_status)
	{
		if(relay_status == 1)
		{
			hv_switch_val = 0xFF;
		}
		else{
			hv_switch_val = 0x00;
		}

		prev_relay_status = relay_status;
		modbus_write_flag = MODBUS_SINGLE_WRITE_FLAG;
		single_write_flag = HV_SWITCH_FLAG;
	}

	if (g_1s_flags.dtc_poll == true)
	{
		if (Modbus_State_DTC == MODBUS_IDLE_DTC)
		{
			if(modbus_write_flag == MODBUS_SINGLE_WRITE_FLAG)
			{
				Modbus_State_DTC = MODBUS_SEND_WRITE_SINGLE;
			}
			else if(modbus_write_flag == MODBUS_MULTIPLE_WRITE_FLAG)
			{
				Modbus_State_DTC = MODBUS_SEND_WRITE_MULTIPLE;
			}
			else
			{
				Modbus_State_DTC = MODBUS_SEND_READ;
			}
		}

		if(Modbus_State_DTC == MODBUS_SEND_WRITE_SINGLE)
		{
			if(single_write_flag == HV_SWITCH_FLAG){
				detector_write_single_register(DTC_HV_SWITCH_ADDR,hv_switch_val);
			}
			if(single_write_flag == DTC_FREQ_SWITCH_FLAG)
			{
				detector_write_single_register(DTC_FREQ_SWITCH_ADDR,dtc_freq_switch_val);
			}
			request_time = HAL_GetTick();
			Modbus_State_DTC = MODBUS_WAIT_RESPONSE;
		}
		else if (Modbus_State_DTC == MODBUS_SEND_WRITE_MULTIPLE)
		{
			// ----for HV Write and Calib Factors-----
			if(multiple_write_flag == HV_WRITE_FLAG){
				detector_write_multiple_registers(DTC_HV_VAL_ADDR,DTC_HV_REG_COUNT);
			}
			else if(multiple_write_flag == CALIB_WRITE_FLAG)
			{
				detector_write_multiple_registers(DTC_CALIB_FACT_ADDR,DTC_CALIB_FACT_REG_COUNT);
			}
			else if(multiple_write_flag == HV_AND_CALIB_WRITE_FLAG)
			{
				detector_write_multiple_registers(HV_AND_CALIB_FACT_ADDR,HV_AND_CALIB_FACT_REG_COUNT);
			}
			else if(multiple_write_flag == HV_AND_FREQ_WRITE_FLAG)
			{
				detector_write_multiple_registers(HV_AND_FREQ_ADDR,HV_AND_FREQ_REG_COUNT);
			}
			else
			{
				detector_write_multiple_registers(DTC_ALLCONF_ADDR,DTC_ALLCONF_REG_COUNT);
			}
			request_time = HAL_GetTick();
			Modbus_State_DTC = MODBUS_WAIT_RESPONSE;
		}
		else if (Modbus_State_DTC == MODBUS_SEND_READ)
		{
			Modbus_Master_Poll_Dtc(DTC_POLLING_START_ADDR,DTC_POLLING_REG_COUNT);
			request_time = HAL_GetTick();
			Modbus_State_DTC = MODBUS_WAIT_RESPONSE;
		}

		Dtc_Failed_Timeout++;
		g_1s_flags.dtc_poll = false;
	}

	if (Modbus_State_DTC == MODBUS_WAIT_RESPONSE)
	{
		if (frame_recv_dtc)
		{
			frame_recv_dtc = 0;

			if (rx_index_dtc > 0 && rx_index_dtc <= sizeof(tx_buffer_dma_dtc))
			{
				memset(tx_buffer_dma_dtc, 0, sizeof(tx_buffer_dma_dtc));
				memcpy(tx_buffer_dma_dtc, rx_buffer_dma_dtc,rx_index_dtc);
				Modbus_Process_Request_dtc(tx_buffer_dma_dtc,rx_index_dtc);
				rx_index_dtc = 0;
			}

			if(dtc_rx_frame.func_code_dtc == 0x03 && dtc_rx_frame.valid_resp == 1)
			{
				initialize_modbus_registers();
				config_CalibFactors_Variables();
				update_cps_hv_mr();
                //config_dtc_switch();

				Dtc_Failed_Timeout         = 0;
				Dtc_Failed_Status          = false;
				dtc_rx_frame.valid_resp    = 0;
				dtc_rx_frame.func_code_dtc = 0;

				Modbus_State_DTC = MODBUS_IDLE_DTC;
			}

			if(dtc_rx_frame.func_code_dtc == 0x06 && dtc_rx_frame.valid_resp == 1)
			{
				Dtc_Failed_Timeout          = 0;
				Dtc_Failed_Status           = false;
				dtc_rx_frame.func_code_dtc  = 0;
				dtc_rx_frame.valid_resp     = 0;

				modbus_write_flag = MODBUS_POLLING_FLAG;
				single_write_flag = SINGLE_WRITE_DONE;
				Modbus_State_DTC  = MODBUS_IDLE_DTC;

				Modbus_Registers_Detector_Write = (master_modbus_db_dtc_t){0};
			}

			if(dtc_rx_frame.func_code_dtc == 0x10 && dtc_rx_frame.valid_resp == 1)
			{
				Dtc_Failed_Timeout          = 0;
				Dtc_Failed_Status           = false;
				dtc_rx_frame.func_code_dtc  = 0;
				dtc_rx_frame.valid_resp     = 0;

				modbus_write_flag           = MODBUS_POLLING_FLAG;
				multiple_write_flag         = MULTIPLE_WRITE_DONE;
				Modbus_State_DTC            = MODBUS_IDLE_DTC;

				Modbus_Registers_Detector_Write = (master_modbus_db_dtc_t){0};
			}
		}

		if (HAL_GetTick() - request_time > TIMEOUT_MS)
		{
			Modbus_State_DTC  = MODBUS_IDLE_DTC;
			modbus_write_flag = MODBUS_POLLING_FLAG;
		}
	}

	if(DTC_UART_Error_Flag == 1)
	{
		DTC_UART_Error_Flag = 0;
		DTC_UART_ReInit();
	}
}

void Modbus_Restart_RX_DMA_DTC()
{
	HAL_UART_DMAStop(&huart2);
	__HAL_UART_CLEAR_OREFLAG(&huart2);
	__HAL_UART_FLUSH_DRREGISTER(&huart2);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer_dma_dtc, BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void Modbus_Master_Poll_Dtc(uint16_t start_addr,uint16_t reg_count)
{
	static uint8_t frame[8];
    frame[0] = 1;
    frame[1] = 0x03;
    frame[2] = (start_addr >> 8) & 0xFF;
    frame[3] = start_addr & 0xFF;
    frame[4] = (reg_count >> 8) & 0xFF;
    frame[5] = reg_count & 0xFF;

    uint16_t crc = Modbus_CRC16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

	RS485_TX_MODE_DTC();
	HAL_UART_Transmit_DMA(&huart2,frame,8);
}

void detector_write_single_register(uint16_t reg_addr,uint16_t reg_value)
{
	static uint8_t frame_dtc[8];
	int idx = 0;

	frame_dtc[idx++] = 1;
	frame_dtc[idx++] = 0x06;
	frame_dtc[idx++] = reg_addr >> 8;
	frame_dtc[idx++] = reg_addr & 0xFF;
	frame_dtc[idx++] = reg_value >> 8;
	frame_dtc[idx++] = reg_value & 0xFF;

	uint16_t crc = Modbus_CRC16(frame_dtc, idx);
	frame_dtc[idx++] = crc & 0xFF;
	frame_dtc[idx++] = crc >> 8;

	RS485_TX_MODE_DTC();
	HAL_UART_Transmit_DMA(&huart2,frame_dtc,idx);

}

void detector_write_multiple_registers(uint16_t start_addr,uint16_t reg_count)
{
    static uint8_t  frame[128];
    uint8_t idx = 0;
    memset(frame, 0, sizeof(frame));

    uint8_t  slave_id   = 1;
    uint16_t *regs      = (uint16_t*)&Modbus_Registers_Detector_Write;

    frame[idx++] = slave_id;
    frame[idx++] = 0x10;
    frame[idx++] = (start_addr >> 8) & 0xFF;
    frame[idx++] =  start_addr       & 0xFF;
    frame[idx++] = (reg_count  >> 8) & 0xFF;
    frame[idx++] =  reg_count        & 0xFF;
    frame[idx++] =  reg_count * 2;

    for (int i = 0; i < reg_count; i++) {

    	uint16_t value = regs[start_addr + i];
    	frame[idx++] = (value >> 8) & 0xFF;
    	frame[idx++] = value & 0xFF;
    }

    uint16_t crc = Modbus_CRC16(frame, idx);
    frame[idx++] = crc & 0xFF;
    frame[idx++] = crc >> 8;

   	RS485_TX_MODE_DTC();
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)frame, idx);
}

void Modbus_Process_Request_dtc(uint8_t *rx_buffer, uint16_t Size)
{
	if (Size < 5){
		return;
	}

	// ---- Extract CRC ----
	uint16_t crc_recv = rx_buffer[Size- 2] | (rx_buffer[Size - 1] << 8);
	uint16_t crc_calc = Modbus_CRC16(rx_buffer, Size - 2);

	if (crc_recv != crc_calc) {
		return;
	}

	uint8_t slave_addr = rx_buffer[0];

	if(slave_addr != 1)
	{
		return;
	}

	uint8_t func_code  = rx_buffer[1];

	if(func_code == 0x03)
	{
		uint8_t byte_count = rx_buffer[2];
		uint16_t* data_ptr = (uint16_t*)&Modbus_Registers_Detector;
		if ((byte_count + 5) != Size) {
			return ;
		}

		if ((byte_count / 2) > MAX_REGISTERS_DTC) {
			return ;
		}

		for (int i = 0; i < byte_count / 2; i++) {
			data_ptr[i] =
					(rx_buffer[3 + i*2] << 8) | rx_buffer[4 + i*2];
		}
		// valid respond
		dtc_rx_frame.valid_resp = 1;
		dtc_rx_frame.func_code_dtc  = func_code;
	}

	if(func_code == 0x06)
	{
		dtc_rx_frame.valid_resp = 1;
		dtc_rx_frame.func_code_dtc  = func_code;
	}

	if(func_code == 0x10)
	{
		uint16_t start_addr = (rx_buffer[2] << 8) | rx_buffer[3];
		uint16_t reg_count  = (rx_buffer[4] << 8) | rx_buffer[5];

		dtc_rx_frame.valid_resp = 1;
		dtc_rx_frame.func_code_dtc  = func_code;
	}
}

void USART2_Start_RX(UART_HandleTypeDef *huart) {
    RS485_RX_MODE_DTC();
    HAL_Delay(2);

    // Clear all flags
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);

    // Flush RX FIFO
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        volatile uint8_t d = huart2.Instance->RDR;
        (void)d;
    }

    // Use Abort instead of AbortReceive
    HAL_UART_Abort(&huart2);

    HAL_Delay(1);

    // Reset UART state machine
    huart->RxState = HAL_UART_STATE_READY;
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    // Disable and re-enable UART to reset it
    __HAL_UART_DISABLE(&huart2);
    HAL_Delay(1);
    __HAL_UART_ENABLE(&huart2);

    HAL_UART_DMAStop(&huart2);
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_IDLEF);

    // Restart DMA RX
    HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer_dma_dtc,BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

    if(st != HAL_OK) {
    	 printf("DTC_Modbus Error: Init Error Receive Interrupt \n");
    }

    printf("DTC UART Re-Init Done \r\n");
}

void DTC_UART_ReInit(void)
{
	printf("DTC UART ERROR \r\n");

	// Disable UART
	HAL_UART_DeInit(&huart2);

	// Re-init UART
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		printf("LCD UART ReInit Failed\r\n");
	}

	USART2_Start_RX(&huart2);
}


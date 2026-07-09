/*
 * tcp_client.c
 *
 *  Created on: Jun 24, 2026
 *      Author: ajit bhosle
 */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"
#include "main_global.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "wizchip_port.h"
#include "modbus.h"
#include "tcp_server_registers.h"
#include "AT24CM01_Eeprom.h"
#include "Fault_Handler.h"
#include "pc_modbus.h"
#include "lcd.h"
#include "fatfs_sd.h"
#include "tcp_client.h"


#define MODBUS_SERVER_SOCKET    0
#define MODBUS_CLIENT_SOCKET    1

#define PC_IP_ADDRESS           {192, 168, 1, 53}  // PC IP
#define PC_MODBUS_PORT          502

/* ─── Add to your header / top of file ───────────────────────── */
#define MODBUS_TCP_ACK_TIMEOUT_MS   100u   /* max wait for slave reply  */
#define MODBUS_TCP_ACK_LEN          12u    /* MBAP(7) + FC(1) + addr(2) + cnt(2) */

typedef enum {
    CLIENT_IDLE,
    CLIENT_CONNECTING,
    CLIENT_ESTABLISHED,
    CLIENT_FAILED
} ClientState;

volatile ClientState client_state = CLIENT_IDLE;
volatile uint8_t pc_ip[4] = {192,168,1,53};

void Reconfig_TCP_PC(void)
{
		tcp_current_port_pc = Modbus_Registers.Ethernet_Port;
		static uint8_t ip[4];

		// ----------------------IP---------------------------
		ip[0] = (Modbus_Registers.Ethernet_IP_MSB_PC >> 8) & 0xFF;
		ip[1] =  Modbus_Registers.Ethernet_IP_MSB_PC       & 0xFF;
		ip[2] = (Modbus_Registers.Ethernet_IP_LSB_PC >> 8) & 0xFF;
		ip[3] =  Modbus_Registers.Ethernet_IP_LSB_PC       & 0xFF;

        pc_ip[0] = ip[0];
        pc_ip[1] = ip[1];
        pc_ip[2] = ip[2];
        pc_ip[3] = ip[3];
}

/* ─── ACK receive + validate ──────────────────────────────────── */
ModbusAckResult Modbus_FC10_WaitAck(uint16_t expected_txn_id)
{
    uint8_t  buf[MODBUS_TCP_ACK_LEN];
    uint32_t start = HAL_GetTick();

    /* 1. Poll until W5500 has bytes ready or timeout */
    while(getSn_RX_RSR(MODBUS_CLIENT_SOCKET) < MODBUS_TCP_ACK_LEN)
    {
        if((HAL_GetTick() - start) >= MODBUS_TCP_ACK_TIMEOUT_MS)
        {
            // printf("FC10 ACK timeout (txn %u)\r\n", expected_txn_id);
            return MODBUS_ACK_TIMEOUT;
        }
        HAL_Delay(1);
    }

    /* 2. Read exactly 12 bytes */
    int32_t received = recv(MODBUS_CLIENT_SOCKET, buf, MODBUS_TCP_ACK_LEN);
    if(received != MODBUS_TCP_ACK_LEN)
    {
        // printf("FC10 ACK short read: %ld\r\n", received);
        return MODBUS_ACK_BAD_FRAME;
    }

    /* 3. Parse MBAP */
    uint16_t rxn_txn_id   = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t rxn_protocol = ((uint16_t)buf[2] << 8) | buf[3];
    /* buf[4..5] = length field, buf[6] = unit ID */
    uint8_t  rxn_func     = buf[7];

    /* 4. Validate transaction ID */
    if(rxn_txn_id != expected_txn_id)
    {
          // printf("FC10 ACK txn mismatch: got %u, expected %u\r\n",
          //               rxn_txn_id, expected_txn_id);
        return MODBUS_ACK_BAD_TXN;
    }

    /* 5. Check protocol ID */
    if(rxn_protocol != 0x0000)
    {
         // printf("FC10 ACK bad protocol ID: 0x%04X\r\n", rxn_protocol);
        return MODBUS_ACK_BAD_FRAME;
    }

    /* 6. Check for Modbus exception (function code | 0x80) */
    if(rxn_func == (0x10 | 0x80))
    {
        uint8_t exception_code = buf[8];
         // printf("FC10 exception code: 0x%02X\r\n", exception_code);
        return MODBUS_ACK_EXCEPTION;
    }

    /* 7. Normal ACK must echo FC 0x10 */
    if(rxn_func != 0x10)
    {
        // printf("FC10 ACK bad function: 0x%02X\r\n", rxn_func);
        return MODBUS_ACK_BAD_FRAME;
    }

    /* 8. verify echoed start address and reg count (buf[8..11]) */
    uint16_t echo_addr = ((uint16_t)buf[8]  << 8) | buf[9];
    uint16_t echo_cnt  = ((uint16_t)buf[10] << 8) | buf[11];
    if(echo_addr != 0x0002 || echo_cnt != 0x000F)
    {
       // printf("FC10 ACK echo mismatch addr=0x%04X cnt=%u\r\n",
       //               echo_addr, echo_cnt);
        return MODBUS_ACK_BAD_FRAME;
    }

    return MODBUS_ACK_OK;
}

bool modbus_client_connect(void)
{
//    uint8_t pc_ip[] = PC_IP_ADDRESS;

    // Close if already open
    close(MODBUS_CLIENT_SOCKET);
    HAL_Delay(10);

    // Open as TCP client
    if(socket(MODBUS_CLIENT_SOCKET, Sn_MR_TCP, 0, 0) != MODBUS_CLIENT_SOCKET)
    {
        printf("Client socket open failed\r\n");
        return false;
    }

    // Connect to PC
    int8_t ret = connect(MODBUS_CLIENT_SOCKET, pc_ip, tcp_current_port_pc);
    if(ret != SOCK_OK)
    {
        printf("Connect to PC failed: %d\r\n", ret);
        close(MODBUS_CLIENT_SOCKET);
        return false;
    }

    // Wait for ESTABLISHED
    uint32_t start = HAL_GetTick();
    while(getSn_SR(MODBUS_CLIENT_SOCKET) != SOCK_ESTABLISHED)
    {
        if((HAL_GetTick() - start) > 3000)
        {
            printf("Connect timeout\r\n");
            close(MODBUS_CLIENT_SOCKET);
            return false;
        }
        HAL_Delay(10);
    }

    printf("Connected to PC as Master\r\n");
    client_state = CLIENT_ESTABLISHED;
    return true;
}

uint16_t Modbus_FC_10_sd_tcp(void)
{
	static uint8_t frame_tcp[256];
	uint8_t  idx = 0;
	memset(frame_tcp, 0, sizeof(frame_tcp));

	static uint16_t transaction_id = 0;
	transaction_id++;

	uint8_t  unit_id    = tcp_slave_id;
	uint16_t start_addr = 0x0002;
	uint16_t reg_count  = 0x000F;
	uint16_t *regs      = (uint16_t*)&read_sd_logs;

	/* ── PDU first ────*/
	uint8_t pdu[256];
	uint8_t pdu_idx = 0;

	pdu[pdu_idx++] = 0x10;
	pdu[pdu_idx++] = (start_addr >> 8) & 0xFF;
	pdu[pdu_idx++] =  start_addr       & 0xFF;
	pdu[pdu_idx++] = (reg_count  >> 8) & 0xFF;
	pdu[pdu_idx++] =  reg_count        & 0xFF;
	pdu[pdu_idx++] =  reg_count * 2;

	for (int i = 0; i < reg_count; i++) {
		uint16_t value = regs[start_addr + i];
		pdu[pdu_idx++] = (value >> 8) & 0xFF;
		pdu[pdu_idx++] =  value       & 0xFF;
	}

	/* ──  MBAP Header (7 bytes) ────── */
	uint16_t mbap_length = 1 + pdu_idx;                // Unit ID (1) + PDU length

	frame_tcp[idx++] = (transaction_id >> 8) & 0xFF;   // Transaction ID  Hi
	frame_tcp[idx++] =  transaction_id       & 0xFF;   // Transaction ID  Lo
	frame_tcp[idx++] = 0x00;                           // Protocol ID     Hi
	frame_tcp[idx++] = 0x00;                           // Protocol ID     Lo
	frame_tcp[idx++] = (mbap_length   >> 8) & 0xFF;    // Length          Hi
	frame_tcp[idx++] =  mbap_length         & 0xFF;    // Length          Lo
	frame_tcp[idx++] =  unit_id;                       // Unit ID

	/* ── Append PDU ─────*/
	memcpy(&frame_tcp[idx], pdu, pdu_idx);
	idx += pdu_idx;

	int32_t sent = send(MODBUS_CLIENT_SOCKET, frame_tcp, idx);
	if (sent != idx) {
		printf("send error\n");
	}

	return transaction_id;
}

void modbus_client_disconnect(void)
{
    uint8_t sr = getSn_SR(MODBUS_CLIENT_SOCKET);

    if(sr == SOCK_CLOSED)
    {
        client_state = CLIENT_IDLE;
        printf("Master socket already closed\r\n");
        return;
    }

    if(sr == SOCK_ESTABLISHED || sr == SOCK_CLOSE_WAIT)
    {
        // Send FIN to peer
        setSn_CR(MODBUS_CLIENT_SOCKET, Sn_CR_DISCON);
        while(getSn_CR(MODBUS_CLIENT_SOCKET));  // wait for command register to clear

        // Wait for SOCK_CLOSED with  timeout
        uint32_t start = HAL_GetTick();
        while(getSn_SR(MODBUS_CLIENT_SOCKET) != SOCK_CLOSED)
        {
            if((HAL_GetTick() - start) > 1000)  // 1 second max wait
            {
                printf("FIN-ACK timeout — force closing socket\r\n");
                break;
            }
            HAL_Delay(10);
        }
    }

    //close
    close(MODBUS_CLIENT_SOCKET);
    client_state = CLIENT_IDLE;
    printf("Master socket closed\r\n");
}

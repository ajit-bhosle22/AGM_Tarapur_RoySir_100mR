/*
 * tcp_client.h
 *
 *  Created on: Jun 24, 2026
 *      Author: admin
 */

#ifndef ETHERNET_W5500_TCP_CLIENT_H_
#define ETHERNET_W5500_TCP_CLIENT_H_


typedef enum {
    MODBUS_ACK_OK       = 0,
    MODBUS_ACK_TIMEOUT,
    MODBUS_ACK_BAD_TXN,     /* transaction ID mismatch    */
    MODBUS_ACK_EXCEPTION,   /* slave returned FC | 0x80   */
    MODBUS_ACK_BAD_FRAME,   /* wrong length / wrong FC    */
} ModbusAckResult;

extern volatile uint8_t pc_ip[4];

uint16_t Modbus_FC_10_sd_tcp(void);
void modbus_client_disconnect(void);
bool modbus_client_connect(void);
void Reconfig_TCP_PC(void);
ModbusAckResult Modbus_FC10_WaitAck(uint16_t expected_txn_id);

#endif /* ETHERNET_W5500_TCP_CLIENT_H_ */

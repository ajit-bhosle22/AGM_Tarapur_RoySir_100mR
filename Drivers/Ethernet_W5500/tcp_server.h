/*
 * tcp_server.h
 *
 *  Created on: Nov 3, 2025
 *      Author: arunrawat
 */

#ifndef ETHERNET_TCP_SERVER_H_
#define ETHERNET_TCP_SERVER_H_


void tcp_server_blocking_loop(void);
void modbus_tcp_server_poll(void);
void modbus_tcp_server_init(void);

#endif /* ETHERNET_TCP_SERVER_H_ */

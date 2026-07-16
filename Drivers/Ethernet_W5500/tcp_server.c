
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

#define MODBUS_SESSION_TIMEOUT_MS 3000

volatile bool W5500_Process_Interrupts_flag = false;
volatile uint8_t Spi_Retry=0;
volatile uint8_t modbus_mbap[7];
volatile uint8_t w5500_reinit_required=0;
volatile uint8_t phy_link_up = 0;
volatile bool ntw_alive = true;

uint32_t last_reopen_ms = 0;
uint32_t last_activity_ms = 0;

volatile phy_state_t  phy_state  = PHY_DOWN;
volatile uint8_t last_link = 0xFF;
volatile uint8_t link;
volatile uint8_t discon_in_progress = 0;  // global flag

void w5500_safe_close(uint8_t s);
int32_t w5500_send_exact(int8_t sn, const uint8_t *buf, uint16_t len, uint32_t timeout_ms);
void handle_fc06(uint8_t sn, const uint8_t *pdu, uint16_t pdu_len);
void modbus_tcp_server_restart(void);
bool w5500_spi_alive(void);

/*--------------------------- Helper (endian) -------------------------------*/
static inline uint16_t read_u16_be(const uint8_t *p) { return (uint16_t)(p[0] << 8) | p[1]; }
static inline void write_u16_be(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFF); }

/* Send exactly len bytes, using W5500 send() and checking TX FSR and errors.
 Returns len on success, 0 on socket/send error, -1 on timeout*/

void modbus_tcp_server_init(void)
{
	socket(MODBUS_TCP_SOCKET, Sn_MR_TCP,tcp_current_port, 0);

//	listen(MODBUS_TCP_SOCKET);
	setSn_KPALVTR(MODBUS_TCP_SOCKET, 1);

	/* ------------Global interrupts----------- */
	setIMR(IM_IR7 | IM_IR6 | IM_IR5 | IM_IR4);

	setSIMR(1 << MODBUS_TCP_SOCKET);

	for (int i = 0; i < MAX_SOCK_NUM; i++){
		setSn_IMR(i, (Sn_IR_CON | Sn_IR_DISCON | Sn_IR_RECV | Sn_IR_TIMEOUT |  Sn_IR_SENDOK ));
	}

}

int32_t w5500_send_exact(int8_t sn, const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if (len == 0) return 0;

    int32_t sent = send(sn, buf, len);
    if (sent < 0) {
        return 0;
    }
    if (sent != len) {
        printf("WARN: send() partial %ld/%u\r\n", sent, (unsigned)len);
    }
    return sent;
}

void handle_fc03(uint8_t sn, const uint8_t *pdu, uint16_t pdu_len)
{
    uint16_t start = read_u16_be(&pdu[1]);
    uint16_t qty   = read_u16_be(&pdu[3]);

	uint16_t* data_ptr = (uint16_t*)&Modbus_Registers_PC_TCP;

    uint16_t byte_count = qty * 2;

    static uint8_t resp[260];

    /* ---------Copy transaction + protocol ID from saved MBAP---------- */
    resp[0] = modbus_mbap[0];
    resp[1] = modbus_mbap[1];
    resp[2] = modbus_mbap[2];
    resp[3] = modbus_mbap[3];

    /* ---------MBAP length = 1 + 1 + 1 + byte_count = 3 + byte_count--- */
    uint16_t mbap_len = 3 + byte_count;
    resp[4] = (mbap_len >> 8) & 0xFF;
    resp[5] = mbap_len & 0xFF;

    /* ------------------------Unit ID---------------------------------- */
    resp[6] = modbus_mbap[6];
    resp[7] = 0x03;
    resp[8] = byte_count;

    for (int i = 0; i < qty; i++)
    {
        uint16_t val = data_ptr[start + i];
        resp[9 + i*2] = val >> 8;
        resp[10 + i*2] = val & 0xFF;
    }

    uint16_t total_len = 9 + byte_count;

    int32_t r = w5500_send_exact(sn, resp, total_len, 500); // 500 ms timeout for TX_FSR

    if (r <= 0) {
	modbus_tcp_server_restart();
	return;
    }

   Pc_TCP_Failed_Timeout = 0;
   Pc_TCP_Failed_Status = false;
 }

/* FC06 - Write Single Register
 * Request: Function(1) + Address(2) + Value(2)
 * Response: Echo of request PDU
 */

void handle_fc06(uint8_t sn, const uint8_t *pdu, uint16_t pdu_len)
{
    uint16_t start_addr = read_u16_be(&pdu[1]);
    uint16_t reg_val  = read_u16_be(&pdu[3]);

    uint16_t* data_ptr = (uint16_t*)&Modbus_Registers_PC_TCP_Write;

    /*  ----------Write register-------------*/
    data_ptr[start_addr] = reg_val;

    static uint8_t resp[260];

    /* -------------MBAP HEADER -------------*/
    resp[0] = modbus_mbap[0];   // Transaction ID
    resp[1] = modbus_mbap[1];
    resp[2] = modbus_mbap[2];   // Protocol ID
    resp[3] = modbus_mbap[3];

    /* -------Length = UnitID(1) + PDU(5)----*/
    resp[4] = 0x00;
    resp[5] = 0x06;

    resp[6] = modbus_mbap[6];   // Unit ID

    /* ---- -----------PDU -------------------*/
    resp[7] = 0x06;
    resp[8] = (start_addr >> 8) & 0xFF;
    resp[9] = start_addr & 0xFF;
    resp[10] = (reg_val >> 8) & 0xFF;
    resp[11] = reg_val & 0xFF;

    uint16_t total_len = 12;

    int32_t r = w5500_send_exact(sn, resp, total_len, 500);
    if (r <= 0) {
        modbus_tcp_server_restart();
        return;
    }

    tcp_state.single_write    = 1;
    tcp_state.last_write_addr = start_addr;
    tcp_state.last_reg_cnt = 1;

    Pc_TCP_Failed_Timeout = 0;
    Pc_TCP_Failed_Status = false;
}

/* FC16 - Write Multiple Registers
 * Request: Function(1) + StartAddr(2) + Quantity(2) + ByteCount(1) + N*Register(2)
 * Response: Function(1) + StartAddr(2) + Quantity(2)
 */
void handle_fc10(uint8_t sn, const uint8_t *pdu, uint16_t pdu_len)
{
    uint16_t start_addr = read_u16_be(&pdu[1]);
    uint16_t qty   = read_u16_be(&pdu[3]);
    uint8_t  byte_count = pdu[5];

    uint16_t* data_ptr = (uint16_t*)&Modbus_Registers_PC_TCP_Write;

    if (byte_count != qty * 2) {
        return;
    }

    /* ------------Write registers -----------*/
    const uint8_t *val_ptr = &pdu[6];
    for (uint16_t i = 0; i < qty; i++) {
        data_ptr[start_addr + i] = read_u16_be(&val_ptr[i * 2]);
    }

    static uint8_t resp[260];

    /* --------------MBAP HEADER --------------*/
    resp[0] = modbus_mbap[0];           // Transaction ID
    resp[1] = modbus_mbap[1];
    resp[2] = modbus_mbap[2];           // Protocol ID
    resp[3] = modbus_mbap[3];

    /* ---------Length = UnitID(1) + PDU(5)-----*/
    resp[4] = 0x00;
    resp[5] = 0x06;

    resp[6] = modbus_mbap[6];           // Unit ID

    /* ---- -------------PDU -------------------*/
    resp[7]  = 0x10;                    // Function code
    resp[8]  = (start_addr >> 8) & 0xFF;
    resp[9]  = start_addr & 0xFF;
    resp[10] = (qty >> 8) & 0xFF;
    resp[11] = qty & 0xFF;

    uint16_t total_len = 12;

    int32_t r = w5500_send_exact(sn, resp, total_len, 500);
    if (r <= 0) {
        modbus_tcp_server_restart();
        return;
    }

    tcp_state.multiple_write  = 1;
    tcp_state.last_write_addr = start_addr;
    tcp_state.last_reg_cnt = qty;

    Pc_TCP_Failed_Timeout = 0;
    Pc_TCP_Failed_Status = false;
}

/* Main Modbus request dispatcher
 * Input: `req` points to full MBAP+PDU buffer, `len` is bytes read from socket
 */
static void modbus_handle_request(uint8_t sn, const uint8_t *req, uint16_t len)
{
    if (len < 8) {
        /* Minimum MBAP(7) + Function(1) = 8 */
        return;
    }

    /* MBAP parsing */
    uint16_t trans_id = read_u16_be(&req[0]);
    uint16_t proto_id = read_u16_be(&req[2]);
    uint16_t mbap_len = read_u16_be(&req[4]);
    uint8_t unit_id = req[6];

    /* Basic checks */
    if (proto_id != 0) {
        /* Not Modbus TCP */
        return;
    }

    /* Verify Unit Id */
    if(tcp_slave_id!=unit_id)
    {
    	//Slave Id Incorrect
        return;
    }

    /* PDU starts at req[7], pdu length should be mbap_len - 1 (unit id) */
    if (mbap_len == 0 || (uint16_t)(mbap_len - 1) > MODBUS_MAX_PDU_SIZE) {
        /* malformed */
        return;
    }

    uint16_t pdu_len = (uint16_t)(mbap_len - 1);
    if ((uint16_t)(7 + pdu_len) > len) {
        /* packet smaller than claimed length -> ignore */
        return;
    }

    const uint8_t *pdu = &req[7];
    uint8_t func = pdu[0];

    switch (func) {
        case 0x03:
            handle_fc03(sn, pdu, pdu_len);
            break;
        case 0x06:
        	Modbus_Registers_PC_TCP_Write = Modbus_Registers_PC_TCP;
            handle_fc06(sn, pdu, pdu_len);
            break;
        case 0x10:
        	Modbus_Registers_PC_TCP_Write = Modbus_Registers_PC_TCP;
        	handle_fc10(sn, pdu, pdu_len);
            break;
        default:
            break;
    }
}

void w5500_safe_close(uint8_t s)
{
    uint8_t sr = getSn_SR(s);

    /*--------------If already closed nothing to do---*/
    if (sr == SOCK_CLOSED)
        return;

    /*-------------disconnect only if socket is established---*/
    if (sr == SOCK_ESTABLISHED || sr == SOCK_CLOSE_WAIT)
    {
        disconnect(s);
    }

    /*-------------------close socket-----------------*/
    close(s);
}

void modbus_tcp_server_restart(void)
{
    w5500_safe_close(MODBUS_TCP_SOCKET);

    socket(MODBUS_TCP_SOCKET, Sn_MR_TCP, tcp_current_port, 0);
    listen(MODBUS_TCP_SOCKET);

    printf("TCP Server restarted\r\n");
}

void W5500_Apply_New_Settings(uint8_t *ip, uint8_t *sn,uint8_t *gw,uint16_t new_port)
{
    wiz_NetInfo net;
    /* -----------------------------
        CLOSE ALL SOCKETS
       ----------------------------- */
    for (uint8_t s = 0; s < 8; s++)
    {
        close(s);
        HAL_Delay(2);
    }
    /* -----------------------------
       READ CURRENT NETINFO (KEEP MAC)
       ----------------------------- */
    ctlnetwork(CN_GET_NETINFO, &net);
    /* -----------------------------
       UPDATE NETWORK VALUES
       ----------------------------- */
    memcpy(net.ip, ip, 4);
    memcpy(net.sn, sn, 4);
    memcpy(net.gw, gw, 4);
    /* -----------------------------
       APPLY NEW SETTINGS
       ----------------------------- */
    ctlnetwork(CN_SET_NETINFO, &net);
    HAL_Delay(100);
    /* -----------------------------
       REOPEN MODBUS TCP SOCKET
       ----------------------------- */
    socket(MODBUS_TCP_SOCKET, Sn_MR_TCP, new_port, 0);
    listen(MODBUS_TCP_SOCKET);
    printf("Network updated successfully\r\n");
    printf("IP: %d.%d.%d.%d  PORT: %d\r\n",
           net.ip[0], net.ip[1], net.ip[2], net.ip[3],
           new_port);
}

void W5500_Link_Monitor(void)
{
	ctlwizchip(CW_GET_PHYLINK, &link);
	if (link == last_link)
		return;

	last_link = link;
	if (link == PHY_LINK_ON)
	{
		phy_state = PHY_UP;
	}
	else
	{
		phy_state = PHY_DOWN;
	}
}

void W5500_Network_Task(void)
{
	static bool disconnect_sent = false;
    uint8_t sr = getSn_SR(MODBUS_TCP_SOCKET);

    switch(sr)
    {
        case SOCK_CLOSED:
            /* Socket entered CLOSED state.
             *
             * This may happen because:
             * 1. Firmware explicitly closed it.
             * 2. Peer disconnected normally.
             * 3. W5500 TCP Keep-Alive detected an unreachable peer and
             *    automatically terminated the connection.
             *
             * Recreate the server socket.
             */
        	disconnect_sent = false;
            modbus_tcp_server_init();
            break;

        case SOCK_INIT:
            listen(MODBUS_TCP_SOCKET);
            break;

        case SOCK_LISTEN:
            break;

        case SOCK_ESTABLISHED:
            break;

        case SOCK_CLOSE_WAIT:
        	if(!disconnect_sent){
        	disconnect(MODBUS_TCP_SOCKET);
        	disconnect_sent = true;
        	}
        	break;

        default:
            break;
    }
}

void W5500_Process_Interrupts(void)
{
	if(W5500_Process_Interrupts_flag == true){

		W5500_Process_Interrupts_flag = false;
		uint8_t sn_ir = getSn_IR(SOCKET_NUM);

		// ---------------SENDOK flag--------------
		if(sn_ir & Sn_IR_SENDOK)
		{
			setSn_IR(SOCKET_NUM, Sn_IR_SENDOK);
			sock_is_sending &= ~(1 << SOCKET_NUM);
		}

		if (sn_ir & Sn_IR_RECV)
		{
			setSn_IR(SOCKET_NUM, Sn_IR_RECV);
			uint16_t rx_len = getSn_RX_RSR(SOCKET_NUM);
			if (rx_len > 0)
			{
				static uint8_t buf[256];
				int32_t ret = recv(SOCKET_NUM, buf, rx_len);
				if (ret > 0)
				{
					/* Filter out kickstart echo*/
					if (ret == 6 && buf[4] == 0x00 && buf[5] == 0x00) {
						/* ignore — kickstart echo*/
						return;
					}

					memcpy(modbus_mbap, buf, 7);
					uint16_t expected_pdu_len = ret - 7;
					if (expected_pdu_len > 0)
					{
						modbus_handle_request(SOCKET_NUM, buf, ret);
						monitor_tcp_handle_write();
					}
				}
			}
		}

		/* Connection established.
		 *
		 * Enable automatic TCP Keep-Alive.
		 *
		 * NOTE:
		 * The W5500 starts automatic keep-alive only after at least one
		 * TCP application data packet has been transmitted.
		 * Therefore a dummy packet is sent to arm the keep-alive engine.
		 */
		if (sn_ir & Sn_IR_CON)
		{
			setSn_IR(SOCKET_NUM, Sn_IR_CON);

			setSn_KPALVTR(MODBUS_TCP_SOCKET, 1);

			/*Keep Alive armed*/
			uint8_t kickstart[] = {0x00, 0x00,   // Transaction ID
					0x00, 0x00,                  // Protocol ID
					0x00, 0x00};                 // Length = 0 (invalid, ignored)
			send(MODBUS_TCP_SOCKET, kickstart, sizeof(kickstart));
		}

		/* -------------DISCONNECT ---------- */
		if(sn_ir & Sn_IR_DISCON)
		{
			setSn_IR(SOCKET_NUM, Sn_IR_DISCON);
		}

		/* -------------TIMEOUT---------------*/
		if(sn_ir & Sn_IR_TIMEOUT)
		{
			setSn_IR(SOCKET_NUM, Sn_IR_TIMEOUT);
		}
	}
}

void monitor_tcp_handle_write(void)
{
	if (tcp_state.single_write)
	{
		tcp_state.single_write = 0;
		uint16_t addr = tcp_state.last_write_addr;
		uint16_t reg_cnt = tcp_state.last_reg_cnt;

		if (addr == REG_ACK) {
			Ack_Button = Modbus_Registers_PC_TCP_Write.ACK;
			Modbus_Registers_PC_TCP_Write.ACK = 0;
		}
		else if (addr == REG_RESET) {
			Reset_Button = Modbus_Registers_PC_TCP_Write.Reset;
			Modbus_Registers_PC_TCP_Write.Reset = 0;
		}
		else{

			validate_modbus_write_tcp();
			is_reconfigure_tcp_rs485();
			is_reconfigure_hv_tcp(addr,reg_cnt);
			is_reconfigure_rtc_tcp();
			is_sd_card_read_tcp();

			Modbus_Registers_PC_TCP = Modbus_Registers_PC_TCP_Write;
			Modbus_Registers        = Modbus_Registers_PC_TCP;
			conf_rs485_tcp();
			config_variables();
			cpy_reg_to_eeprom_reg();
			Save_Config_To_Eeprom();
			refresh_lcd();
		}

		tcp_state.last_write_addr = 0;
		tcp_state.last_reg_cnt = 0;
	}

	if(tcp_state.multiple_write)
	{
		tcp_state.multiple_write = 0;
		uint16_t addr = tcp_state.last_write_addr;
		uint16_t reg_cnt =  tcp_state.last_reg_cnt;

		validate_modbus_write_tcp();
		check_ack_and_reset_tcp(addr,reg_cnt);
		is_reconfigure_tcp_rs485();
		is_reconfigure_hv_tcp(addr,reg_cnt);
		is_reconfigure_rtc_tcp();
		is_sd_card_read_tcp();

		Modbus_Registers_PC_TCP = Modbus_Registers_PC_TCP_Write;
		Modbus_Registers        = Modbus_Registers_PC_TCP;
		conf_rs485_tcp();
		config_variables();
		e4_20mA_calib();
		cpy_reg_to_eeprom_reg();
		Save_Config_To_Eeprom();
		refresh_lcd();

		tcp_state.last_write_addr = 0;
		tcp_state.last_reg_cnt = 0;
	}
}

void W5500_Health_Monitor(void)
{
    static uint32_t last_check = 0;

    if (HAL_GetTick() - last_check < 3000)
        return;

    last_check = HAL_GetTick();

    if (!w5500_spi_alive())
    {
    	Spi_Retry++;
        if(Spi_Retry >= 3)
        {
           printf("W5500 spi communication dead \r\n");
           return;
        }

        HAL_StatusTypeDef status;
        status = SPI3_ReInit();
        if(status==HAL_OK){
          printf("W5500 spi re_init done \r\n");
          W5500_init();
          modbus_tcp_server_init();
        }
        else{
          printf("W5500 spi re_init failed \r\n");
        }
    }
}

bool w5500_spi_alive(void)
{
    uint8_t ver = getVERSIONR();
    if(ver == 0x04)
    {
    	return 1;
    }
    else{
    	printf("W5500 spi communication failed \r\n");
    	return 0;
    }
}

void tcp_task(void)
{
	if(g_1s_flags.eth_status == true)
	{
		g_1s_flags.eth_status = false;
		Pc_TCP_Failed_Timeout++;
	}

//	W5500_Link_Monitor();
	W5500_Network_Task();
	W5500_Process_Interrupts();
	W5500_Health_Monitor();

	ntw_alive = W5500_Network_Alive();
	if(!ntw_alive)
	{
		W5500_init();
	}

}

void Reconfigure_TCP(void)
{
	/* ----------------Apply Changes---------------------- */
	tcp_slave_id     = Modbus_Registers.Ethernet_Slave_Id;
	tcp_current_port = Modbus_Registers.Ethernet_Port;

	static uint8_t ip[4];
	static uint8_t sn[4];
	static uint8_t gw[4];

	/* ----------------------IP--------------------------- */
	ip[0] = (Modbus_Registers.Ethernet_IP_MSB >> 8) & 0xFF;
	ip[1] =  Modbus_Registers.Ethernet_IP_MSB       & 0xFF;
	ip[2] = (Modbus_Registers.Ethernet_IP_LSB >> 8) & 0xFF;
	ip[3] =  Modbus_Registers.Ethernet_IP_LSB       & 0xFF;

	/* --------------------SUBNET-------------------------- */
	sn[0] = (Modbus_Registers.Ethernet_Subnet_MSB >> 8) & 0xFF;
	sn[1] =  Modbus_Registers.Ethernet_Subnet_MSB       & 0xFF;
	sn[2] = (Modbus_Registers.Ethernet_Subnet_LSB >> 8) & 0xFF;
	sn[3] =  Modbus_Registers.Ethernet_Subnet_LSB       & 0xFF;

	/* --------------------GATEWAY--------------------------- */
	gw[0] = (Modbus_Registers.Ethernet_Gateway_MSB >> 8) & 0xFF;
	gw[1] =  Modbus_Registers.Ethernet_Gateway_MSB       & 0xFF;
	gw[2] = (Modbus_Registers.Ethernet_Gateway_LSB >> 8) & 0xFF;
	gw[3] =  Modbus_Registers.Ethernet_Gateway_LSB       & 0xFF;


	W5500_Apply_New_Settings(ip,sn,gw,tcp_current_port);
}

bool W5500_Network_Alive(void)
{
    wiz_NetInfo net;
    ctlnetwork(CN_GET_NETINFO, &net);

    /* IP wiped to zeros : power glitch hit common registers */
    if (net.ip[0] == 0 && net.ip[1] == 0 &&
        net.ip[2] == 0 && net.ip[3] == 0)
    {
        printf("W5500 IP lost!\r\n");
        return false;
    }

//    /* Check PHY link is actually up in hardware */
//    uint8_t phyLink;
//    ctlwizchip(CW_GET_PHYLINK, &phyLink);
//    if (phyLink != PHY_LINK_ON)
//    {
//        /* PHY down : not a network stack issue */
//        /* W5500_Link_Monitor handle this */
//        return true;
//    }
//
//    /* SPI alive and PHY up but socket stuck at CLOSED for too long : frozen*/
//    uint8_t sr = getSn_SR(MODBUS_TCP_SOCKET);
//    static uint32_t sock_closed_since = 0;
//
//    if (sr == SOCK_CLOSED || sr == SOCK_INIT)
//    {
//        if (sock_closed_since == 0)
//            sock_closed_since = HAL_GetTick();
//
//        /* If stuck closed for more than 5 seconds despite PHY being up*/
//        if ((HAL_GetTick() - sock_closed_since) > 5000)
//        {
//            printf("W5500 socket stuck — network frozen\r\n");
//            sock_closed_since = 0;
//            return false;
//        }
//    }
//    else
//    {
//    	 /* reset when socket is healthy */
//        sock_closed_since = 0;
//    }
    return true;
}

HAL_StatusTypeDef SPI3_ReInit(void)
{
    if (HAL_SPI_DeInit(&hspi3) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(10);

    hspi3.Instance = SPI3;
    hspi3.Init.Mode                       = SPI_MODE_MASTER;
    hspi3.Init.Direction                  = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize                   = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity                = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase                   = SPI_PHASE_1EDGE;
    hspi3.Init.NSS                        = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_16;
    hspi3.Init.FirstBit                   = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode                     = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial              = 0x0;
    hspi3.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;
    hspi3.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
    hspi3.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
    hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi3.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi3.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi3.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi3.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi3.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;

    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

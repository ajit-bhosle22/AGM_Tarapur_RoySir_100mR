#include "main.h"
#include "wizchip_conf.h"
#include "stdio.h"
#include "string.h"
#include "socket.h"
#include "stdbool.h"
#include "DHCP/dhcp.h"
#include "DNS/dns.h"
#include "wizchip_port.h"
#include "modbus.h"

void W5500_ReadBurst(uint8_t* pBuf, uint16_t len);
void W5500_WriteBurst(uint8_t* pBuf, uint16_t len);

#define W550_SPI hspi1
#define LOG_UART huart7
//#define USE_DHCP 1
#define DNS_SOCKET  6
uint8_t DNS_buffer[512];
volatile uint8_t boot_up_link=1;

wiz_NetInfo netInfo = {
	.mac ={0x02, 0x12, 0x34, 0x56, 0x78, 0x01},
	.ip = {192,168,1,10},
	.sn = {255,255,255,0},
	.gw = {192,168,1,1},
	.dns = {8,8,8,8},
#if USE_DHCP
    .dhcp = NETINFO_DHCP
#else
	.dhcp = NETINFO_STATIC
#endif
};

#define W5500_CS_LOW()       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
#define W5500_CS_HIGH()      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);

#define W5500_RST_LOW()      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);
#define W5500_RST_HIGH()     HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);

// SPI Transmit/Receive
void W5500_Select(void)     {W5500_CS_LOW();}
void W5500_Unselect(void)   {W5500_CS_HIGH();}

extern SPI_HandleTypeDef  W5500_SPI;

extern UART_HandleTypeDef LOG_UART;

int _write(int fd,unsigned char *buf,int len)
{
	if(fd==1 || fd==2)
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
		HAL_UART_Transmit(&LOG_UART,buf,len,999);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
	}
	return len;
}

uint8_t W5500_ReadByte(void)
{
	uint8_t rx;
	uint8_t tx=0xFF;
	HAL_SPI_TransmitReceive(&W5500_SPI,&tx,&rx,1,HAL_MAX_DELAY);
	return rx;
}

void W5500_WriteByte(uint8_t byte)
{
	HAL_SPI_Transmit(&W5500_SPI,&byte,1,HAL_MAX_DELAY);
}

void W5500_ReadBurst(uint8_t* pBuf, uint16_t len)
{
    HAL_SPI_Receive(&W5500_SPI, pBuf, len, HAL_MAX_DELAY);
}

void W5500_WriteBurst(uint8_t* pBuf, uint16_t len)
{
    HAL_SPI_Transmit(&W5500_SPI, pBuf, len, HAL_MAX_DELAY);
}

#if USE_DHCP
volatile bool ip_assigned = false;
#define DHCP_SOCKET 7            // Last Available Socket
#define DNS_SOCKET  6            // 2nd Last Socket
uint8_t DHCP_buffer[548];
uint8_t DNS_buffer[512];

void Callback_IPAssigned(void)
{
	ip_assigned = true;
}

void Callback_IPConflict(void)
{
	ip_assigned = false;
}
#endif


int W5500_init(void)
{
    uint8_t memsize[2][8] = {
        {2,2,2,2,2,2,2,2},    // TX
        {2,2,2,2,2,2,2,2}     // RX
    };

    /* ------------------------------
       HARDWARE RESET SEQUENCE
       ------------------------------ */
    W5500_RST_LOW();
    HAL_Delay(10);
    W5500_RST_HIGH();
    HAL_Delay(200);

    /* Ensure CS is HIGH BEFORE init */
    W5500_Unselect();
    HAL_Delay(2);

    /* ------------------------------
       REGISTER CALLBACKS
       ------------------------------ */
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);

    // Important: register burst BEFORE ctlwizchip()
    reg_wizchip_spiburst_cbfunc(W5500_ReadBurst, W5500_WriteBurst);

    /* ------------------------------
       INITIALIZE W5500 CHIP
       ------------------------------ */
    if (ctlwizchip(CW_INIT_WIZCHIP, (void*)memsize) == -1)
    {
        printf("Error initializing W5500\r\n");
        return -1;
    }

    printf("W5500 Init OK\r\n");

    /* ------------------------------
       WAIT FOR PHY LINK
       ------------------------------ */
    uint8_t link;
    uint8_t retries = 2;

    do {
        ctlwizchip(CW_GET_PHYLINK, &link);

        if (link == PHY_LINK_ON) {
            printf("Link: UP\r\n");
            break;
        } else {
            printf("Link: DOWN (retry %d)\r\n", 20 - retries);
        }

        retries--;
        HAL_Delay(300);

    } while (retries > 0);

    if (link != PHY_LINK_ON)
    {
        printf("ERROR: LINK DOWN. Check cable.\r\n");
        boot_up_link=0;
        //return -2;
    }

    /* ------------------------------
       DHCP OR STATIC IP
       ------------------------------ */
#if USE_DHCP

    printf("Using DHCP…\r\n");

    setSHAR(netInfo.mac);             // Always set MAC first
    DHCP_init(DHCP_SOCKET, DHCP_buffer);

    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned,
        Callback_IPConflict
    );

    retries = 20;
    ip_assigned = 0;

    while (!ip_assigned && retries--)
    {
        DHCP_run();
        HAL_Delay(500);
    }

    if (!ip_assigned)
    {
        printf("DHCP Failed → Using static IP\r\n");
        ctlnetwork(CN_SET_NETINFO, (void*)&netInfo);
    }
    else
    {
        printf("DHCP Success\r\n");

        getIPfromDHCP(netInfo.ip);
        getIPfromDHCP(netInfo.gw);
        getIPfromDHCP(netInfo.sn);
        getIPfromDHCP(netInfo.dns);

        ctlnetwork(CN_SET_NETINFO, (void*)&netInfo);
    }

#else
    /* STATIC IP PATH */
    printf("Using Static IP...\r\n");

    setSHAR(netInfo.mac);  // *** FIX: MUST SET MAC ***
    HAL_Delay(5);

    memcpy(netInfo.ip, ip, 4);
    memcpy(netInfo.sn, sn, 4);
    memcpy(netInfo.gw, gw, 4);

    ctlnetwork(CN_SET_NETINFO, (void*)&netInfo);
#endif

    /* ------------------------------
       DNS INITIALIZATION
       ------------------------------ */
    HAL_Delay(200);
    printf("Configuring DNS...\r\n");
    DNS_init(DNS_SOCKET, DNS_buffer);

    /* ------------------------------
       PRINT NETWORK INFO
       ------------------------------ */
    wiz_NetInfo tmp;
    ctlnetwork(CN_GET_NETINFO, &tmp);

    printf("IP: %d.%d.%d.%d\r\n",
            tmp.ip[0], tmp.ip[1], tmp.ip[2], tmp.ip[3]);

    printf("MASK: %d.%d.%d.%d\r\n",
            tmp.sn[0], tmp.sn[1], tmp.sn[2], tmp.sn[3]);

    printf("GW: %d.%d.%d.%d\r\n",
            tmp.gw[0], tmp.gw[1], tmp.gw[2], tmp.gw[3]);

    printf("DNS: %d.%d.%d.%d\r\n",
            tmp.dns[0], tmp.dns[1], tmp.dns[2], tmp.dns[3]);

    return 0;
}


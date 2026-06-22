
#ifndef __FATFS_SD_H
#define __FATFS_SD_H

#include "diskio.h"
#include "ff.h"

/* Definitions for MMC/SDC command */
#define CMD0     (0x40+0)     	/* GO_IDLE_STATE */
#define CMD1     (0x40+1)     	/* SEND_OP_COND */
#define CMD8     (0x40+8)     	/* SEND_IF_COND */
#define CMD9     (0x40+9)     	/* SEND_CSD */
#define CMD10    (0x40+10)    	/* SEND_CID */
#define CMD12    (0x40+12)    	/* STOP_TRANSMISSION */
#define CMD16    (0x40+16)    	/* SET_BLOCKLEN */
#define CMD17    (0x40+17)    	/* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)    	/* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)    	/* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)    	/* WRITE_BLOCK */
#define CMD25    (0x40+25)    	/* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)    	/* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    	/* APP_CMD */
#define CMD58    (0x40+58)    	/* READ_OCR */

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		0x06		/* SD */
#define CT_BLOCK	0x08		/* Block addressing */

/* Functions */
DSTATUS SD_disk_initialize (BYTE pdrv);
DSTATUS SD_disk_status (BYTE pdrv);
DRESULT SD_disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);

#define SPI_TIMEOUT 100

typedef struct
{
	uint16_t RTC_DAY;    //Addr0
	uint16_t RTC_MONTH;  //Addr1
	uint16_t RTC_YEAR;   //Addr2
	uint16_t RTC_HOUR;   //Addr3
	uint16_t RTC_MIN;    //Addr4
	uint16_t RTC_SEC;    //Addr5
	uint16_t Overload;   //Addr6
	uint16_t Overrun;    //Addr7
	uint16_t Alarm;      //Addr8
	uint16_t Hv_Fault;   //Addr9
	uint16_t Dtc_Fault;  //Addr10
	uint16_t CPS_MSB;    //Addr11
	uint16_t CPS_LSB;    //Addr12
	uint16_t mR_MSB;     //Addr13
	uint16_t mR_LSB;     //Addr14
} __attribute__((packed)) LogRecord_t;

extern LogRecord_t sd_logs;

extern uint32_t g_sequence ;
extern uint32_t g_journal_slot ;

FRESULT Logger_ReadRecord(uint16_t index,
                          LogRecord_t *record);
FRESULT Logger_WriteRecord(LogRecord_t *record);
FRESULT Logger_LoadIndex(void);
FRESULT Logger_SaveIndex(void);
FRESULT Logger_ExportCSV(void);

void file_operations(void);
void create_file(void);
void sd_card_operations(void);
void modbus_task_sd(void);
void Modbus_FC_10_sd(void);

#endif

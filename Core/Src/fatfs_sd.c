
#define TRUE  1
#define FALSE 0
#define bool BYTE

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "main_global.h"
#include "diskio.h"
#include "fatfs_sd.h"
#include "ff.h"
#include "lcd.h"
#include "pc_modbus.h"
#include "modbus.h"
#include "AT24CM01_Eeprom.h"
#include "tcp_server_registers.h"
#include "tcp_client.h"

extern  SPI_HandleTypeDef 	hspi1;
#define HSPI_SDCARD		 	&hspi1
#define	SD_CS_PORT			GPIOB
#define SD_CS_PIN			GPIO_PIN_5
#define INDEX_FILE          "sd_index.bin"
#define MAX_RECORDS         500

LogRecord_t sd_logs = {0};
LogRecord_t read_sd_logs = {0};
static uint16_t sd_index = 0;

volatile uint16_t Timer1, Timer2;			/* 1ms Timer Counter */

static volatile DSTATUS Stat = STA_NOINIT;	/* Disk Status */
static uint8_t CardType;                    /* Type 0:MMC, 1:SDC, 2:Block addressing */
static uint8_t PowerFlag = 0;				/* Power flag */

uint32_t g_sequence = 0;
uint32_t g_journal_slot = 0;
uint32_t g_write_count = 0;

FATFS fs;
static FIL g_fil;
static FIL g_history_fil;
static FIL g_index_fil;
static FIL g_read_fil;

static uint32_t get_write_count(LogRecord_t *r);
static FRESULT SaveRecoveredIndex(void);

/***************************************
 * SPI functions
 **************************************/

typedef struct {
    uint32_t magic;      // 0xDEADBEEF
    uint16_t sd_index;
    uint16_t checksum;   // simple XOR or CRC16
} IndexFile_t;

#define INDEX_MAGIC  0xDEADBEEF

uint16_t Index_Checksum(uint16_t index)
{
    return (uint16_t)(INDEX_MAGIC ^ index);
}

/* slave select */
static void SELECT(void)
{
	HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
	//HAL_Delay(1);
}

/* slave deselect */
static void DESELECT(void)
{
	HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
	//HAL_Delay(1);
}

/* SPI transmit a byte */
static void SPI_TxByte(uint8_t data)
{
	while(!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE));
	HAL_SPI_Transmit(HSPI_SDCARD, &data, 1, SPI_TIMEOUT);
}

/* SPI transmit buffer */
static void SPI_TxBuffer(uint8_t *buffer, uint16_t len)
{
	while(!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE));
	HAL_SPI_Transmit(HSPI_SDCARD, buffer, len, SPI_TIMEOUT);
}

/* SPI receive a byte */
static uint8_t SPI_RxByte(void)
{
	uint8_t dummy, data;
	dummy = 0xFF;

	while(!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE));
	HAL_SPI_TransmitReceive(HSPI_SDCARD, &dummy, &data, 1, SPI_TIMEOUT);

	return data;
}

/* SPI receive a byte via pointer */
static void SPI_RxBytePtr(uint8_t *buff)
{
	*buff = SPI_RxByte();
}

/***************************************
 * SD functions
 **************************************/

void sd_card_operations(void)
{
	if(g_1s_flags.log_sd_card == true)
	{
		g_1s_flags.log_sd_card = false;
		Fill_Record();
		Logger_WriteRecord(&sd_logs);
	}
}

bool create_file(void)
{
    FRESULT fres;
    FILINFO fno;

    fres = f_mount(&fs, "", 1);
    printf("f_mount = %d\r\n", fres);

    if(fres != FR_OK)
    {
        printf("Mount Failed\r\n");
        return false;
    }

    UINT bw;

    uint32_t expected_size = MAX_RECORDS * sizeof(LogRecord_t);

    fres = f_stat("history.bin", &fno);
    printf("f_stat = %d\r\n", fres);
    printf("File Size = %lu\r\n", (uint32_t)fno.fsize);
    printf("Record Size = %u\r\n", sizeof(LogRecord_t));

    if(fres != FR_OK)
    {
        printf("Creating history.bin\r\n");

        memset(&g_history_fil, 0, sizeof(g_history_fil));
        fres = f_open(&g_history_fil, "history.bin", FA_CREATE_ALWAYS | FA_WRITE);
        printf("f_open = %d\r\n", fres);

        if(fres == FR_OK)
        {
            fres = f_lseek(&g_history_fil, expected_size - 1);
            printf("f_lseek = %d\r\n", fres);

            uint8_t dummy = 0;
            fres = f_write(&g_history_fil, &dummy, 1, &bw);
            printf("f_write = %d\r\n", fres);
            printf("Bytes Written = %u\r\n", bw);

            f_close(&g_history_fil);
        }
    }
    else
    {
        printf("history.bin already exists\r\n");
        printf("File Size = %lu\r\n", (uint32_t)fno.fsize);
    }
    return true;
}

// Helper to combine MSB/LSB into uint32_t
static uint32_t get_write_count(LogRecord_t *r)
{
    return ((uint32_t)r->WRITE_COUNT_MSB << 16) | r->WRITE_COUNT_LSB;
}

static FRESULT SaveRecoveredIndex(void)
{
    FRESULT fres;

    fres = f_open(&g_index_fil,
                  "index.bin",
                  FA_OPEN_ALWAYS | FA_WRITE);

    if(fres != FR_OK)
    {
        return fres;
    }

    fres = Logger_SaveIndex();

    f_close(&g_index_fil);

    return fres;
}


FRESULT Logger_RecoverIndex(void)
{
    FIL tmp_history_fil;
    UINT br;
    FRESULT fres;
    LogRecord_t record;

    static uint32_t counts[MAX_RECORDS];

    fres = f_open(&tmp_history_fil,
                  "history.bin",
                  FA_OPEN_EXISTING | FA_READ);

    if(fres != FR_OK)
    {
        sd_index = 0;
        g_write_count = 0;
        return fres;
    }

    /* Read all records and extract write_count */
    for(uint16_t i = 0; i < MAX_RECORDS; i++)
    {
        fres = f_read(&tmp_history_fil,
                      &record,
                      sizeof(LogRecord_t),
                      &br);

        if((fres != FR_OK) ||
           (br != sizeof(LogRecord_t)))
        {
            /* Partial file */
            sd_index = i;

            g_write_count =
                    (i == 0) ?
                    0 :
                    (counts[i - 1] + 1);

            f_close(&tmp_history_fil);

            printf("Recovered (partial) sd_index = %u\r\n",
                   sd_index);

            return SaveRecoveredIndex();
        }

        counts[i] = get_write_count(&record);
    }

    f_close(&tmp_history_fil);

    /* Detect circular-buffer wrap point */
    for(uint16_t i = 0; i < (MAX_RECORDS - 1); i++)
    {
        /*
         * Example:
         * 498,499,500,501
         * Normal difference = +1
         *
         * 4294967295,0
         * Difference becomes huge due to wrap
         */
        if((uint32_t)(counts[i + 1] - counts[i])
                > 0x80000000UL)
        {
            sd_index = i + 1;

            g_write_count = counts[i] + 1;

            printf("Recovered (circular) sd_index = %u\r\n",
                   sd_index);

            return SaveRecoveredIndex();
        }
    }

    /*
     * No wrap found.
     * Buffer is either:
     * 1. Completely full and wrapped exactly to slot 0
     * 2. Contains monotonically increasing counters
     */
    sd_index = 0;
    g_write_count = counts[MAX_RECORDS - 1] + 1;

    printf("Recovered (full wrap) sd_index = 0\r\n");

    return SaveRecoveredIndex();
}

FRESULT Logger_LoadIndex(void)
{
    UINT br;
    FRESULT fres;
    IndexFile_t idx;

    fres = f_open(&g_index_fil, "index.bin", FA_OPEN_EXISTING | FA_READ);

    if (fres == FR_NO_FILE)
    {
        sd_index      = 0;
        g_write_count = 0;
        f_open(&g_index_fil,
               "index.bin",
               FA_OPEN_ALWAYS | FA_WRITE);
        FRESULT status = Logger_SaveIndex();
        f_close(&g_index_fil);
        return status;
    }

    if (fres != FR_OK) return fres;

    fres = f_read(&g_index_fil, &idx, sizeof(idx), &br);
    f_close(&g_index_fil);

    if (br != sizeof(idx)              ||
        idx.magic    != INDEX_MAGIC    ||
        idx.checksum != Index_Checksum(idx.sd_index) ||
        idx.sd_index >= MAX_RECORDS)
    {
        printf("Index corrupted! Scanning file to recover...\r\n");
        return Logger_RecoverIndex();
    }

    sd_index = idx.sd_index;
    printf("Loaded sd_index = %u\r\n", sd_index);

    // ── ADDED: restore g_write_count by reading last written record ──
    if(sd_index > 0)
    {
    	FIL tmp_fil;
        LogRecord_t last_record;
        UINT hbr;
        uint16_t last_slot = sd_index - 1;  // last written slot

        fres = f_open(&tmp_fil, "history.bin", FA_OPEN_EXISTING | FA_READ);
        if(fres == FR_OK)
        {
            f_lseek(&tmp_fil, last_slot * sizeof(LogRecord_t));
            f_read(&tmp_fil, &last_record, sizeof(LogRecord_t), &hbr);
            f_close(&tmp_fil);

            if(hbr == sizeof(LogRecord_t))
            {
                g_write_count = get_write_count(&last_record) + 1;
                printf("Restored g_write_count = %lu\r\n", g_write_count);
            }
        }
    }
    else
    {
        g_write_count = 0;
    }

    return FR_OK;
}

FRESULT Logger_SaveIndex(void)
{
    UINT bw;
    FRESULT fres;

    IndexFile_t idx =
    {
        .magic    = INDEX_MAGIC,
        .sd_index = sd_index,
        .checksum = Index_Checksum(sd_index)
    };

    fres = f_lseek(&g_index_fil, 0);

    if(fres == FR_OK)
    {
        fres = f_write(&g_index_fil,
                       &idx,
                       sizeof(idx),
                       &bw);
        if(bw != sizeof(idx))
        {
            return FR_DISK_ERR;
        }
    }

    f_sync(&g_index_fil);
    return fres;
}

FRESULT Logger_WriteRecord(LogRecord_t *record)
{

	UINT bw;
	FRESULT fres;

	uint32_t offset =
			sd_index * sizeof(LogRecord_t);

	fres = f_lseek(&g_history_fil, offset);


	if(fres != FR_OK)
	    return fres;

	fres = f_write(&g_history_fil,
			record,
			sizeof(LogRecord_t),
			&bw);

	if(fres == FR_OK)
	{
	    f_sync(&g_history_fil);
	}

	if((fres == FR_OK) &&
			(bw == sizeof(LogRecord_t)))
	{
		sd_index++;

		if(sd_index >= MAX_RECORDS)
		{
			sd_index = 0;
		}
		Logger_SaveIndex();
	}

	return fres;
}

FRESULT Logger_ReadRecord(uint16_t index,
                          LogRecord_t *record)
{
    UINT br;
    FRESULT fres;

    uint32_t offset =
            index * sizeof(LogRecord_t);

    fres = f_open(&g_fil,
                  "history.bin",
                  FA_READ);

    if(fres != FR_OK)
        return fres;

    fres = f_lseek(&g_fil, offset);

    if(fres != FR_OK)
    {
        f_close(&g_fil);
        return fres;
    }

    fres = f_read(&g_fil,
                  record,
                  sizeof(LogRecord_t),
                  &br);

    f_close(&g_fil);

    if(br != sizeof(LogRecord_t))
    {
        return FR_INT_ERR;
    }

    return fres;
}

FRESULT Logger_ReadRecord_Open(FIL *fil, uint16_t index, LogRecord_t *record)
{
    UINT br;
    FRESULT fres;

    uint32_t offset = index * sizeof(LogRecord_t);

    fres = f_lseek(fil, offset);
    if(fres != FR_OK) return fres;

    fres = f_read(fil, record, sizeof(LogRecord_t), &br);
    if(br != sizeof(LogRecord_t)) return FR_INT_ERR;

    return fres;
}

/* wait SD ready */
static uint8_t SD_ReadyWait(void)
{
	uint8_t res;

	/* timeout 500ms */
	Timer2 = 500;

	/* if SD goes ready, receives 0xFF */
	do {
		res = SPI_RxByte();
	} while ((res != 0xFF) && Timer2);

	return res;
}

/* power on */
static void SD_PowerOn(void)
{
	uint8_t args[6];
	uint32_t cnt = 0x1FFF;

	/* transmit bytes to wake up */
	DESELECT();
	for(int i = 0; i < 10; i++)
	{
		SPI_TxByte(0xFF);
	}

	/* slave select */
	SELECT();

	/* make idle state */
	args[0] = CMD0;		/* CMD0:GO_IDLE_STATE */
	args[1] = 0;
	args[2] = 0;
	args[3] = 0;
	args[4] = 0;
	args[5] = 0x95;		/* CRC */

	SPI_TxBuffer(args, sizeof(args));

	/* wait response */
	while ((SPI_RxByte() != 0x01) && cnt)
	{
		cnt--;
	}

	DESELECT();
	SPI_TxByte(0XFF);

	PowerFlag = 1;
}

/* power off */
static void SD_PowerOff(void)
{
	PowerFlag = 0;
}

/* check power flag */
static uint8_t SD_CheckPower(void)
{
	return PowerFlag;
}

/* receive data block */
static bool SD_RxDataBlock(BYTE *buff, UINT len)
{
	uint8_t token;

	/* timeout 200ms */
	Timer1 = 200;

	/* loop until receive a response or timeout */
	do {
		token = SPI_RxByte();
	} while((token == 0xFF) && Timer1);

	/* invalid response */
	if(token != 0xFE) return FALSE;

	/* receive data */
	for (uint16_t i = 0; i < len; i++)
	{
	    SPI_RxBytePtr(&buff[i]);
	}

	/* discard CRC */
	SPI_RxByte();
	SPI_RxByte();

	return TRUE;
}

/* transmit data block */
#if _USE_WRITE == 1
static bool SD_TxDataBlock(const uint8_t *buff, BYTE token)
{
	uint8_t resp;
	uint8_t i = 0;

	/* wait SD ready */
	if (SD_ReadyWait() != 0xFF) return FALSE;

	/* transmit token */
	SPI_TxByte(token);

	/* if it's not STOP token, transmit data */
	if (token != 0xFD)
	{
		SPI_TxBuffer((uint8_t*)buff, 512);

		/* discard CRC */
		SPI_RxByte();
		SPI_RxByte();

		/* receive response */
		while (i <= 64)
		{
			resp = SPI_RxByte();

			/* transmit 0x05 accepted */
			if ((resp & 0x1F) == 0x05) break;
			i++;
		}

		/* recv buffer clear */
		while (SPI_RxByte() == 0);
	}

	/* transmit 0x05 accepted */
	if ((resp & 0x1F) == 0x05) return TRUE;

	return FALSE;
}
#endif /* _USE_WRITE */

/* transmit command */
static BYTE SD_SendCmd(BYTE cmd, uint32_t arg)
{
	uint8_t crc, res;

	/* wait SD ready */
	if (SD_ReadyWait() != 0xFF) return 0xFF;

	/* transmit command */
	SPI_TxByte(cmd); 					/* Command */
	SPI_TxByte((uint8_t)(arg >> 24)); 	/* Argument[31..24] */
	SPI_TxByte((uint8_t)(arg >> 16)); 	/* Argument[23..16] */
	SPI_TxByte((uint8_t)(arg >> 8)); 	/* Argument[15..8] */
	SPI_TxByte((uint8_t)arg); 			/* Argument[7..0] */

	/* prepare CRC */
	if(cmd == CMD0) crc = 0x95;	/* CRC for CMD0(0) */
	else if(cmd == CMD8) crc = 0x87;	/* CRC for CMD8(0x1AA) */
	else crc = 1;

	/* transmit CRC */
	SPI_TxByte(crc);

	/* Skip a stuff byte when STOP_TRANSMISSION */
	if (cmd == CMD12) SPI_RxByte();

	/* receive response */
	uint8_t n = 10;
	do {
		res = SPI_RxByte();
	} while ((res & 0x80) && --n);

	return res;
}

/***************************************
 * user_diskio.c functions
 **************************************/

/* initialize SD */
DSTATUS SD_disk_initialize(BYTE drv)
{
	uint8_t n, type, ocr[4];

	/* single drive, drv should be 0 */
	if(drv) return STA_NOINIT;

	/* no disk */
	if(Stat & STA_NODISK) return Stat;

	/* power on */
	SD_PowerOn();

	/* slave select */
	SELECT();

	/* check disk type */
	type = 0;

	/* send GO_IDLE_STATE command */
	if (SD_SendCmd(CMD0, 0) == 1)
	{
		/* timeout 1 sec */
		Timer1 = 1000;

		/* SDC V2+ accept CMD8 command, http://elm-chan.org/docs/mmc/mmc_e.html */
		if (SD_SendCmd(CMD8, 0x1AA) == 1)
		{
			/* operation condition register */
			for (n = 0; n < 4; n++)
			{
				ocr[n] = SPI_RxByte();
			}

			/* voltage range 2.7-3.6V */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA)
			{
				/* ACMD41 with HCS bit */
				do {
					if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 1UL << 30) == 0) break;
				} while (Timer1);

				/* READ_OCR */
				if (Timer1 && SD_SendCmd(CMD58, 0) == 0)
				{
					/* Check CCS bit */
					for (n = 0; n < 4; n++)
					{
						ocr[n] = SPI_RxByte();
					}

					/* SDv2 (HC or SC) */
					type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
				}
			}
		}
		else
		{
			/* SDC V1 or MMC */
			type = (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) <= 1) ? CT_SD1 : CT_MMC;

			do
			{
				if (type == CT_SD1)
				{
					if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) == 0) break; /* ACMD41 */
				}
				else
				{
					if (SD_SendCmd(CMD1, 0) == 0) break; /* CMD1 */
				}

			} while (Timer1);

			/* SET_BLOCKLEN */
			if (!Timer1 || SD_SendCmd(CMD16, 512) != 0) type = 0;
		}
	}

	CardType = type;

	/* Idle */
	DESELECT();
	SPI_RxByte();

	/* Clear STA_NOINIT */
	if (type)
	{
		Stat &= ~STA_NOINIT;
	}
	else
	{
		/* Initialization failed */
		SD_PowerOff();
	}

	return Stat;
}

/* return disk status */
DSTATUS SD_disk_status(BYTE drv)
{
	if (drv) return STA_NOINIT;
	return Stat;
}

/* read sector */
DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count)
{
	/* pdrv should be 0 */
	if (pdrv || !count) return RES_PARERR;

	/* no disk */
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	/* convert to byte address */
	if (!(CardType & CT_SD2)) sector *= 512;

	SELECT();

	if (count == 1)
	{
		/* READ_SINGLE_BLOCK */
		if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512)) count = 0;
	}
	else
	{
		/* READ_MULTIPLE_BLOCK */
		if (SD_SendCmd(CMD18, sector) == 0)
		{
			do {
				if (!SD_RxDataBlock(buff, 512)) break;
				buff += 512;
			} while (--count);

			/* STOP_TRANSMISSION */
			SD_SendCmd(CMD12, 0);
		}
	}

	/* Idle */
	DESELECT();
	SPI_RxByte();

	return count ? RES_ERROR : RES_OK;
}

/* write sector */
#if _USE_WRITE == 1
DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
	/* pdrv should be 0 */
	if (pdrv || !count) return RES_PARERR;

	/* no disk */
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	/* write protection */
	if (Stat & STA_PROTECT) return RES_WRPRT;

	/* convert to byte address */
	if (!(CardType & CT_SD2)) sector *= 512;

	SELECT();

	if (count == 1)
	{
		/* WRITE_BLOCK */
		if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE))
			count = 0;
	}
	else
	{
		/* WRITE_MULTIPLE_BLOCK */
		if (CardType & CT_SD1)
		{
			SD_SendCmd(CMD55, 0);
			SD_SendCmd(CMD23, count); /* ACMD23 */
		}

		if (SD_SendCmd(CMD25, sector) == 0)
		{
			do {
				if(!SD_TxDataBlock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);

			/* STOP_TRAN token */
			if(!SD_TxDataBlock(0, 0xFD))
			{
				count = 1;
			}
		}
	}

	/* Idle */
	DESELECT();
	SPI_RxByte();

	return count ? RES_ERROR : RES_OK;
}
#endif /* _USE_WRITE */

/* ioctl */
DRESULT SD_disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
	DRESULT res;
	uint8_t n, csd[16], *ptr = buff;
	WORD csize;

	/* pdrv should be 0 */
	if (drv) return RES_PARERR;
	res = RES_ERROR;

	if (ctrl == CTRL_POWER)
	{
		switch (*ptr)
		{
		case 0:
			SD_PowerOff();		/* Power Off */
			res = RES_OK;
			break;
		case 1:
			SD_PowerOn();		/* Power On */
			res = RES_OK;
			break;
		case 2:
			*(ptr + 1) = SD_CheckPower();
			res = RES_OK;		/* Power Check */
			break;
		default:
			res = RES_PARERR;
		}
	}
	else
	{
		/* no disk */
		if (Stat & STA_NOINIT) return RES_NOTRDY;

		SELECT();

		switch (ctrl)
		{
		case GET_SECTOR_COUNT:
			/* SEND_CSD */
			if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16))
			{
				if ((csd[0] >> 6) == 1)
				{
					/* SDC V2 */
					csize = csd[9] + ((WORD) csd[8] << 8) + 1;
					*(DWORD*) buff = (DWORD) csize << 10;
				}
				else
				{
					/* MMC or SDC V1 */
					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
					csize = (csd[8] >> 6) + ((WORD) csd[7] << 2) + ((WORD) (csd[6] & 3) << 10) + 1;
					*(DWORD*) buff = (DWORD) csize << (n - 9);
				}
				res = RES_OK;
			}
			break;
		case GET_SECTOR_SIZE:
			*(WORD*) buff = 512;
			res = RES_OK;
			break;
		case CTRL_SYNC:
			if (SD_ReadyWait() == 0xFF) res = RES_OK;
			break;
		case MMC_GET_CSD:
			/* SEND_CSD */
			if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(ptr, 16)) res = RES_OK;
			break;
		case MMC_GET_CID:
			/* SEND_CID */
			if (SD_SendCmd(CMD10, 0) == 0 && SD_RxDataBlock(ptr, 16)) res = RES_OK;
			break;
		case MMC_GET_OCR:
			/* READ_OCR */
			if (SD_SendCmd(CMD58, 0) == 0)
			{
				for (n = 0; n < 4; n++)
				{
					*ptr++ = SPI_RxByte();
				}
				res = RES_OK;
			}
			break;
		default:
			res = RES_PARERR;
			break;
		}

		DESELECT();
		SPI_RxByte();
	}

	return res;
}

void sd_task(void)
{
	if(g_sd_present){     /* g_sd_present :SD Present on Board*/
		static bool server_conn_failed_flag = false;
		bool status = modbus_task_sd();
		if(status == true){
			read_sd_flag = false;
		}
		else
		{
			static bool server_wait_flag = true;
			static uint32_t server_tick = 0;
			static uint8_t server_fail_cnt = 0;
            if(HAL_GetTick() - server_tick > 1000)
            {
            	server_fail_cnt += 1;
            	server_tick = HAL_GetTick();
            }

            if(server_wait_flag == true)
            {
            	display_server_wait();
            	server_wait_flag = false;
            }
            if(server_fail_cnt >= 30)
            {
            	server_fail_cnt = 0;
            	server_conn_failed_flag = true;
            	server_wait_flag = true;
            	read_sd_flag = false;
            }
		}

		if(server_conn_failed_flag == true)
		{
	       server_conn_failed_flag = false;
	   	   display_server_failed_status();
		}
	}
	else
	{
		read_sd_flag = false;
	}
}

bool modbus_task_sd(void)
{
	f_close(&g_history_fil);
	f_close(&g_index_fil);

	// ── Connect to PC as Master ──
	if(Modbus_Registers.SD_MODE == 1){
		if(!modbus_client_connect())
		{
			// printf("SD send aborted — cannot reach PC\r\n");
			// Reopen files and return
			f_open(&g_history_fil, "history.bin", FA_OPEN_EXISTING | FA_WRITE);
			f_open(&g_index_fil,   "index.bin",   FA_OPEN_ALWAYS   | FA_WRITE);
			return false;
		}
	}
	uint16_t i     = (sd_index == 0) ? (MAX_RECORDS - 1) : (sd_index - 1);
	uint16_t start = i;

	FRESULT fres = f_open(&g_read_fil, "history.bin", FA_READ);
	if(fres != FR_OK)
	{
		// printf("modbus_task_sd open failed: %d\r\n", fres);
		modbus_client_disconnect();
		return false;
	}

	Modbus_Restart_RX_DMA_PC_SD();
	display_data_writing();
	do
	{
		HAL_Delay(1);
		Logger_ReadRecord_Open(&g_read_fil, i, &read_sd_logs);

		if(Modbus_Registers.SD_MODE == 0){
			//            Modbus_FC_10_sd_rs485();
			Modbus_Send_Record_And_WaitAck();
		}
		else{
			uint16_t txn_id  = Modbus_FC_10_sd_tcp();
			ModbusAckResult ack = Modbus_FC10_WaitAck(txn_id);
			if(ack != MODBUS_ACK_OK)
			{
				// printf("Record %u failed (ack=%d) — skipping\r\n", i, ack);
				/* Options: break, retry, or just skip this record */
			}
		}
		HAL_Delay(1);

		if(i == 0)
			i = MAX_RECORDS - 1;
		else
			i -= 1;

	} while(i != start);

	HAL_Delay(1);
	f_close(&g_read_fil);

	modbus_client_disconnect();

	f_open(&g_history_fil, "history.bin", FA_OPEN_EXISTING | FA_WRITE);
	f_open(&g_index_fil,   "index.bin",   FA_OPEN_ALWAYS   | FA_WRITE);

	// Reset For Normal PC Modbus
	Modbus_Restart_RX_DMA_PC();
	display_data_writing_done();

	return true;
}

void open_files(void)
{
	f_open(&g_history_fil,
			"history.bin",
			FA_OPEN_EXISTING | FA_WRITE);

	f_open(&g_index_fil,
			"index.bin",
			FA_OPEN_ALWAYS | FA_WRITE);
}


//void modbus_task_sd(void)
//{
//	f_close(&g_history_fil);
//	f_close(&g_index_fil);
//
//    uint16_t i = (sd_index == 0) ? (MAX_RECORDS - 1) : (sd_index - 1);
//    uint16_t start = i;
//
//    FRESULT fres = f_open(&g_read_fil, "history.bin", FA_READ);
//    if(fres != FR_OK)
//    {
//        printf("modbus_task_sd open failed: %d\r\n", fres);
//        return;
//    }
//
//    do
//    {
//        // seek + read only, no open/close per iteration
//        Logger_ReadRecord_Open(&g_read_fil, i, &read_sd_logs);
//
//        if(Modbus_Registers.SD_MODE == 0)
//            Modbus_FC_10_sd_rs485();
//        else
//            Modbus_FC_10_sd_tcp();
//
//        HAL_Delay(50);
//
//        if(i == 0)
//            i = MAX_RECORDS - 1;
//        else
//            i -= 1;
//
//    } while(i != start);
//
//    // ── Close ONCE after loop ──
//    f_close(&g_read_fil);
//
//    f_open(&g_history_fil,
//           "history.bin",
//           FA_OPEN_EXISTING | FA_WRITE);
//
//    f_open(&g_index_fil,
//           "index.bin",
//           FA_OPEN_ALWAYS | FA_WRITE);
//}



//
//FRESULT Logger_RecoverIndex(void)
//{
//	FIL tmp_history_fil;
//	UINT br;
//	FRESULT fres;
//	LogRecord_t record;
//	static uint32_t counts[MAX_RECORDS];
//
//	fres = f_open(&tmp_history_fil, "history.bin", FA_OPEN_EXISTING | FA_READ);
//	if (fres != FR_OK) { sd_index = 0; return fres; }
//
//	// Read all write_count values
//	for (uint16_t i = 0; i < MAX_RECORDS; i++)
//	{
//		fres = f_read(&tmp_history_fil, &record, sizeof(LogRecord_t), &br);
//		if (fres != FR_OK || br != sizeof(LogRecord_t))
//		{
//			// Partial file — not yet fully written, simple case
//			sd_index = i;
//			f_close(&tmp_history_fil);
//			printf("Recovered (partial) sd_index = %u\r\n", sd_index);
//			g_write_count = (i == 0) ? 0 : counts[i - 1] + 1;
//			f_open(&g_index_fil,
//					"index.bin",
//					FA_OPEN_ALWAYS | FA_WRITE);
//			FRESULT status = Logger_SaveIndex();
//			f_close(&g_index_fil);
//			return status;
//
//		}
//
//		// ── CHANGED: combine MSB+LSB instead of record.write_count ──
//		counts[i] = get_write_count(&record);
//	}
//
//	f_close(&tmp_history_fil);
//
//	// Find where counter drops — that's the oldest record = current write pos
//	// Example: [500,501,502,...,998,999, 0,1,2,...,499]
//	//                                   ↑ this is sd_index
//	for (uint16_t i = 0; i < MAX_RECORDS - 1; i++)
//	{
//		// ── CHANGED: rollover-safe comparison for uint32_t ──
//		if ((uint32_t)(counts[i+1] - counts[i]) > 0x80000000UL)
//		{
//			sd_index = i + 1;
//			printf("Recovered (circular) sd_index = %u\r\n", sd_index);
//
//			// Also recover g_write_count
//			g_write_count = counts[i] + 1;
//			f_open(&g_index_fil,
//					"index.bin",
//					FA_OPEN_ALWAYS | FA_WRITE);
//			FRESULT status = Logger_SaveIndex();
//			f_close(&g_index_fil);
//			return status;
//
//		}
//	}
//
//	// No wrap found — buffer never wrapped, next slot is after last
//	sd_index = 0;  // wrapped perfectly back to 0
//	g_write_count = counts[MAX_RECORDS - 1] + 1;
//	printf("Recovered (full wrap) sd_index = 0\r\n");
//	f_open(&g_index_fil,
//			"index.bin",
//			FA_OPEN_ALWAYS | FA_WRITE);
//	FRESULT status = Logger_SaveIndex();
//	f_close(&g_index_fil);
//	return status;
//}

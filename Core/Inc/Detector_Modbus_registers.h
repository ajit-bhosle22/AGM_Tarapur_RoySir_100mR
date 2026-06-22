
#ifndef DETECTOR_MODBUS_REGISTERS_H
#define DETECTOR_MODBUS_REGISTERS_H

#define TIMEOUT_MS                2000
#define DTC_HV_SWITCH_ADDR        3
#define DTC_POLLING_START_ADDR    0
#define DTC_POLLING_REG_COUNT     10
#define DTC_FREQ_SWITCH_ADDR      8

typedef enum
{
	HV_WRITE_FLAG,
	CALIB_WRITE_FLAG,
	HV_AND_CALIB_WRITE_FLAG,
	HV_AND_FREQ_WRITE_FLAG,
	ALL_CONFIG_WRITE_FLAG,
	MULTIPLE_WRITE_DONE
}Modbus_Multiple_Write_Flag_t;

typedef enum
{
    HV_SWITCH_FLAG,
	DTC_FREQ_SWITCH_FLAG,
	SINGLE_WRITE_DONE
}Modbus_Single_Write_Flag_t;

typedef enum
{
  MODBUS_SINGLE_WRITE_FLAG,
  MODBUS_MULTIPLE_WRITE_FLAG,
  MODBUS_POLLING_FLAG
}Modbus_Write_Flag_t;

typedef enum {
    MODBUS_IDLE_DTC,
	MODBUS_SEND_WRITE_SINGLE,
    MODBUS_SEND_WRITE_MULTIPLE,
    MODBUS_SEND_READ,
    MODBUS_WAIT_RESPONSE
} DTC_Modbus_State_t;

typedef struct
{
	uint8_t  func_code_dtc;
	uint8_t  valid_resp;
}__attribute__((packed))dtc_rx_t;

typedef struct
{
	uint16_t CPS_MSB;                        //Addr0       GM APPLICATION
	uint16_t CPS_LSB;                        //Addr1
	uint16_t Gm_Tube;                        //Addr2       GM1/GM2 ---> 0xAA/0x55
	uint16_t HV_SWITCH;                      //Addr3
	uint16_t HV_MSB;                         //Addr4
	uint16_t HV_LSB;                         //Addr5
	uint16_t Reserved_1;                     //Addr6
	uint16_t Reserved_2;                     //Addr7
	uint16_t DTC_FRQ_SWITCH;                 //Addr8
	uint16_t CALIB_FACTOR1;                  //Addr9
	uint16_t CALIB_FACTOR2;                  //Addr10
	uint16_t CALIB_FACTOR3;                  //Addr11
	uint16_t CALIB_FACTOR4;                  //Addr12
	uint16_t CALIB_FACTOR5;                  //Addr13
	uint16_t CALIB_FACTOR6;                  //Addr14
	uint16_t CALIB_FACTOR7;                  //Addr15
	uint16_t CALIB_FACTOR8;                  //Addr16
	uint16_t CALIB_FACTOR9;                  //Addr17
	uint16_t CALIB_FACTOR10;                 //Addr18
	uint16_t CALIB_FACTOR11;                 //Addr19
	uint16_t CALIB_FACTOR12;                 //Addr20
	uint16_t CALIB_FACTOR13;                 //Addr21
	uint16_t CALIB_FACTOR14;                 //Addr22
	uint16_t CALIB_FACTOR15;                 //Addr23
	uint16_t CALIB_FACTOR16;                 //Addr24
	uint16_t CALIB_FACTOR17;                 //Addr25
	uint16_t CALIB_FACTOR18;                 //Addr26
	uint16_t CALIB_FACTOR19;                 //Addr27
	uint16_t CALIB_FACTOR20;                 //Addr28
}__attribute__((packed)) master_modbus_db_dtc_t;

extern master_modbus_db_dtc_t Modbus_Registers_Detector;
extern master_modbus_db_dtc_t Modbus_Registers_Detector_Write;
extern Modbus_Multiple_Write_Flag_t multiple_write_flag;
extern Modbus_Single_Write_Flag_t single_write_flag;
extern Modbus_Write_Flag_t modbus_write_flag;

extern uint8_t rx_buffer_dma_dtc[255];
extern uint8_t tx_buffer_dma_dtc[255];
extern volatile uint8_t Is_Tx_Done_dtc;
extern volatile uint8_t  frame_recv_dtc;
extern volatile uint16_t rx_index_dtc;
extern volatile uint8_t DTC_UART_Error_Flag;
extern volatile bool dtc_write;
extern volatile dtc_rx_t dtc_rx_frame;
extern uint16_t dtc_freq_switch_val;

void Modbus_Process_Request_dtc(uint8_t *rx_buffer, uint16_t Size);
void Modbus_Master_Poll_Dtc(uint16_t start_addr,uint16_t reg_count);
void USART2_Start_RX(UART_HandleTypeDef *huart);
void DTC_UART_ReInit(void);
void Modbus_Restart_RX_DMA_DTC();
void Modbus_Master_Write_Dtc(uint16_t);
void modbus_task_dtc(void);
void detector_write_multiple_registers(uint16_t start_addr,uint16_t reg_count);
void detector_write_single_register(uint16_t reg_addr,uint16_t reg_value);

#endif


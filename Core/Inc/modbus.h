#ifndef __MODBUS_H
#define __MODBUS_H

#include <stdbool.h>
#include "main.h"

#define REG_ACK                     16
#define REG_RESET                   17
#define REG_READ_SD                 6

#define BUFFER_SIZE                 255
#define MAX_HOLDING_REGS            76

#define RS485_TX_MODE_DTC()         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET)
#define RS485_RX_MODE_DTC()         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET)

#define RS485_TX_MODE_PC()          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define RS485_RX_MODE_PC()          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)

#define RS485_TX_MODE()             HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET)
#define RS485_RX_MODE()             HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET)

#define GM_TUBE_SENSITIVITY         18.0f

//#define CALIB_FACT_START_ADDR       47
#define MAX_REG_WRITE               42

typedef struct {
    uint16_t    last_write_addr;
    uint16_t    last_reg_cnt;
    uint8_t     single_write;
    uint8_t     multiple_write;
} Write_ModbusState_t;

typedef struct
{
	uint16_t RTC_DAY;                         //Addr0
	uint16_t RTC_MONTH;                       //Addr1
	uint16_t RTC_YEAR;                        //Addr2
	uint16_t RTC_HOUR;                        //Addr3
	uint16_t RTC_MIN;                         //Addr4
	uint16_t RTC_SEC;                         //Addr5
	uint16_t Overload;                        //Addr6
	uint16_t Overrun;                         //Addr7
	uint16_t Alarm;                           //Addr8
	uint16_t Hv_Fault;                        //Addr9
	uint16_t Dtc_Fault;                       //Addr10
	uint16_t CPS_MSB;                         //Addr11
	uint16_t CPS_LSB;                         //Addr12
	uint16_t mR_MSB;                          //Addr13
	uint16_t mR_LSB;                          //Addr14
	uint16_t READ_SD;                         //Addr15
	uint16_t ACK;                             //Addr16
	uint16_t Reset;                           //Addr17
	uint16_t AUDIO_MODE;                      //Addr18
	uint16_t Primary_Unit;                    //Addr19
	uint16_t AGM_MODE;                        //Addr20
	uint16_t FREQ_RECV;                       //Addr21
	uint16_t FREQ_DTC;                        //Addr22
	uint16_t Alarm_Value_MSB;                 //Addr23
	uint16_t Alarm_Value_LSB;                 //Addr24
	uint16_t Alarm_Unit;                      //Addr25
	uint16_t RS485_Slave_Id;                  //Addr26
	uint16_t RS485_Baud_Rate;                 //Addr27
	uint16_t PASSWORD_MSB;                    //Addr28
	uint16_t PASSWORD_LSB;                    //Addr29
	uint16_t Ethernet_IP_MSB;                 //Addr30
	uint16_t Ethernet_IP_LSB;                 //Addr31
	uint16_t Ethernet_Subnet_MSB;             //Addr32
	uint16_t Ethernet_Subnet_LSB;             //Addr33
	uint16_t Ethernet_Gateway_MSB;            //Addr34
	uint16_t Ethernet_Gateway_LSB;            //Addr35
	uint16_t Ethernet_Slave_Id;               //Addr36
	uint16_t Ethernet_Port;                   //Addr37
	uint16_t Overload_MSB;                    //Addr38
	uint16_t Overload_LSB;                    //Addr39
	uint16_t Overload_Unit;                   //Addr40
	uint16_t Overrange_MSB;                   //Addr41
	uint16_t Overrange_LSB;                   //Addr42
	uint16_t Overrange_Unit;                  //Addr43
	uint16_t Analog_4_to_20mA_Min_MSB;        //Addr44
	uint16_t Analog_4_to_20mA_Min_LSB;        //Addr45
	uint16_t Analog_4_to_20mA_Min_Unit;       //Addr46
	uint16_t Analog_4_to_20mA_Max_MSB;        //Addr47
	uint16_t Analog_4_to_20mA_Max_LSB;        //Addr48
	uint16_t Analog_4_to_20mA_Max_Unit;       //Addr49
	uint16_t HV_MSB;                          //Addr50
	uint16_t HV_LSB;                          //Addr51
	uint16_t E4MA_CALIB_MSB;                  //Addr52
	uint16_t E4MA_CALIB_LSB;                  //Addr53
	uint16_t E20MA_CALIB_MSB;                 //Addr54
	uint16_t E20MA_CALIB_LSB;                 //Addr55
	uint16_t CALIB_FACTOR1;                   //Addr56
	uint16_t CALIB_FACTOR2;                   //Addr57
	uint16_t CALIB_FACTOR3;                   //Addr58
	uint16_t CALIB_FACTOR4;                   //Addr59
	uint16_t CALIB_FACTOR5;                   //Addr60
	uint16_t CALIB_FACTOR6;                   //Addr61
	uint16_t CALIB_FACTOR7;                   //Addr62
	uint16_t CALIB_FACTOR8;                   //Addr63
	uint16_t CALIB_FACTOR9;                   //Addr64
	uint16_t CALIB_FACTOR10;                  //Addr65
	uint16_t CALIB_FACTOR11;                  //Addr66
	uint16_t CALIB_FACTOR12;                  //Addr67
	uint16_t CALIB_FACTOR13;                  //Addr68
	uint16_t CALIB_FACTOR14;                  //Addr69
	uint16_t CALIB_FACTOR15;                  //Addr70
	uint16_t CALIB_FACTOR16;                  //Addr71
	uint16_t CALIB_FACTOR17;                  //Addr72
	uint16_t CALIB_FACTOR18;                  //Addr73
	uint16_t CALIB_FACTOR19;                  //Addr74
	uint16_t CALIB_FACTOR20;                  //Addr75
}__attribute__((packed)) master_modbus_db_t;

extern master_modbus_db_t Modbus_Registers;
extern master_modbus_db_t Modbus_Registers_Write;
extern master_modbus_db_t Modbus_Registers_PC_TCP;
extern master_modbus_db_t Modbus_Registers_PC_TCP_Write;

extern Write_ModbusState_t pc_state;
extern Write_ModbusState_t tcp_state;

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;

extern uint32_t cpm;
extern double dose_mRh;
extern uint8_t ip[4];
extern uint8_t sn[4];
extern uint8_t gw[4];
extern volatile uint16_t tcp_slave_id;
extern volatile uint16_t tcp_current_port;
extern bool tcp_reconfig_required;
extern bool rs485_reconfig_required;
extern bool read_sd_flag;
extern char Factor1_Value[12];
extern char Factor2_Value[12];
extern char Factor3_Value[12];
extern char Factor4_Value[12];

void initialize_modbus_registers();
void Update_Modbus_Registers(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len);
void GetUnitString(uint16_t unit, char *dest);
int8_t GetUnitNumber(char *unit);
void update_configuration(uint16_t start_addr);
void Update_Registers(void);
void config_modbus_registers(void);
void config_variables(void);
void conf_regs_stg_tcp(uint16_t start_addr,uint16_t reg_cnt);
void e4_20mA_calib(void);
void cpy_4_20mA_factors_eeprom_reg();
void is_reconfigure_rs485_tcp_pc(void);
double convert_cps_to_mr_h();
void Reconfigure_RS485(void);
void Fill_Record(void);
void read_e4_20ma_CalibVal(void);
void read_e4_20mA_factors(void);
void update_cps_hv_mr(void);
void config_CalibFactors_Variables(void);
void update_detector_calib_registers(void);
void update_detector_hv_register(void);
void detector_write(uint16_t start_addr , uint16_t reg_cnt);
void conf_rs485_tcp(void);
void conf_tcp_rs485(void);
int GetBaudRate(uint8_t baud_index);
void is_reconfigure_rs485_tcp(void);
void is_reconfigure_hv_rs485(void);
void is_reconfigure_tcp_rs485(void);
void is_reconfigure_hv_tcp(void);
void update_detector_hv_register_tcp(void);
void update_detector_calib_registers_tcp(void);
void update_detector_hv_register_rs485(void);
void update_detector_calib_registers_rs485(void);
void detector_write_rs485(uint16_t start_addr , uint16_t reg_cnt);
void update_detector_hv_register_rs485(void);
void update_detector_calib_registers_rs485(void);
bool Validate_Date_RS485_Registers(uint16_t day,uint16_t month,uint16_t year);
bool Validate_Time_RS485_Registers(uint16_t hour,uint16_t min,uint16_t sec);
void update_rtc_registers(void);
void is_reconfigure_rtc_rs485(void);
void is_reconfigure_rtc_tcp(void);
void is_sd_card_read_rs485(void);
void is_sd_card_read_tcp(void);

#endif

#ifndef USER_MAIN_H_
#define USER_MAIN_H_

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern ADC_HandleTypeDef hadc1;

void UserInit(void);
void UserMain(void);
void Debug(void);
void UpdateLCD(void);
void HeartDataProcess(void);
void StepDataProcess(void);
void Gyro1DataProcess(void);
void DataPacketTransfer(void);
//int32_t get_volt(uint32_t num);
//void ADS1292DataTransfer(void);

//int8_t user_i2c_read1(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
//int8_t user_i2c_write1(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
//void user_delay1(uint32_t period);
//void test();

#endif
#ifndef GYRO_H_
#define GYRO_H_

#include "main.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

void GyroUARTInit(void);

#endif
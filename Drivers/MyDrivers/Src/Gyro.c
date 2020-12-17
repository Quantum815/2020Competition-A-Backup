#include "Gyro.h"

uint8_t GyroUnlockInstruction[5] = {0xff, 0xaa, 0x69, 0x88, 0xb5};  //解锁指令
uint8_t GyroAutoCalibration[5] = {0xff, 0xaa, 0x63, 0x00, 0x00};  //陀螺仪自动校准
uint8_t GyroKeepConfiguration[5] = {0xff, 0xaa, 0x00, 0x00, 0x00};  //保持配置

void GyroUARTInit(void)
{
	HAL_UART_Transmit(&huart4, GyroUnlockInstruction, 5, 10);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart4, GyroAutoCalibration, 5, 10);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart4, GyroKeepConfiguration, 5, 10);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart6, GyroUnlockInstruction, 5, 10);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart6, GyroAutoCalibration, 5, 10);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart6, GyroKeepConfiguration, 5, 10);
	HAL_Delay(100);
}
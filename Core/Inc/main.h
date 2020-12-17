/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "sys.h"
#include "MatrixKeyboard.h"
#include "MLX90614.h"
#include "LMT70.h"
#include "LCD.h"
#include "Gui.h"
#include "ADS1292.h"
#include "Gyro.h"
#include "BMI160.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_DC_Pin GPIO_PIN_2
#define LCD_DC_GPIO_Port GPIOE
#define Row1_Pin GPIO_PIN_2
#define Row1_GPIO_Port GPIOF
#define Row2_Pin GPIO_PIN_3
#define Row2_GPIO_Port GPIOF
#define Row3_Pin GPIO_PIN_4
#define Row3_GPIO_Port GPIOF
#define Row4_Pin GPIO_PIN_5
#define Row4_GPIO_Port GPIOF
#define Column1_Pin GPIO_PIN_6
#define Column1_GPIO_Port GPIOF
#define Column2_Pin GPIO_PIN_7
#define Column2_GPIO_Port GPIOF
#define Column3_Pin GPIO_PIN_8
#define Column3_GPIO_Port GPIOF
#define Column4_Pin GPIO_PIN_9
#define Column4_GPIO_Port GPIOF
#define LMT70_IN_Pin GPIO_PIN_0
#define LMT70_IN_GPIO_Port GPIOA
#define TwoM_Tx_Pin GPIO_PIN_2
#define TwoM_Tx_GPIO_Port GPIOA
#define TwoM_Rx_Pin GPIO_PIN_3
#define TwoM_Rx_GPIO_Port GPIOA
#define LCD_SCK_Pin GPIO_PIN_5
#define LCD_SCK_GPIO_Port GPIOA
#define LCD_MISO_Pin GPIO_PIN_6
#define LCD_MISO_GPIO_Port GPIOA
#define LCD_MOSI_Pin GPIO_PIN_7
#define LCD_MOSI_GPIO_Port GPIOA
#define Transfer_Tx_Pin GPIO_PIN_10
#define Transfer_Tx_GPIO_Port GPIOB
#define Transfer_Rx_Pin GPIO_PIN_11
#define Transfer_Rx_GPIO_Port GPIOB
#define ADS1292_SCK_Pin GPIO_PIN_13
#define ADS1292_SCK_GPIO_Port GPIOB
#define ADS1292_MISO_Pin GPIO_PIN_14
#define ADS1292_MISO_GPIO_Port GPIOB
#define ADS1292_MOSI_Pin GPIO_PIN_15
#define ADS1292_MOSI_GPIO_Port GPIOB
#define ADS1292_CS_Pin GPIO_PIN_8
#define ADS1292_CS_GPIO_Port GPIOD
#define ADS1292_DRDY_Pin GPIO_PIN_9
#define ADS1292_DRDY_GPIO_Port GPIOD
#define ADS1292_START_Pin GPIO_PIN_10
#define ADS1292_START_GPIO_Port GPIOD
#define ADS1292_PWDN_Pin GPIO_PIN_11
#define ADS1292_PWDN_GPIO_Port GPIOD
#define JY_Tx2_Pin GPIO_PIN_6
#define JY_Tx2_GPIO_Port GPIOC
#define JY_Rx2_Pin GPIO_PIN_7
#define JY_Rx2_GPIO_Port GPIOC
#define Debug_Tx_Pin GPIO_PIN_9
#define Debug_Tx_GPIO_Port GPIOA
#define Debug_Rx_Pin GPIO_PIN_10
#define Debug_Rx_GPIO_Port GPIOA
#define JY_Tx1_Pin GPIO_PIN_10
#define JY_Tx1_GPIO_Port GPIOC
#define JY_Rx1_Pin GPIO_PIN_11
#define JY_Rx1_GPIO_Port GPIOC
#define Packet_CLK_Pin GPIO_PIN_0
#define Packet_CLK_GPIO_Port GPIOD
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOG
#define LCD_RESET_Pin GPIO_PIN_0
#define LCD_RESET_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_1
#define LCD_CS_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

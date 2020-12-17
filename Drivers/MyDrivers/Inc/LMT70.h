#ifndef LMT70_H_
#define LMT70_H_

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern uint8_t DataPacket[22];
extern float ContactTemperature;  //½Ó´¥ÎÂ¶È

float LMT70_ReadContactTemperature(int FilterTimes);
void LMT70_GetContactTemperature(void);

#endif
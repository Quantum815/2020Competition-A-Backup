#include "LMT70.h"

float LMT70_ReadContactTemperature(int FilterTimes)
{
	int i;
	uint32_t value[FilterTimes+1];
	uint32_t averageValue;
	uint32_t minValue;
	uint32_t maxValue;
	uint32_t allValue = 0;
	float Temperature;
	
	for(i=0; i<FilterTimes; i++)  //ÂË²¨
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);       
		if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
		{
			value[i] = HAL_ADC_GetValue(&hadc1);
			allValue += value[i];
		}                
	}
	
	minValue = value[0];
	maxValue = value[0];
	for(i=0; i<FilterTimes; i++)
	{
		minValue = (minValue<value[i])?minValue:value[i];
		maxValue = (value[i]<maxValue)?maxValue:value[i];    
	}
	averageValue = (allValue-minValue-maxValue)/(FilterTimes-2)*(3.3/4096)*1000;         
	Temperature = (-0.0000084515)*averageValue*averageValue+(-0.176928)*averageValue+204.393;
	
	return Temperature;
}

void LMT70_GetContactTemperature(void)
{
	int value;
    float temp;
	
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 500);
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
		value=HAL_ADC_GetValue(&hadc1);
	DataPacket[18] = value;
	DataPacket[19] = value >> 8;
    temp=(float)value*(3.3/4096)*1000;
    ContactTemperature = (-0.0000084515)*temp*temp+(-0.176928)*temp+204.393;
//	ContactTemperature = value;
}
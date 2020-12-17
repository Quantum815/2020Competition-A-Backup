#include "MatrixKeyboard.h"

int KeyFlag;

void MatrixKeyboardScanning(void)
{
	HAL_GPIO_WritePin(Row1_GPIO_Port, Row1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Row2_GPIO_Port, Row2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Row3_GPIO_Port, Row3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Row4_GPIO_Port, Row4_Pin, GPIO_PIN_SET);
	if(!HAL_GPIO_ReadPin(Column1_GPIO_Port, Column1_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column1_GPIO_Port, Column1_Pin))
			KeyFlag = 1;
	}
	else if(!HAL_GPIO_ReadPin(Column2_GPIO_Port, Column2_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column2_GPIO_Port, Column2_Pin))
			KeyFlag = 2;
	}
	else if(!HAL_GPIO_ReadPin(Column3_GPIO_Port, Column3_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column3_GPIO_Port, Column3_Pin))
			KeyFlag = 3;
	}
	else if(!HAL_GPIO_ReadPin(Column4_GPIO_Port, Column4_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column4_GPIO_Port, Column4_Pin))
			KeyFlag = 4;
	}
	else
		KeyFlag = 0;
	
	HAL_GPIO_WritePin(Row1_GPIO_Port, Row1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Row2_GPIO_Port, Row2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Row3_GPIO_Port, Row3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Row4_GPIO_Port, Row4_Pin, GPIO_PIN_SET);
	if(!HAL_GPIO_ReadPin(Column1_GPIO_Port, Column1_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column1_GPIO_Port, Column1_Pin))
			KeyFlag = 5;
	}
	else if(!HAL_GPIO_ReadPin(Column2_GPIO_Port, Column2_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column2_GPIO_Port, Column2_Pin))
			KeyFlag = 6;
	}
	else if(!HAL_GPIO_ReadPin(Column3_GPIO_Port, Column3_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column3_GPIO_Port, Column3_Pin))
			KeyFlag = 7;
	}
	else if(!HAL_GPIO_ReadPin(Column4_GPIO_Port, Column4_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column4_GPIO_Port, Column4_Pin))
			KeyFlag = 8;
	}

	HAL_GPIO_WritePin(Row1_GPIO_Port, Row1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Row2_GPIO_Port, Row2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Row3_GPIO_Port, Row3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Row4_GPIO_Port, Row4_Pin, GPIO_PIN_SET);	
	if(!HAL_GPIO_ReadPin(Column1_GPIO_Port, Column1_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column1_GPIO_Port, Column1_Pin))
			KeyFlag = 9;
	}
	else if(!HAL_GPIO_ReadPin(Column2_GPIO_Port, Column2_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column2_GPIO_Port, Column2_Pin))
			KeyFlag = 10;
	}
	else if(!HAL_GPIO_ReadPin(Column3_GPIO_Port, Column3_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column3_GPIO_Port, Column3_Pin))
			KeyFlag = 11;
	}
	else if(!HAL_GPIO_ReadPin(Column4_GPIO_Port, Column4_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column4_GPIO_Port, Column4_Pin))
			KeyFlag = 12;
	}

	HAL_GPIO_WritePin(Row1_GPIO_Port, Row1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Row2_GPIO_Port, Row2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Row3_GPIO_Port, Row3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Row4_GPIO_Port, Row4_Pin, GPIO_PIN_RESET);
	if(!HAL_GPIO_ReadPin(Column1_GPIO_Port, Column1_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column1_GPIO_Port, Column1_Pin))
			KeyFlag = 13;
	}
	else if(!HAL_GPIO_ReadPin(Column2_GPIO_Port, Column2_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column2_GPIO_Port, Column2_Pin))
			KeyFlag = 14;
	}
	else if(!HAL_GPIO_ReadPin(Column3_GPIO_Port, Column3_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column3_GPIO_Port, Column3_Pin))
			KeyFlag = 15;
	}
	else if(!HAL_GPIO_ReadPin(Column4_GPIO_Port, Column4_Pin))
	{
		HAL_Delay(5);
		if(!HAL_GPIO_ReadPin(Column4_GPIO_Port, Column4_Pin))
			KeyFlag = 16;
	}
	
	switch(KeyFlag)
	{
		case 1: R1C1Fuction(); break;
		case 2: R1C2Fuction(); break;
		case 3: R1C3Fuction(); break;
		case 4: R1C4Fuction(); break;
		case 5: R2C1Fuction(); break;
		case 6: R2C2Fuction(); break;
		case 7: R2C3Fuction(); break;
		case 8: R2C4Fuction(); break;
		case 9: R3C1Fuction(); break;
		case 10: R3C2Fuction(); break;
		case 11: R3C3Fuction(); break;
		case 12: R3C4Fuction(); break;
		case 13: R4C1Fuction(); break;
		case 14: R4C2Fuction(); break;
		case 15: R4C3Fuction(); break;
		case 16: R4C4Fuction(); break;
		default: ElseFuction(); break;
	}
}

void R1C1Fuction(void)
{
//	LCDFlag = 1;
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R1C2Fuction(void)
{
//	LCDFlag = 0;
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
void R1C3Fuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
void R1C4Fuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
void R2C1Fuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R2C2Fuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R2C3Fuction(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R2C4Fuction(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R3C1Fuction(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R3C2Fuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R3C3Fuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R3C4Fuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R4C1Fuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R4C2Fuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R4C3Fuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void R4C4Fuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void ElseFuction(void)
{
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

#include "UserMain.h"

//参数
//****************************************************
int AllDataFlag;

//Debug
uint8_t buffer[33];
uint8_t length;


//红外传感参数
//float ObjectTemperature;  //物体温度
//float AmbientTemperature;  //环境温度
//float Emissivity;  //发射率


//温度LMT70参数
int FilterTimes = 10;
float ContactTemperature;  //接触温度


//串口2接收********************************
//心跳传感参数
uint8_t HeartCount;
uint8_t HeartBuffer[15];
uint8_t HeartReceiveNum;
int HeartReceiveFlag;

//计步
uint16_t StepCount;
uint8_t StepBuffer[5];
uint8_t StepReceiveNum;
int StepReceiveFlag;
//*********************************************


//两个陀螺仪***********************************
//串口4接收
//double PitchAcceleration1, RollAcceleration1, YawAcceleration1;
//uint8_t Gyro1Buffer[11];
//uint8_t Gyro1ReceiveNum;
//int Gyro1ReceiveFlag;
////串口6接收
//volatile double PitchAcceleration2, RollAcceleration2, YawAcceleration2;
//uint8_t Gyro2Buffer[11];
//uint8_t Gyro2CheckSum;
//uint8_t Gyro2ReceiveNum;


//发送至ESP32
uint8_t DataPacket[22];

//LCD参数***************************************
uint8_t StrInit[]= {"System initializing......."};
//**********************************************


//struct bmi160_sensor_data accel;
//int8_t rslt = BMI160_OK;

int fputc(int ch,FILE *f)
{
	
    uint8_t temp[1]={ch};
    HAL_UART_Transmit(&huart1,temp,1,10);
    return ch;
}

//int fgetc(FILE *f)
//{
//	uint8_t ch;
//	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET);
//	HAL_UART_Receive(&huart1, (uint8_t*)&ch, 2, 10);
//	return ch;
//}
//****************************************************

void UserInit(void)
{
//	ads1292_Init();
	
	Lcd_Init();
    Lcd_Clear(GRAY2);
	LcdAppendList(StrInit);
//*****************************************************	
	//两个陀螺仪
//	GyroUARTInit();
//	HAL_UART_Receive_IT(&huart4,&Gyro1Buffer[Gyro1ReceiveNum],1);
//	HAL_UART_Receive_IT(&huart6,&Gyro2Buffer[Gyro2ReceiveNum],1);
	
	//计步
	HAL_UART_Receive_IT(&huart1,&StepBuffer[StepReceiveNum],1);
	
	
	//心跳
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_IT(&huart2, &HeartBuffer[HeartReceiveNum], 1);
	
//	ADS1292_Init();
    /*
    下方函数 Set_ADS1292_Collect中参数表示对ADS1292模式设置
    其中0 正常采集，1 1mV1Hz内部测试信号 内部短接噪声测试  --------------------------------->目前已经弃用
    */
//    while(Set_ADS1292_Collect(0))
//    {
//        ADS1292_Init();
//        HAL_Delay(1000);
//    }


	//LMT70的ADC启动
//	HAL_ADC_Start(&hadc1);
	
	
	//红外传感
//	MLX90614_Init(&hi2c1);
//	MLX90614_SetEmissivity(0.985); //红外反射率，可修改
//*******************************************************
	HAL_Delay(100); //初始化判断
    Lcd_Clear(GRAY2);
}

void UserMain(void)
{
//	ADS1292_Genarate();
//	Debug();
//	HeartDataProcess();
//	StepDataProcess();
//	Gyro1DataProcess();
	DataPacketTransfer();
//	if(LCDFlag)
	UpdateLCD();
//	if(!LCDFlag)
//		Lcd_Clear(GRAY2);
//	
//	HAL_Delay(500);

////矩阵键盘和看门狗
//	char ch;
//	ch = getchar();
//	if(ch == 0xff)
//		printf("ok");
//	HAL_Delay(500);
//	MatrixKeyboardScanning();
//	if(HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin))
//		HAL_IWDG_Refresh(&hiwdg);
}

void Debug(void)
{
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "Emissivity: %.3f\r\n", Emissivity);
//	HAL_UART_Transmit(&huart1, buffer, length, 10);
//	HAL_Delay(1);
//	
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "AmbientTemp: %.3f\r\n", AmbientTemperature);
//	HAL_UART_Transmit(&huart1, buffer, length, 10);
//	HAL_Delay(1);
//	
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "ObjectTemp: %.3f\r\n", ObjectTemperature);
//	HAL_UART_Transmit(&huart1, buffer, length, 10);
//	HAL_Delay(1);
	
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "ContactTemp: %.3f\r\n", ContactTemperature);
//	HAL_UART_Transmit(&huart1, buffer, length, 10);
//	HAL_Delay(1);
//	
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "HeartRate: %d\r\n\r\n", HeartRate);
//	HAL_UART_Transmit(&huart1, buffer, length, 10);
//	HAL_Delay(1);

//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "PitchAcceleration1: %.3lf\r\n", PitchAcceleration1);
//	HAL_UART_Transmit(&huart1, buffer, length, 10);
//	HAL_Delay(1);
//	
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "RollAcceleration1: %.3lf\r\n", RollAcceleration1);
//	HAL_UART_Transmit(&huart1, buffer, length, 10);
//	HAL_Delay(1);
//	
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "YawAcceleration1: %.3lf\r\n\r\n", YawAcceleration1);
//	HAL_UART_Transmit(&huart1, buffer, length, 10);
//	HAL_Delay(1);

//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "PitchAcceleration2: %.3lf\r\n", PitchAcceleration2);
//	HAL_UART_Transmit(&huart1, buffer, length, 10);
//	HAL_Delay(1);
//	
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "RollAcceleration2: %.3lf\r\n", RollAcceleration2);
//	HAL_UART_Transmit(&huart1, buffer, length, 10);
//	HAL_Delay(1);
//	
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "YawAcceleration2: %.3lf\r\n\r\n", YawAcceleration2);
//	HAL_UART_Transmit(&huart1, buffer, length, 10);
//	HAL_Delay(1);
}

void UpdateLCD(void)
{
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "Emissivity: %.3f", Emissivity);
//	Gui_DrawFont_GBK16(5, 5, WHITE, GRAY2, buffer);
//	HAL_Delay(1);

//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "AmbientTemp: %.3f", AmbientTemperature);
//	Gui_DrawFont_GBK16(5, 25, WHITE, GRAY2, buffer);
//	HAL_Delay(1);

//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "ObjectTemp: %.3f", ObjectTemperature);
//	Gui_DrawFont_GBK16(5, 45, WHITE, GRAY2, buffer);
//	HAL_Delay(1);

	memset(buffer, 0x00, sizeof(buffer));
	length = sprintf((char*)buffer, "StepCount: %d   ", StepCount);
	Gui_DrawFont_GBK16(5, 5, WHITE, GRAY2, buffer);
//	HAL_Delay(1);
	
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "PitchAcceleration1: %.3lf   ", PitchAcceleration1);
//	Gui_DrawFont_GBK16(5, 25, WHITE, GRAY2, buffer);
//	HAL_Delay(1);
//	
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "RollAcceleration1: %.3lf   ", RollAcceleration1);
//	Gui_DrawFont_GBK16(5, 45, WHITE, GRAY2, buffer);
//	HAL_Delay(1);
//		
//	memset(buffer, 0x00, sizeof(buffer));
//	length = sprintf((char*)buffer, "YawAcceleration1: %.3lf   ", YawAcceleration1);
//	Gui_DrawFont_GBK16(5, 65, WHITE, GRAY2, buffer);
//	HAL_Delay(1);
	
	memset(buffer, 0x00, sizeof(buffer));
	length = sprintf((char*)buffer, "HeartCount: %d   ", HeartCount);
	Gui_DrawFont_GBK16(5, 25, WHITE, GRAY2, buffer);
//	HAL_Delay(1);
	
	memset(buffer, 0x00, sizeof(buffer));
	length = sprintf((char*)buffer, "ContactTemp: %.3f   ", ContactTemperature);
	Gui_DrawFont_GBK16(5, 45, WHITE, GRAY2, buffer);
//	HAL_Delay(1);
}

//void ADS1292DataTransfer(void)
//{
//	//DMA
//	uint8_t res,i,sum;	
//	uint8_t data_to_send[60];//串口发送缓存
//	uint8_t usbstatus=0;	
//	uint32_t cannle[2];	//存储两个通道的数据
//	int32_t	p_Temp[2];	//数据缓存
//	
//	data_to_send[0]=0xAA;
//	data_to_send[1]=0xAA;
//	data_to_send[2]=0xF1;	
//	data_to_send[3]=8;	
//		DMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)data_to_send);//串口1DMA设置
//		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //DMA		
//		
//		TIM2_Init(10000,7200);//系统指示
//		//TIM4_Init(2000,7200);//按键消抖
//		
//		EXTI->IMR |= EXTI_Line8;//开DRDY中断			
//		while(1)//循环发送数据		
//		{				
//				if(ads1292_recive_flag)
//				{										
//							cannle[0]=ads1292_Cache[3]<<16 | ads1292_Cache[4]<<8 | ads1292_Cache[5];//获取原始数据		
//							cannle[1]=ads1292_Cache[6]<<16 | ads1292_Cache[7]<<8 | ads1292_Cache[8];
//						
//							p_Temp[0] = get_volt(cannle[0]);	//把采到的3个字节转成有符号32位数
//							p_Temp[1] = get_volt(cannle[1]);	//把采到的3个字节转成有符号32位数
//					
//							//有符号数为再转为无符号，无符号数为逻辑右移
//							cannle[0] = p_Temp[0];
//							cannle[1]	= p_Temp[1];
//							data_to_send[4]=cannle[0]>>24;		//25-32位
//							data_to_send[5]=cannle[0]>>16;  	//17-24
//							data_to_send[6]=cannle[0]>>8;		//9-16
//							data_to_send[7]=cannle[0]; 			//1-8

//							data_to_send[8]=cannle[1]>>24;		//25-32位
//							data_to_send[9]=cannle[1]>>16;  	//17-24
//							data_to_send[10]=cannle[1]>>8;		//9-16
//							data_to_send[11]=cannle[1];			 //1-8
//							
//							for(i=0;i<12;i++)
//									sum += data_to_send[i];							
//							data_to_send[12] = sum;	//校验和																		
//							DMA_Enable(DMA1_Channel4,13);//串口1DMA 
//																							
//							ads1292_recive_flag=0;
//							sum = 0;	
//				}
//		}		
//}


////外部中断B8
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == GPIO_PIN_8)
//		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//}

////窗口看门狗中断
//void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
//{
//	HAL_WWDG_Refresh(hwwdg);
//	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//}

/*
外部中断回调函数
分支应用功能：
GPIO_Pin==DRDY_Pin：
ADS1292采集完毕触发中断
*/
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if(GPIO_Pin==DRDY_Pin && ADS_DRDY==0 && EXTIflag==1)
//    {
//        ADS1292_Read_Data((uint8_t *)ads1292_Cache);//把数据存到缓冲区
//        ads1292_recive_flag=1;
//        ads1292_recive_flag=1;
//    }
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int count = 0;
	//定时器2中断（主进程）
	if(htim->Instance == TIM2)  //10ms
	{
		count++;
		
		//红外传感函数
//		MLX90614_GetEmissivity(&Emissivity);
//		MLX90614_ReadAmbientTemperature(&AmbientTemperature);
//		MLX90614_ReadObjectTemperature(&ObjectTemperature);
		
		//温度LMT70函数
//		ContactTemperature = LMT70_ReadContactTemperature(FilterTimes);
		
//	HeartDataProcess();
//	StepDataProcess();
		
		//LED闪烁
		if(count >= 50)
		{
			static int TempCount = 0;
			if(TempCount == 5)
			{
				LMT70_GetContactTemperature();
				TempCount = 0;
			}
			else
				TempCount++;

			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			count = 0;
		}
		AllDataFlag = 1;
//		HAL_UART_Transmit(&huart1, DataPacket, 11, 10);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//串口1接收中断（步数值）
	if(huart->Instance == USART1)
    {
		if(StepReceiveNum == 0 && StepBuffer[0] != 0x01)
			StepReceiveNum = 0;
		else if(StepReceiveNum == 1 && StepBuffer[1] != 0x02)
			StepReceiveNum = 0;
		else
		{
			if(StepReceiveNum == 4)
			{
				int sum = 0;
				sum += StepBuffer[0];
				sum += StepBuffer[1];
				sum += StepBuffer[2];
				sum += StepBuffer[3];
				if(StepBuffer[4] == sum)
				{
					StepReceiveFlag = 1;
					StepDataProcess();
					StepReceiveNum = 0;;
				}
				else
					StepReceiveNum = 0;
			}
			else
				StepReceiveNum++;
		}
//		DataPacket[8] = StepCount;
		HAL_UART_Receive_IT(&huart1,&StepBuffer[StepReceiveNum],1);
	}	
	//串口2接收中断（心跳）
    if(huart->Instance == USART2)
    {
		//**********************心跳值
		if(HeartReceiveNum == 0 && HeartBuffer[0] != 0x0A)
			HeartReceiveNum = 0;
		else if(HeartReceiveNum == 1 && HeartBuffer[1] != 0xFA)
			HeartReceiveNum = 0;
		else
        {
			if(HeartReceiveNum == 13)
			{
				HeartReceiveFlag = 1;
				HeartDataProcess();
				HeartReceiveNum = 0;
			}
			else
				HeartReceiveNum++;
        }
		HAL_UART_Receive_IT(&huart2,&HeartBuffer[HeartReceiveNum],1);
		
		
		
//        if(HeartRateBuffer[0] != 0x01)
//        {
//            HeartCount = 0;
//        }
//		else if(HeartRateBuffer[1] != 0x01 && HeartCount == 1)
//		{
//			HeartCount = 0;
//		}
//        else
//        {
//            if(HeartCount == 3)
//            {	
//				for(int i=0; i<3; i++)
//					HeartCheckSum += HeartRateBuffer[i];	
//				if(HeartCheckSum != HeartRateBuffer[3] || HeartRateBuffer[1] == 0xff)
//					HeartCount = 0;
//				else
//				{
////					HAL_UART_Transmit(&huart1, &HeartRateBuffer[2], 1, 10);		
//					HeartRate = (int)HeartRateBuffer[2];
//					HeartCount = 0;					
//				}
//            }
//			else
//				HeartCount++;
//        }
//		HAL_UART_Receive_IT(&huart2,&HeartRateBuffer[HeartCount],1);		
    }
	//串口4接收中断（陀螺仪1）
//	else if(huart->Instance == UART4)
//	{
//		if(Gyro1ReceiveNum == 0 && Gyro1Buffer[0] != 0x55)
//			Gyro1ReceiveNum = 0;
//		else if(Gyro1ReceiveNum == 1 && Gyro1Buffer[1] != 0x51)
//			Gyro1ReceiveNum = 0;
//		else if(Gyro1ReceiveNum == 10)
//        {
//			uint8_t sum = 0;
//			sum += Gyro1Buffer[0];
//			sum += Gyro1Buffer[1];
//			sum += Gyro1Buffer[2];
//			sum += Gyro1Buffer[3];
//			sum += Gyro1Buffer[4];
//			sum += Gyro1Buffer[5];
//			sum += Gyro1Buffer[6];
//			sum += Gyro1Buffer[7];
//			sum += Gyro1Buffer[8];
//			sum += Gyro1Buffer[9];
//			if(Gyro1Buffer[10] != sum)
//			{
////				PitchAcceleration1 = -1;
////				RollAcceleration1 = -1;
////				YawAcceleration1 = -1;
//				Gyro1ReceiveNum = 0;
//			}
//			else
//			{
//				Gyro1ReceiveFlag = 1;
//				Gyro1ReceiveNum = 0;
//			}
//        }
//		else
//			Gyro1ReceiveNum++;
//		HAL_UART_Receive_IT(&huart4,&Gyro1Buffer[Gyro1ReceiveNum],1);
//	}
//	//串口6接收中断（陀螺仪2）
//	else if(huart->Instance == USART6)
//	{
//		if(Gyro2ReceiveNum == 0 && Gyro2Buffer[0] != 0x55)
//			Gyro2ReceiveNum = 0;
//		else if(Gyro2ReceiveNum == 1 && Gyro2Buffer[1] != 0x51)
//			Gyro2ReceiveNum = 0;
//		else if(Gyro2ReceiveNum == 10)
//        {
//			uint8_t sum = 0;
//			sum += Gyro2Buffer[0];
//			sum += Gyro2Buffer[1];
//			sum += Gyro2Buffer[2];
//			sum += Gyro2Buffer[3];
//			sum += Gyro2Buffer[4];
//			sum += Gyro2Buffer[5];
//			sum += Gyro2Buffer[6];
//			sum += Gyro2Buffer[7];
//			sum += Gyro2Buffer[8];
//			sum += Gyro2Buffer[9];
//			if(Gyro2Buffer[10] != sum)
//				Gyro2ReceiveNum = 0;
//			else
//			{
//				short temp;
//				temp = (Gyro2Buffer[3] << 8) | Gyro2Buffer[2];
//				PitchAcceleration2 = (double)temp / (double)32768 * (double)16 * (double)9.8; 

//				temp = (Gyro2Buffer[5] << 8) | Gyro2Buffer[4];
//				RollAcceleration2 = (double)temp / (double)32768 * (double)16 * (double)9.8; 

//				temp = (Gyro2Buffer[7] << 8) | Gyro2Buffer[6];
//				YawAcceleration2 = (double)temp / (double)32768 * (double)16 * (double)9.8;
//				
//				Gyro2ReceiveNum = 0;
//			}
//        }
//		else
//			Gyro2ReceiveNum++;
//		HAL_UART_Receive_IT(&huart6,&Gyro2Buffer[Gyro2ReceiveNum],1);
//	}
}


//int8_t user_i2c_read1(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
//{
//	HAL_I2C_Mem_Read(&hi2c2, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 10);
//	return (int8_t)data;
//}

//int8_t user_i2c_write1(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
//{
//	HAL_I2C_Mem_Write(&hi2c2, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 10);
//	return (int8_t)data;
//}

//void user_delay1(uint32_t period)
//{
//	HAL_Delay(period);
//}

//void test()
//{
//	struct bmi160_dev sensor;
//	
//	bmi160_com_fptr_t p, q;
//	bmi160_delay_fptr_t r;
//	p = user_i2c_read1;
//	q = user_i2c_write1;
//	r = user_delay1;
//	sensor.id = BMI160_I2C_ADDR;
//	sensor.interface = BMI160_I2C_INTF;
//	sensor.read = p;
//	sensor.write = q;
//	sensor.delay_ms = r;

//	rslt = bmi160_init(&sensor);


//	/* Select the Output data rate, range of accelerometer sensor */
//	sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
//	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
//	sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

//	/* Select the power mode of accelerometer sensor */
//	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

//	/* Select the Output data rate, range of Gyroscope sensor */
//	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
//	sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
//	sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

//	/* Select the power mode of Gyroscope sensor */
//	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

//	/* Set the sensor configuration */
//	rslt = bmi160_set_sens_conf(&sensor);
//	

//	/* To read only Accel data */
//	rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &sensor);
//}



//外部中断回调函数
//分支应用功能：
//GPIO_Pin==DRDY_Pin：
//ADS1292采集完毕触发中断
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if(GPIO_Pin==ADS1292_DRDY_Pin && ADS1292_DRDY_Pin == GPIO_PIN_RESET)
//    {
//        ADS1292_Genarate();
//    }
//}

void HeartDataProcess(void)
{
	if(HeartReceiveFlag == 1)
	{
		DataPacket[13] = HeartBuffer[7];
		DataPacket[14] = HeartBuffer[8];
		DataPacket[17] = HeartBuffer[11];
		HeartCount = HeartBuffer[11];
//			for(int i=0; i<14; i++)
//	{
//		HAL_UART_Transmit(&huart6, &HeartBuffer[i], 1, 10); 
//	}
		HeartReceiveFlag = 0;
	}
}

void StepDataProcess(void)
{
	if(StepReceiveFlag == 1)
	{
		DataPacket[1] = StepBuffer[2];
		DataPacket[2] = StepBuffer[3];
		StepCount = StepBuffer[2] + StepBuffer[3] * 256;
		StepReceiveFlag = 0;
	}
}

//void Gyro1DataProcess(void)
//{
//		if(Gyro1ReceiveFlag == 1)
//	{
//		DataPacket[7] = Gyro1Buffer[2];
//		DataPacket[8] = Gyro1Buffer[3];
//		DataPacket[9] = Gyro1Buffer[4];
//		DataPacket[10] = Gyro1Buffer[5];
//		DataPacket[11] = Gyro1Buffer[6];
//		DataPacket[12] = Gyro1Buffer[7];
//		
//		short temp;
//		temp = (Gyro1Buffer[3] << 8) | Gyro1Buffer[2];
//		PitchAcceleration1 = (double)temp / (double)32768 * (double)16 * (double)9.8; 

//		temp = (Gyro1Buffer[5] << 8) | Gyro1Buffer[4];
//		RollAcceleration1 = (double)temp / (double)32768 * (double)16 * (double)9.8; 

//		temp = (Gyro1Buffer[7] << 8) | Gyro1Buffer[6];
//		YawAcceleration1 = (double)temp / (double)32768 * (double)16 * (double)9.8;
//		Gyro1ReceiveFlag = 0;
//	}
//}

void DataPacketTransfer(void)
{
//	if(AllDataFlag==1)
//	{
		static char count = 0;
		int i;
		DataPacket[0] = count;
		HAL_GPIO_WritePin(Packet_CLK_GPIO_Port, Packet_CLK_Pin, GPIO_PIN_RESET);
		for(i=0; i<20; i++)
		{
			HAL_UART_Transmit(&huart1, &DataPacket[i], 1, 10); 
		}
		HAL_Delay(2);
		HAL_GPIO_WritePin(Packet_CLK_GPIO_Port, Packet_CLK_Pin, GPIO_PIN_SET);
		if(count != 255)
			count++;
		else
			 count = 0;
//		AllDataFlag = 0;
//	}
}
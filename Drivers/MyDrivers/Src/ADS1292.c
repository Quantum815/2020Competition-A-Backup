//////////////////////////////////////////////////////////////////////////////////////////
//
//   Arduino Library for ADS1292R Shield/Breakout
//
//   Copyright (c) 2017 ProtoCentral
//   
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT). 
//   
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   Requires g4p_control graphing library for processing.  Built on V4.1
//   Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "ADS1292.h"

char* ads1292_Read_Data()
{

   static char SPI_Dummy_Buff[10];
   
	HAL_GPIO_WritePin(ADS1292_CS_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_RESET);
   
	for (int i = 0; i < 9; ++i)
	{
		HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)CONFIG_SPI_MASTER_DUMMY, (uint8_t*)&SPI_Dummy_Buff[i], 1, 10);
	}
	
	HAL_GPIO_WritePin(ADS1292_CS_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_SET);
	
	return SPI_Dummy_Buff;
}

void ads1292_Init()
{ 
//  // start the SPI library:
//  SPI.begin();
//  SPI.setBitOrder(MSBFIRST); 
//  //CPOL = 0, CPHA = 1
//  SPI.setDataMode(SPI_MODE1);
//  // Selecting 1Mhz clock for SPI
//  SPI.setClockDivider(SPI_CLOCK_DIV16);

  ads1292_Reset();
  HAL_Delay(100);
  ads1292_Disable_Start();
  ads1292_Enable_Start();
  
  ads1292_Hard_Stop();
  ads1292_Start_Data_Conv_Command();
  ads1292_Soft_Stop();
  HAL_Delay(50);
  ads1292_Stop_Read_Data_Continuous();					// SDATAC command
  HAL_Delay(300);
  
  ads1292_Reg_Write(ADS1292_REG_CONFIG1, 0x00); 		//Set sampling rate to 125 SPS
  HAL_Delay(10);
  ads1292_Reg_Write(ADS1292_REG_CONFIG2, 0b10100000);	//Lead-off comp off, test signal disabled
  HAL_Delay(10);
  ads1292_Reg_Write(ADS1292_REG_LOFF, 0b00010000);		//Lead-off defaults
  HAL_Delay(10);
  ads1292_Reg_Write(ADS1292_REG_CH1SET, 0b01000000);	//Ch 1 enabled, gain 6, connected to electrode in
  HAL_Delay(10);
  ads1292_Reg_Write(ADS1292_REG_CH2SET, 0b01100000);	//Ch 2 enabled, gain 6, connected to electrode in
  HAL_Delay(10);
  ads1292_Reg_Write(ADS1292_REG_RLDSENS, 0b00101100);	//RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
  HAL_Delay(10);
  ads1292_Reg_Write(ADS1292_REG_LOFFSENS, 0x00);		//LOFF settings: all disabled
  HAL_Delay(10);
														//Skip register 8, LOFF Settings default
  ads1292_Reg_Write(ADS1292_REG_RESP1, 0b11110010);		//Respiration: MOD/DEMOD turned only, phase 0
  HAL_Delay(10);
  ads1292_Reg_Write(ADS1292_REG_RESP2, 0b00000011);		//Respiration: Calib OFF, respiration freq defaults
  HAL_Delay(10);
  ads1292_Start_Read_Data_Continuous();
  HAL_Delay(10);
  ads1292_Enable_Start();
}

void ads1292_Reset()
{
	HAL_GPIO_WritePin(ADS1292_PWDN_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(100);					// Wait 100 mSec
	HAL_GPIO_WritePin(ADS1292_PWDN_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
	HAL_GPIO_WritePin(ADS1292_PWDN_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
}

void ads1292_Disable_Start()
{
	HAL_GPIO_WritePin(ADS1292_START_GPIO_Port, ADS1292_START_Pin, GPIO_PIN_RESET);
  HAL_Delay(20);
}

void ads1292_Enable_Start()
{
	HAL_GPIO_WritePin(ADS1292_START_GPIO_Port, ADS1292_START_Pin, GPIO_PIN_SET);
  HAL_Delay(20);
}

void ads1292_Hard_Stop (void)
{
	HAL_GPIO_WritePin(ADS1292_START_GPIO_Port, ADS1292_START_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
}


void ads1292_Start_Data_Conv_Command (void)
{
  ads1292_SPI_Command_Data(START);					// Send 0x08 to the ADS1x9x
}

void ads1292_Soft_Stop (void)
{
  ads1292_SPI_Command_Data(STOP);                   // Send 0x0A to the ADS1x9x
}

void ads1292_Start_Read_Data_Continuous (void)
{
  ads1292_SPI_Command_Data(RDATAC);					// Send 0x10 to the ADS1x9x
}

void ads1292_Stop_Read_Data_Continuous (void)
{
  ads1292_SPI_Command_Data(SDATAC);					// Send 0x11 to the ADS1x9x
}

void ads1292_SPI_Command_Data(unsigned char data_in)
{
//   byte data[1];
  //data[0] = data_in;
	HAL_GPIO_WritePin(ADS1292_CS_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);
	HAL_GPIO_WritePin(ADS1292_CS_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
	HAL_GPIO_WritePin(ADS1292_CS_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);
	HAL_SPI_Transmit(&hspi2, &data_in, 1, 10);
  HAL_Delay(2);
	HAL_GPIO_WritePin(ADS1292_CS_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_SET);
}

//Sends a write command to SCP1000
void ads1292_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
{
  switch (READ_WRITE_ADDRESS)
  {
    case 1:
            DATA = DATA & 0x87;
	    break;
    case 2:
            DATA = DATA & 0xFB;
	    DATA |= 0x80;		
	    break;
    case 3:
	    DATA = DATA & 0xFD;
	    DATA |= 0x10;
	    break;
    case 7:
	    DATA = DATA & 0x3F;
	    break;
    case 8:
    	    DATA = DATA & 0x5F;
	    break;
    case 9:
	    DATA |= 0x02;
	    break;
    case 10:
	    DATA = DATA & 0x87;
	    DATA |= 0x01;
	    break;
    case 11:
	    DATA = DATA & 0x0F;
	    break;
    default:
	    break;		
  }
  
  // now combine the register address and the command into one byte:
  unsigned char dataToSend = READ_WRITE_ADDRESS | WREG;
  	HAL_GPIO_WritePin(ADS1292_CS_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_RESET);
   HAL_Delay(2);
  	HAL_GPIO_WritePin(ADS1292_CS_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_SET);
   HAL_Delay(2);
  // take the chip select low to select the device:
  	HAL_GPIO_WritePin(ADS1292_CS_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);
  	HAL_SPI_Transmit(&hspi2, &dataToSend, 1, 10);//Send register location
  	HAL_SPI_Transmit(&hspi2, 0x00, 1, 10);//number of register to wr
  	HAL_SPI_Transmit(&hspi2, &DATA, 1, 10);//Send value to record into register
  
  HAL_Delay(2);
  // take the chip select high to de-select:
  	HAL_GPIO_WritePin(ADS1292_CS_GPIO_Port, ADS1292_CS_Pin, GPIO_PIN_SET);
}




//////////////////////////////////////////////////////////////////////////////////////////
//
//   Arduino Library for ADS1292R Shield/Breakout
//
//   Copyright (c) 2017 ProtoCentral
//   Heartrate and respiration computation based on original code from Texas Instruments
//
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   Requires g4p_control graphing library for processing.  Built on V4.1
//   Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install

// If you have bought the breakout the connection with the Arduino board is as follows:
//
//|ads1292r pin label| Arduino Connection   |Pin Function      |
//|----------------- |:--------------------:|-----------------:|
//| VDD              | +5V                  |  Supply voltage  |
//| PWDN/RESET       | D4                   |  Reset           |
//| START            | D5                   |  Start Input     |
//| DRDY             | D6                   |  Data Ready Outpt|
//| CS               | D7                   |  Chip Select     |
//| MOSI             | D11                  |  Slave In        |
//| MISO             | D12                  |  Slave Out       |
//| SCK              | D13                  |  Serial Clock    |
//| GND              | Gnd                  |  Gnd             |
//
/////////////////////////////////////////////////////////////////////////////////////////


//void setup()
//{
//  delay(2000);
//  // initalize the  data ready and chip select pins:
//  pinMode(ADS1292_DRDY_PIN, INPUT);  //6
//  pinMode(ADS1292_CS_PIN, OUTPUT);    //7
//  pinMode(ADS1292_START_PIN, OUTPUT);  //5
//  pinMode(ADS1292_PWDN_PIN, OUTPUT);  //4

//  Serial.begin(115200);  // Baudrate for serial communica

//  ADS1292.ads1292_Init();  //initalize ADS1292 slave
//}

volatile uint8_t  SPI_Dummy_Buff[30];
uint8_t DataPacketHeader[16];
volatile signed long s32DaqVals[8];
uint8_t data_len = 7;
volatile char SPI_RX_Buff[15] ;
volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile HAL_StatusTypeDef ads1292dataReceived = HAL_ERROR;
unsigned long uecgtemp = 0;
signed long secgtemp = 0;
int i, j;

void ECG_ProcessCurrSample(int16_t *CurrAqsSample, int16_t *FilteredOut);
void QRS_Algorithm_Interface(int16_t CurrSample);
static void QRS_process_buffer( void );
static void QRS_check_sample_crossing_threshold( uint16_t scaled_result );


//************** ecg *******************
int16_t CoeffBuf_40Hz_LowPass[FILTERORDER] =
{
  -72,    122,    -31,    -99,    117,      0,   -121,    105,     34,
  -137,     84,     70,   -146,     55,    104,   -147,     20,    135,
  -137,    -21,    160,   -117,    -64,    177,    -87,   -108,    185,
  -48,   -151,    181,      0,   -188,    164,     54,   -218,    134,
  112,   -238,     90,    171,   -244,     33,    229,   -235,    -36,
  280,   -208,   -115,    322,   -161,   -203,    350,    -92,   -296,
  361,      0,   -391,    348,    117,   -486,    305,    264,   -577,
  225,    445,   -660,     93,    676,   -733,   -119,    991,   -793,
  -480,   1486,   -837,  -1226,   2561,   -865,  -4018,   9438,  20972,
  9438,  -4018,   -865,   2561,  -1226,   -837,   1486,   -480,   -793,
  991,   -119,   -733,    676,     93,   -660,    445,    225,   -577,
  264,    305,   -486,    117,    348,   -391,      0,    361,   -296,
  -92,    350,   -203,   -161,    322,   -115,   -208,    280,    -36,
  -235,    229,     33,   -244,    171,     90,   -238,    112,    134,
  -218,     54,    164,   -188,      0,    181,   -151,    -48,    185,
  -108,    -87,    177,    -64,   -117,    160,    -21,   -137,    135,
  20,   -147,    104,     55,   -146,     70,     84,   -137,     34,
  105,   -121,      0,    117,    -99,    -31,    122,    -72
};
int16_t ECG_WorkingBuff[2 * FILTERORDER];
unsigned char Start_Sample_Count_Flag = 0;
unsigned char first_peak_detect = FALSE ;
unsigned int sample_index[MAX_PEAK_TO_SEARCH + 2] ;
uint16_t scaled_result_display[150];
uint8_t indx = 0;

int cnt = 0;
volatile uint8_t flag = 0;

int QRS_Second_Prev_Sample = 0 ;
int QRS_Prev_Sample = 0 ;
int QRS_Current_Sample = 0 ;
int QRS_Next_Sample = 0 ;
int QRS_Second_Next_Sample = 0 ;

static uint16_t QRS_B4_Buffer_ptr = 0 ;
/*   Variable which holds the threshold value to calculate the maxima */
int16_t QRS_Threshold_Old = 0;
int16_t QRS_Threshold_New = 0;
unsigned int sample_count = 0 ;

/* Variable which will hold the calculated heart rate */
volatile uint16_t QRS_Heart_Rate = 0 ;
int16_t ecg_wave_buff[1], ecg_filterout[1];

volatile uint8_t global_HeartRate = 0;
volatile uint8_t global_RespirationRate = 0;
long status_byte=0;
uint8_t LeadStatus=0;
// boolean leadoff_deteted = true;
HAL_StatusTypeDef leadoff_deteted = HAL_OK;

void ADS1292_Genarate()
{
//  if (HAL_GPIO_ReadPin(ADS1292_DRDY_GPIO_Port, ADS1292_DRDY_Pin) == GPIO_PIN_RESET)      // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
//  {
    SPI_RX_Buff_Ptr = ads1292_Read_Data(); // Read the data,point the data to a pointer

    for (i = 0; i < 9; i++)
    {
      SPI_RX_Buff[SPI_RX_Buff_Count++] = *(SPI_RX_Buff_Ptr + i);  // store the result data in array
    }
    ads1292dataReceived = HAL_OK;
//  }


  if (ads1292dataReceived == HAL_OK)      // process the data
  {
    j = 0;
    for (i = 3; i < 9; i += 3)         // data outputs is (24 status bits + 24 bits Respiration data +  24 bits ECG data)
    {

      uecgtemp = (unsigned long) (  ((unsigned long)SPI_RX_Buff[i + 0] << 16) | ( (unsigned long) SPI_RX_Buff[i + 1] << 8) |  (unsigned long) SPI_RX_Buff[i + 2]);
      uecgtemp = (unsigned long) (uecgtemp << 8);
      secgtemp = (signed long) (uecgtemp);
      secgtemp = (signed long) (secgtemp >> 8);

      s32DaqVals[j++] = secgtemp;  //s32DaqVals[0] is Resp data and s32DaqVals[1] is ECG data
    }

    status_byte = (long)((long)SPI_RX_Buff[2] | ((long) SPI_RX_Buff[1]) <<8 | ((long) SPI_RX_Buff[0])<<16); // First 3 bytes represents the status
    status_byte  = (status_byte & 0x0f8000) >> 15;  // bit15 gives the lead status
    LeadStatus = (unsigned char ) status_byte ;  

    if(!((LeadStatus & 0x1f) == 0 ))
      leadoff_deteted  = HAL_OK; 
    else
      leadoff_deteted  = HAL_ERROR;
    
    ecg_wave_buff[0] = (int16_t)(s32DaqVals[1] >> 8) ;  // ignore the lower 8 bits out of 24bits

    if(leadoff_deteted == HAL_ERROR) 
       {
          ECG_ProcessCurrSample(&ecg_wave_buff[0], &ecg_filterout[0]);   // filter out the line noise @40Hz cutoff 161 order
          QRS_Algorithm_Interface(ecg_filterout[0]);             // calculate 
       }
    else
       ecg_filterout[0] = 0;

    DataPacketHeader[0] = CES_CMDIF_PKT_START_1 ;   // Packet header1 :0x0A
    DataPacketHeader[1] = CES_CMDIF_PKT_START_2;    // Packet header2 :0xFA
    DataPacketHeader[2] = (uint8_t) (data_len);     // data length
    DataPacketHeader[3] = (uint8_t) (data_len >> 8);
    DataPacketHeader[4] = CES_CMDIF_TYPE_DATA;      // packet type: 0x02 -data 0x01 -commmand

    DataPacketHeader[5] = ecg_filterout[0];
    DataPacketHeader[6] = ecg_filterout[0] >> 8;
    
    DataPacketHeader[7] = s32DaqVals[0];            // 4 bytes ECG data
    DataPacketHeader[8] = s32DaqVals[0] >> 8;
    DataPacketHeader[9] = s32DaqVals[0] >> 16;
    DataPacketHeader[10] = s32DaqVals[0] >> 24;

    if(leadoff_deteted == HAL_OK) // lead in not connected
      DataPacketHeader[11] = 0 ; 
    else
      DataPacketHeader[11] = global_HeartRate ; 

    DataPacketHeader[12] = CES_CMDIF_PKT_STOP_1;   // Packet footer1:0x00
    DataPacketHeader[13] = CES_CMDIF_PKT_STOP_2 ;   // Packet footer2:0x0B

	HAL_UART_Transmit(&huart1, DataPacketHeader, 14, 10);
//    for (i = 0; i < 14; i++)
//    {
//      Serial.write(DataPacketHeader[i]);     // transmit the data over USB
//    }
  }

  ads1292dataReceived = HAL_ERROR;
  SPI_RX_Buff_Count = 0;

}





void ECG_FilterProcess(int16_t * WorkingBuff, int16_t * CoeffBuf, int16_t* FilterOut)
{

  int32_t acc = 0;   // accumulator for MACs
  int  k;

  // perform the multiply-accumulate
  for ( k = 0; k < 161; k++ )
  {
    acc += (int32_t)(*CoeffBuf++) * (int32_t)(*WorkingBuff--);
  }
  // saturate the result
  if ( acc > 0x3fffffff )
  {
    acc = 0x3fffffff;
  } else if ( acc < -0x40000000 )
  {
    acc = -0x40000000;
  }
  // convert from Q30 to Q15
  *FilterOut = (int16_t)(acc >> 15);
  //*FilterOut = *WorkingBuff;
}




void ECG_ProcessCurrSample(int16_t *CurrAqsSample, int16_t *FilteredOut)
{
  static uint16_t ECG_bufStart = 0, ECG_bufCur = FILTERORDER - 1, ECGFirstFlag = 1;
  static int16_t ECG_Pvev_DC_Sample, ECG_Pvev_Sample;/* Working Buffer Used for Filtering*/
  //  static short ECG_WorkingBuff[2 * FILTERORDER];
  int16_t *CoeffBuf;
  int16_t temp1, temp2, ECGData;

  /* Count variable*/
  uint16_t Cur_Chan;
  int16_t FiltOut = 0;
  //  short FilterOut[2];
  CoeffBuf = CoeffBuf_40Hz_LowPass;         // Default filter option is 40Hz LowPass

  if  ( ECGFirstFlag )                // First Time initialize static variables.
  {
    for ( Cur_Chan = 0 ; Cur_Chan < FILTERORDER; Cur_Chan++)
    {
      ECG_WorkingBuff[Cur_Chan] = 0;
    }
    ECG_Pvev_DC_Sample = 0;
    ECG_Pvev_Sample = 0;
    ECGFirstFlag = 0;
  }

  temp1 = NRCOEFF * ECG_Pvev_DC_Sample;       //First order IIR
  ECG_Pvev_DC_Sample = (CurrAqsSample[0]  - ECG_Pvev_Sample) + temp1;
  ECG_Pvev_Sample = CurrAqsSample[0];
  temp2 = ECG_Pvev_DC_Sample >> 2;
  ECGData = (int16_t) temp2;

  /* Store the DC removed value in Working buffer in millivolts range*/
  ECG_WorkingBuff[ECG_bufCur] = ECGData;
  ECG_FilterProcess(&ECG_WorkingBuff[ECG_bufCur], CoeffBuf, (int16_t*)&FiltOut);
  /* Store the DC removed value in ECG_WorkingBuff buffer in millivolts range*/
  ECG_WorkingBuff[ECG_bufStart] = ECGData;

  /* Store the filtered out sample to the LeadInfo buffer*/
  FilteredOut[0] = FiltOut ;//(CurrOut);

  ECG_bufCur++;
  ECG_bufStart++;

  if ( ECG_bufStart  == (FILTERORDER - 1))
  {
    ECG_bufStart = 0;
    ECG_bufCur = FILTERORDER - 1;
  }
  return ;
}


void QRS_Algorithm_Interface(int16_t CurrSample)
{
  //  static FILE *fp = fopen("ecgData.txt", "w");
  static int16_t prev_data[32] = {0};
  int16_t i;
  long Mac = 0;
  prev_data[0] = CurrSample;

  for ( i = 31; i > 0; i--)
  {
    Mac += prev_data[i];
    prev_data[i] = prev_data[i - 1];

  }
  Mac += CurrSample;
  Mac = Mac >> 2;
  CurrSample = (int16_t) Mac;
  QRS_Second_Prev_Sample = QRS_Prev_Sample ;
  QRS_Prev_Sample = QRS_Current_Sample ;
  QRS_Current_Sample = QRS_Next_Sample ;
  QRS_Next_Sample = QRS_Second_Next_Sample ;
  QRS_Second_Next_Sample = CurrSample ;



  QRS_process_buffer();
}

static void QRS_process_buffer( void )
{

  int16_t first_derivative = 0 ;
  int16_t scaled_result = 0 ;

  static int16_t Max = 0 ;

  /* calculating first derivative*/
  first_derivative = QRS_Next_Sample - QRS_Prev_Sample  ;



  /*taking the absolute value*/

  if (first_derivative < 0)
  {
    first_derivative = -(first_derivative);
  }


  scaled_result = first_derivative;

  if ( scaled_result > Max )
  {
    Max = scaled_result ;
  }


  QRS_B4_Buffer_ptr++;
  if (QRS_B4_Buffer_ptr ==  TWO_SEC_SAMPLES)
  {
    QRS_Threshold_Old = ((Max * 7) / 10 ) ;
    QRS_Threshold_New = QRS_Threshold_Old ;
   // if (Max > 70)
      first_peak_detect = TRUE ;
    Max = 0;
 
    //  ecg_wave_buff[0] = first_derivative;
    QRS_B4_Buffer_ptr = 0;


  }


  if ( TRUE == first_peak_detect )
  {
    QRS_check_sample_crossing_threshold( scaled_result ) ;
  }


}


static void QRS_check_sample_crossing_threshold( uint16_t scaled_result )
{
  /* array to hold the sample indexes S1,S2,S3 etc */

  static uint16_t s_array_index = 0 ;
  static uint16_t m_array_index = 0 ;

  static unsigned char threshold_crossed = FALSE ;
  static uint16_t maxima_search = 0 ;
  static unsigned char peak_detected = FALSE ;
  static uint16_t skip_window = 0 ;
  static long maxima_sum = 0 ;
  static unsigned int peak = 0;
  static unsigned int sample_sum = 0;
  static unsigned int nopeak = 0;
  uint16_t Max = 0 ;
  uint16_t HRAvg;
  // uint16_t  RRinterval = 0;

  if ( TRUE == threshold_crossed  )
  {
    /*
    Once the sample value crosses the threshold check for the
    maxima value till MAXIMA_SEARCH_WINDOW samples are received
    */
    sample_count ++ ;
    maxima_search ++ ;

    if ( scaled_result > peak )
    {
      peak = scaled_result ;
    }

    if ( maxima_search >= MAXIMA_SEARCH_WINDOW )
    {
      // Store the maxima values for each peak
      maxima_sum += peak ;
      maxima_search = 0 ;


      threshold_crossed = FALSE ;
      peak_detected = TRUE ;
    }

  }
  else if ( TRUE == peak_detected )
  {
    /*
    Once the sample value goes below the threshold
    skip the samples untill the SKIP WINDOW criteria is meet
    */
    sample_count ++ ;
    skip_window ++ ;

    if ( skip_window >= MINIMUM_SKIP_WINDOW )
    {
      skip_window = 0 ;
      peak_detected = FALSE ;
    }

    if ( m_array_index == MAX_PEAK_TO_SEARCH )
    {
      sample_sum = sample_sum / (MAX_PEAK_TO_SEARCH - 1);
      HRAvg =  (uint16_t) sample_sum  ;

      // Compute HR without checking LeadOffStatus
      QRS_Heart_Rate = (uint16_t) 60 *  SAMPLING_RATE;
      QRS_Heart_Rate =  QRS_Heart_Rate / HRAvg ;
      if (QRS_Heart_Rate > 250)
        QRS_Heart_Rate = 250 ;
 //     Serial.println(QRS_Heart_Rate);

      /* Setting the Current HR value in the ECG_Info structure*/
      maxima_sum =  maxima_sum / MAX_PEAK_TO_SEARCH;
      Max = (int16_t) maxima_sum ;
      /*  calculating the new QRS_Threshold based on the maxima obtained in 4 peaks */
      maxima_sum = Max * 7;
      maxima_sum = maxima_sum / 10;
      QRS_Threshold_New = (int16_t)maxima_sum;

      /* Limiting the QRS Threshold to be in the permissible range*/
      if (QRS_Threshold_New > (4 * QRS_Threshold_Old))
      {
        QRS_Threshold_New = QRS_Threshold_Old;
      }

      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;

      sample_sum = 0;
    }
  }
  else if ( scaled_result > QRS_Threshold_New )
  {
    /*
      If the sample value crosses the threshold then store the sample index
    */
    Start_Sample_Count_Flag = 1;
    sample_count ++ ;
    m_array_index++;
    threshold_crossed = TRUE ;
    peak = scaled_result ;
    nopeak = 0;

    /*  storing sample index*/
    sample_index[ s_array_index ] = sample_count ;
    if ( s_array_index >= 1 )
    {
      sample_sum += sample_index[ s_array_index ] - sample_index[ s_array_index - 1 ] ;
    }
    s_array_index ++ ;
  }

  else if (( scaled_result < QRS_Threshold_New ) && (Start_Sample_Count_Flag == 1))
  {
    sample_count ++ ;
    nopeak++;
    if (nopeak > (3 * SAMPLING_RATE))
    {
      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;
      peak_detected = FALSE ;
      sample_sum = 0;

      first_peak_detect = FALSE;
      nopeak = 0;

      QRS_Heart_Rate = 0;

    }
  }
  else
  {
    nopeak++;
    if (nopeak > (3 * SAMPLING_RATE))
    {
      /* Reset heart rate computation sate variable in case of no peak found in 3 seconds */
      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;
      peak_detected = FALSE ;
      sample_sum = 0;
      first_peak_detect = FALSE;
      nopeak = 0;
      QRS_Heart_Rate = 0;
    }
  }

  global_HeartRate = (uint8_t)QRS_Heart_Rate;
  //Serial.println(global_HeartRate);

}
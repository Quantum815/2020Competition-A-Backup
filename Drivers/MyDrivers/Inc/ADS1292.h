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


#ifndef ADS1292_H_
#define ADS1292_H_

#include "main.h"

#define CONFIG_SPI_MASTER_DUMMY   0xFF

// Register Read Commands
#define  RREG    0x20;		//Read n nnnn registers starting at address r rrrr
                                //first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG    0x40;		//Write n nnnn registers starting at address r rrrr
                                //first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)

#define START		0x08		//Start/restart (synchronize) conversions
#define STOP		0x0A		//Stop conversion
#define RDATAC      0x10		//Enable Read Data Continuous mode. 

//This mode is the default mode at power-up.
#define SDATAC		0x11		//Stop Read Data Continuously mode
#define RDATA		0x12		//Read data by command; supports multiple read back.

//register address
#define ADS1292_REG_ID			0x00
#define ADS1292_REG_CONFIG1		0x01
#define ADS1292_REG_CONFIG2		0x02
#define ADS1292_REG_LOFF		0x03
#define ADS1292_REG_CH1SET		0x04
#define ADS1292_REG_CH2SET		0x05
#define ADS1292_REG_RLDSENS		0x06
#define ADS1292_REG_LOFFSENS    0x07
#define ADS1292_REG_LOFFSTAT    0x08
#define ADS1292_REG_RESP1	    0x09
#define ADS1292_REG_RESP2	    0x0A

extern SPI_HandleTypeDef hspi2;

void ads1292_Init(void);
void ads1292_Reset(void);
void ads1292_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA);
void ads1292_Reg_Read (unsigned char READ_WRITE_ADDRESS);
void ads1292_SPI_Command_Data(unsigned char data_in);
void ads1292_Disable_Start(void);
void ads1292_Enable_Start(void);
void ads1292_Hard_Stop (void);
void ads1292_Start_Data_Conv_Command (void);
void ads1292_Soft_Stop (void);
void ads1292_Start_Read_Data_Continuous (void);
void ads1292_Stop_Read_Data_Continuous (void);
char* ads1292_Read_Data(void);



//*****************************************************************************************
//Arduino Main.h

//Packet format
#define  CES_CMDIF_PKT_START_1      0x0A
#define  CES_CMDIF_PKT_START_2      0xFA
#define  CES_CMDIF_TYPE_DATA        0x02
#define  CES_CMDIF_PKT_STOP_1       0x00
#define  CES_CMDIF_PKT_STOP_2       0x0B

#define TEMPERATURE 0
#define FILTERORDER         161
/* DC Removal Numerator Coeff*/
#define NRCOEFF (0.992)
#define WAVE_SIZE  1

//******* ecg filter *********
#define MAX_PEAK_TO_SEARCH         5
#define MAXIMA_SEARCH_WINDOW      25
#define MINIMUM_SKIP_WINDOW       30
#define SAMPLING_RATE             125
#define TWO_SEC_SAMPLES           2 * SAMPLING_RATE
#define QRS_THRESHOLD_FRACTION    0.4
#define TRUE 1
#define FALSE 0

extern UART_HandleTypeDef huart1;

void ADS1292_Genarate();

//************************************************************************

#endif

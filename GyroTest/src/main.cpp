 /*!
  * accelGyro.ino
  *
  * I2C addr:
  *   0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
  *   0x69: set I2C address by parameter
  *
  * Through the example, you can get the sensor data by using getSensorData:
  * get acell by paremeter onlyAccel;
  * get gyro by paremeter onlyGyro;
  * get both acell and gyro by paremeter bothAccelGyro.
  * 
  * With the rotation of the sensor, data changes are visible.
  *
  * Copyright   [DFRobot](http://www.dfrobot.com), 2016
  * Copyright   GNU Lesser General Public License
  *
  * version  V1.0
  * date  2017-11-27
  */

#include <DFRobot_BMI160.h>
#include <WiFi.h>

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x68;
bool readStep = false;
/*
#if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_MEGA2560 || defined ARDUINO_AVR_PRO
  //interrupt number of uno and mega2560 is 0
  int pbIn = 2;
#elif ARDUINO_AVR_LEONARDO
  //interrupt number of uno and leonardo is 0
  int pbIn = 3; 
#else
  int pbIn = 13;
#endif
*/
int pbIn = 4;
/*the bmi160 have two interrput interfaces*/
int int1 = 1;
int int2 = 2;

uint16_t stepCounter = 0;
uint8_t TransferToSTM32Buffer[5];
long time_elapsed = 0;
int i;

const char *ssid = "DebugTi815";
const char *password = "7418520963";
IPAddress softLocal(192,168,43,70); 
IPAddress softGateway(192,168,43,1);
IPAddress softSubnet(255,255,255,0);
WiFiClient client;
WiFiServer server(8080);
long count;
bool messagecome;

void stepChange()
{
  stepCounter++;
  //once the step conter is changed, the value can be read 
  readStep = true;
}

void setup(){
  Serial.begin(115200);
  delay(100);

  WiFi.mode(WIFI_STA);
  WiFi.config(softLocal, softGateway, softSubnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("No connect");
  }
  Serial.println(WiFi.localIP());
  server.begin();
  server.setNoDelay(true);
  client = server.available();
  if(client)
  {
    if(client.connected())
    {
      Serial.write("Connected");
    }
  }
 pinMode(A11,INPUT);
 count=100;
 messagecome=false;
  
  //set and init the bmi160 i2c address  
  while (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("i2c init fail");
    delay(1000); 
  }
  
  //set interrput number to int1 or int2
  if (bmi160.setInt(int2) != BMI160_OK){
    Serial.println("set interrput fail");
    while(1);
  }

  //set the bmi160 mode to step counter
  if (bmi160.setStepCounter() != BMI160_OK){
    Serial.println("set step fail");
    while(1);
  }
  
  //set the bmi160 power model,contains:stepNormalPowerMode,stepLowPowerMode
  if (bmi160.setStepPowerMode(bmi160.stepNormalPowerMode) != BMI160_OK){
    Serial.println("set setStepPowerMode fail");
    while(1);
  }

  attachInterrupt(pbIn, stepChange, FALLING);

  // Serial.println(pbIn);
}

void loop(){
    if(count>=100)
  {   
    if(client)
    {
      if(client.connected())
      {
        count=0;
      }
    }
    else 
    {
      client = server.available();
    }
  }
  // int i = 0;
  // int rslt;
  // int16_t accelGyro[6]={0}; 
  
  // //get both accel and gyro data from bmi160
  // //parameter accelGyro is the pointer to store the data
  // rslt = bmi160.getAccelGyroData(accelGyro);
  // if(rslt == 0){
  //   for(i=0;i<6;i++){
  //     if (i<3){
  //       //the first three are gyro datas
  //       Serial.print(accelGyro[i]*3.14/180.0);Serial.print("\t");
  //     }else{
  //       //the following three data are accel datas
  //       Serial.print(accelGyro[i]/16384.0);Serial.print("\t");
  //     }
  //   }
  //   Serial.println();
  // }else{
  //   Serial.println("err");
  // }
  // delay(100);

//步数数据采集
  if(millis() > time_elapsed){
    // if (bmi160.readStepCounter(&stepCounter)==BMI160_OK){
      TransferToSTM32Buffer[0] = 0x01;
      TransferToSTM32Buffer[1] = 0x02;
      TransferToSTM32Buffer[2] = stepCounter;
      TransferToSTM32Buffer[3] = stepCounter >> 8;
      TransferToSTM32Buffer[4] = TransferToSTM32Buffer[0] + TransferToSTM32Buffer[1] + TransferToSTM32Buffer[2] + TransferToSTM32Buffer[3];
      for(i=0; i<5; i++)
      {
        Serial.write(TransferToSTM32Buffer[i]);
      }
    // }
    time_elapsed += 50;
  }

  if(digitalRead(A11)==LOW)
  {
    messagecome=true;
    //Serial.printf("16 lOW\r\n");
  }
  if(digitalRead(A11)==HIGH&&messagecome)
  {
      size_t len = Serial.available();
      uint8_t sbuf[len];
      //Serial.printf("%d\r\n",len);
      Serial.readBytes(sbuf, len);
      //if(sbuf[0]==0xaa && sbuf[1]==0xfa && sbuf[21]==0xaa)
      client.write(sbuf, len);
      //Serial.write(sbuf, len);
      //delay(1000);
      count++;
      messagecome=false;
      //Serial.printf("16 HIGH\r\n");
  }
  //Step
  //   if (readStep){
  //   uint16_t stepCounter = 0;
  //   //read step counter from hardware bmi160 
  //   if (bmi160.readStepCounter(&stepCounter)==BMI160_OK){
  //     // Serial.print("step counter = ");
  //     Serial.print(stepCounter);
  //   }
  //     /* code */
    
  //   readStep = false;
  // }
  // else
  // {
  //   Serial.print(0xff);
  // }
  

  /*
   * //only read accel data from bmi160
   * int16_t onlyAccel[3]={0};
   * bmi160.getAccelData(onlyAccel);
   */

  /*
   * ////only read gyro data from bmi160
   * int16_t onlyGyro[3]={0};
   * bmi160.getGyroData(onlyGyro);
   */
}
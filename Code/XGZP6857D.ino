//Libraries

#include <Arduino.h>

#include <Wire.h>

 

//constants & variables

bool error=0;                             //reading error variable (not used in this sketch)

float pressureKPA=0.0;                    //declare pressure KPa variable

float pressureCMH2O=0.0;                  //declare pressure cmH2O variable

float temperatureC=0.0;                   //declare temperature Celsius variable

float temperatureF=0.0;                   //declare temperature Fahrenheit

long startMillis=0;                       //start sampling milliseconds time

const long period=2000;                   //time between samples in milliseconds

 

//hardware settings

 

//functions

bool readCFSensor(float &temp, float &press,byte sensorAddress);

 

void setup() {

  Serial.begin(115200);                     //begin serial port

  Wire.begin();                           //begin i2c bus

  delay(200);                             //wait for electronics

 

}

 

void loop() {

 

  Serial.println("");                                                                           //

  Serial.println(F("-------------------------------------------------------------------------"));  //samples division

  Serial.println("");                                                                           //

 

  startMillis=millis();                                             //save the starting time

  error = readCFSensor(temperatureC,pressureKPA,0x6D);              //start conversion and read on pressure sensor ad 0x6D address

 

  Serial.println("Temperature *C: " + String(temperatureC,1));      //print *C temperature

  temperatureF= temperatureC * 1.8 + 32;                            //*C to *F conversion

  Serial.println("Temperature *F: " + String(temperatureF,1));      //print *F temperature

 

  Serial.println("Pressure KPa: " + String(pressureKPA,3));         //print KPa pressure

  pressureCMH2O = pressureKPA * 10.1972;                            //KPa to cmH2O conversion

  Serial.println("Pressure cmH2O: " + String(pressureCMH2O,2));     //print cmH2O pressure

 

  while((millis()-startMillis) < period);                           //waits until period done

}

 

bool readCFSensor(float &temp, float &press,byte sensorAddress) {

 

  byte reg0xA5=0;

 

  Wire.beginTransmission(sensorAddress);    //send Start and sensor address

  Wire.write(0xA5);                         //send 0xA5 register address

  Wire.endTransmission();                   //send Stop

  Wire.requestFrom(sensorAddress,byte(1));  //send Start and read 1 byte command from sensor address

  if(Wire.available()){                     //check if data is available on i2c buffer

    reg0xA5 = Wire.read();                  //read 0xA5 register value

  }

  Wire.endTransmission();                   //send Stop

  Serial.println("Register 0xA5 read: " + String(reg0xA5,HEX));           //for debugging purposes

 

  reg0xA5 = reg0xA5 & 0xFD;                 //mask 0xA5 register AND 0xFD to set ADC output calibrated data

  Wire.beginTransmission(sensorAddress);    //send Start and sensor address

  Wire.write(0xA5);                         //send 0xA5 register address

  Wire.write(reg0xA5);                      //send 0xA5 regiter new value

  Wire.endTransmission();                   //send Stop

  Serial.println("Write 0xA5 register: " + String(reg0xA5,HEX));         //for debugging purposes

 

  Wire.beginTransmission(sensorAddress);    //send Start and sensor address

  Wire.write(0x30);                         //send 0x30 register address

  Wire.write(0x0A);                         //set and start combined conversion

  Wire.endTransmission();                   //send Stop

  Serial.println("Write 0x0A on 0x30 register and start conversion");    //for debugging purposes

 

  byte reg0x30 = 0x08;                      //declare byte variable for 0x30 register copy (0x08 initializing for while enter)

  while((reg0x30 & 0x08) > 0) {             //loop while bit 3 of 0x30 register copy is 1

    delay(1);                               //1mS delay

    Wire.beginTransmission(sensorAddress);  //send Start and sensor address

    Wire.write(0x30);                       //send 0x30 register address

    Wire.endTransmission();                 //send Stop

    Wire.requestFrom(sensorAddress,byte(1));//send Start and read 1 byte command from sensor address

    if(Wire.available()){                   //check if data is available on i2c buffer

      reg0x30 = Wire.read();                //read 0x30 register value

    }

    Wire.endTransmission();                 //send Stop

    Serial.println("Register 0x30 read: " + String(reg0x30,HEX));         //for debugging purposes

  }

 

  unsigned long pressure24bit;              //declare 32bit variable for pressure ADC 24bit value

  byte pressHigh=0;                         //declare byte temporal pressure high byte variable

  byte pressMid=0;                          //declare byte temporal pressure middle byte variable

  byte pressLow=0;                          //declare byte temporal pressure low byte variable

 

  Wire.beginTransmission(sensorAddress);    //send Start and sensor address

  Wire.write(0x06);                         //send pressure high byte register address

  Wire.endTransmission();                   //send Stop

  Wire.requestFrom(sensorAddress,byte(3));  //send Start and read 1 byte command from sensor address

  while(Wire.available() < 3);              //wait for 3 byte on buffer

  pressHigh = Wire.read();                  //read pressure high byte

  pressMid = Wire.read();                   //read pressure middle byte

  pressLow = Wire.read();                   //read pressure low byte

  Wire.endTransmission();                   //send Stop

 

  Serial.print(String(pressHigh));            //for debugging purposes

  Serial.print("," + String(pressMid));              //for debugging purposes

  Serial.print("," + String(pressLow));              //for debugging purposes

 

  pressure24bit = pressure24bit | pressHigh;

  pressure24bit = pressure24bit & 0x000000FF;

  pressure24bit = pressure24bit << 8;

 

  pressure24bit = pressure24bit | pressMid;

  pressure24bit = pressure24bit & 0x0000FFFF;

  pressure24bit = pressure24bit << 8;

 

  pressure24bit = pressure24bit | pressLow;

  pressure24bit = pressure24bit & 0x00FFFFFF;

 

  Serial.print(":" + String(pressure24bit));      //for debugging purposes

  int temp16bit=0;                          //declare 16bit variable for temperature ADC value

  byte tempHigh=0;                          //declare temperature high byte variable

  byte tempLow=0;                           //declare temperature low byte variable

 

  Wire.beginTransmission(sensorAddress);    //send Start and sensor address

  Wire.write(0x09);                         //send temperature high byte register address

  Wire.endTransmission();                   //send Stop

  Wire.requestFrom(sensorAddress,byte(2));  //send Start and read 1 byte command from sensor address

  while(Wire.available() < 2);              //wait for 2 byte on buffer

  tempHigh = Wire.read();                   //read temperature high byte

  tempLow = Wire.read();                    //read temperature low byte

  Wire.endTransmission();                   //send Stop

  Serial.print(" - " + String(tempHigh));           //for debugging purposes

  Serial.print("," + String(tempLow));            //for debugging purposes

 

  temp16bit = tempHigh * 256 + tempLow;     //merge of 16bit temperature ADC value

  Serial.println(":" + String(temp16bit));       //for debugging purposes

 

  temp = float(temp16bit)/256;              //real Celsius temperature calculation

 

 

  if(pressure24bit > 8388608) {                                         //check sign bit for two's complement

    press = (float(pressure24bit) - float(16777216)) * 0.0000078125;    //KPa negative pressure calculation

  }

  else {                                                                //no sign

    press = float(pressure24bit) * 0.0000078125;                        //KPa positive pressure calculation

  }

 

  return 0;

}

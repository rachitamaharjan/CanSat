#include <Wire.h>
#include <ThingSpeak.h>                   // ThingSpeak Library
#include <ESP8266WiFi.h>                  // Wifi library for ESP8266 of NodeMCU
#include <SFE_BMP180.h>

// You will need to create an SFE_BMP180 object, here called "pressure":
SFE_BMP180 pressure;

#define ALTITUDE 1655.0 // Altitude in meters

// Set Network Parameters
const char* ssid     = "WELL_WAY";       // WiFi Name
const char* password = "nepal123";   // WiFi password

// ThingSpeak information
char thingSpeakAddress[] = "api.thingspeak.com";
unsigned long channelID = 874723;                         // ChannelID defined in ThingSpeak profile
char* readAPIKey = "RXVA9IMUFWHT4UEB";                    // API key to read data
char* writeAPIKey = "S6JJXPWYHE0UGVZ9";                   // API key to write data
unsigned int dataField1 = 1;                            // Field to write Acceleration X data 
unsigned int dataField2 = 2;                            // Field to write Acceleration Y data 
unsigned int dataField3 = 3;                            // Field to write Acceleration Z data
unsigned int dataField4 = 4;                            // Field to write Gyro X data
unsigned int dataField5 = 5;                            // Field to write Gyro Y data
unsigned int dataField6 = 6;                            // Field to write Gyro Z data
unsigned int dataField7 = 7;                            // Field to write altitude data
unsigned int dataField8 = 8;                            // Field to write Pressure data

WiFiClient client;                        // Create client object

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = 12; //D6 or GPI12
const uint8_t sda = 13; //D7 or GPI13

// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

void setup()
{
  
  Wire.begin(sda, scl);
  MPU6050_Init();

  Serial.begin(115200);
  Serial.println("Start");

  // Initialize the sensor (it is important to get calibration values stored on the device).
  if (pressure.begin())
    Serial.println("BMP180(pressure) init success");
  else
  {
    Serial.println("BMP180(pressure) init fail\n\n");
    while(1); // Pause forever.
  }

  //THINGSPEAK
  WiFi.begin( ssid, password );
  while (WiFi.status() != WL_CONNECTED)
  {
      delay(500);
      Serial.println("Connecting to WiFi");
  }

  Serial.println( "Connected" );
  ThingSpeak.begin( client );
}
void loop()
{
  double Ax, Ay, Az, Gx, Gy, Gz;
  float Ax1, Ay1, Az1,T1,a1, P1, Gx1, Gy1, Gz1;
  
  //Pressure sensor
  char status;
  double T,P,p0,a;
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  //T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;


  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  //Serial.print(" T: "); Serial.print(T);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);

  delay(100);

  Ax1 = float(Ax); 
  Ay1 = float(Ay1); 
  Az1 = float(Az); 
  P1 = float(P); 
  a1 = float(a); 
  Gx1 = float(Gx); 
  Gy1 = float(Gy); 
  Gz1 = float(Gz);
  float b;

  
  //Pressure sensor

  // Loop here getting pressure readings every 10 seconds.

  // If you want sea-level-compensated pressure, as used in weather reports,
  // you will need to know the altitude at which your measurements are taken.
  // We're using a constant called ALTITUDE in this sketch:
  
  Serial.println();
  Serial.print("provided altitude: ");
  Serial.print(ALTITUDE,0);
  Serial.print(" meters, ");
  Serial.print(ALTITUDE*3.28084,0);
  Serial.println(" feet");
  
  // If you want to measure altitude, and not pressure, you will instead need
  // to provide a known baseline pressure. This is shown at the end of the sketch.

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");

          // The pressure sensor returns absolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sea level function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.print(" mb, ");
          Serial.print(p0*0.0295333727,2);
          Serial.println(" inHg");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = pressure.altitude(P,p0);
          Serial.print("computed altitude: ");
          Serial.print(a,0);
          Serial.print(" meters, ");
          Serial.print(a*3.28084,0);
          Serial.println(" feet");
        }
        else Serial.println("error retrieving pressure measurement(bmp)\n");
      }
      else Serial.println("error starting pressure measurement(bmp)\n");
    }
    else Serial.println("error retrieving temperature measurement(bmp)\n");
  }
  else Serial.println("error starting temperature measurement(bmp)\n");

  delay(5000);  // Pause for 5 seconds.
  //Send data to Thingspeak
  ThingSpeak.setField(dataField1 , Ax1 );
  ThingSpeak.setField(dataField2 , Ay1 );
  ThingSpeak.setField(dataField3 , Az1 );
  ThingSpeak.setField(dataField4 , Gx1 );
  ThingSpeak.setField(dataField5 , Gy1 );
  ThingSpeak.setField(dataField6 , Gz1 );
  ThingSpeak.setField(dataField7 , a1 );
  ThingSpeak.setField(dataField8 , P1 );

  ThingSpeak.writeFields( channelID, writeAPIKey );
  b=ThingSpeak.writeFields( channelID, writeAPIKey );
  Serial.print("OH NOOOOO, Data not sent!!");
  Serial.print(b);
  if (ThingSpeak.writeFields( channelID, writeAPIKey )!=210)
  {
    Serial.print("OH NOOOOO, Data not sent!");
  }
  if (ThingSpeak.writeFields( channelID, writeAPIKey )==210)
  {
    Serial.print("Oh YAAAY, Data Sent!!");
  }
}
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

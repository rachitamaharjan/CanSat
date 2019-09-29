#include <DHT.h>
#include <Wire.h>
#include <ThingSpeak.h>                   // ThingSpeak Library
#include <ESP8266WiFi.h>                  // Wifi library for ESP8266 of NodeMCU

// Set Network Parameters
const char* ssid     = "simple";       // WiFi Name
const char* password = "hahaha123";   // WiFi password

// ThingSpeak information
char thingSpeakAddress[] = "api.thingspeak.com";
unsigned long channelID = 871795;                         // ChannelID defined in ThingSpeak profile
char* readAPIKey = "7ROCGZ8U0YKEZQD5";                    // API key to read data
char* writeAPIKey = "W36CGX1P9RAD6SQD";                   // API key to write data
unsigned int dataField1 = 1;                            // Field to write temperature data
unsigned int dataField2 = 2;                            // Field to write humidity data
unsigned int dataField3 = 3;                            // Field to write Acceleration X data
unsigned int dataField4 = 4;                            // Field to write Acceleration Y data
unsigned int dataField5 = 5;                            // Field to write Acceleration Z data
unsigned int dataField6 = 6;                            // Field to write Gyro X data
unsigned int dataField7 = 7;                            // Field to write Gyro Y data
unsigned int dataField8 = 8;                            // Field to write Gyro Z data

WiFiClient client;                        // Create client object

#define DHTPIN 12 // D2 or GPIO4
#define DHTTYPE DHT11
DHT dht(DHTPIN,DHTTYPE);

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = 5; //D6 or GPI12
const uint8_t sda = 4; //D7 or GPI13

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
  
  dht.begin();
  Wire.begin(sda, scl);
  MPU6050_Init();

  Serial.begin(115200);
  Serial.println("Start");

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
  delay(2000);
  float h= (float)dht.readHumidity();
  float t= (float)dht.readTemperature();
  if(isnan(h)||isnan(t))
  {
    Serial.print("fail to read");
    return;
  }
  Serial.print("humidity is = ");
  Serial.println(h);
  Serial.print("temperature is = ");
  Serial.println(t);

  double Ax, Ay, Az, T, Gx, Gy, Gz;
  float Ax1, Ay1, Az1, t1, h1, Gx1, Gy1, Gz1;
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;


  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  Serial.print(" T: "); Serial.print(T);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);

  delay(100);

  Ax1 = float(Ax); 
  Ay1 = float(Ay1); 
  Az1 = float(Az); 
  t1 = float(t); 
  h1 = float(h); 
  Gx1 = float(Gx); 
  Gy1 = float(Gy); 
  Gz1 = float(Gz);
  float a;

  //Send data to Thingspeak
  ThingSpeak.setField(dataField1 , t1 );
  ThingSpeak.setField(dataField2 , h1 );
  ThingSpeak.setField(dataField3 , Ax1 );
  ThingSpeak.setField(dataField4 , Ay1 );
  ThingSpeak.setField(dataField5 , Az1 );
  ThingSpeak.setField(dataField6 , Gx1 );
  ThingSpeak.setField(dataField7 , Gy1 );
  ThingSpeak.setField(dataField8 , Gz1 );

  ThingSpeak.writeFields( channelID, writeAPIKey );
  a=ThingSpeak.writeFields( channelID, writeAPIKey );
  Serial.print("HELL NOOOOO");
  Serial.print(a);

  if (ThingSpeak.writeFields( channelID, writeAPIKey )==200)
  {
    Serial.print("HELL YAAAS");
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

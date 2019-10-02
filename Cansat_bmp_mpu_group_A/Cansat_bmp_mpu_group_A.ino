#include <Wire.h>
#include <ThingSpeak.h>                   // ThingSpeak Library
#include <ESP8266WiFi.h>                  // Wifi library for ESP8266 of NodeMCU
#include <Adafruit_BMP085.h>

//mpu
const int MPU6050_addr=0x68;
int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ;

//bmp
Adafruit_BMP085 bmp;

// Set Network Parameters
const char* ssid     = "netis_0155D9";       // WiFi Name
const char* password = "password";   // WiFi password

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


void setup()
{
  
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(115200);
  Serial.println("Start");

  //bmp
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
  while (1) {}
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
  //mpu
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);
  AccX=Wire.read()<<8|Wire.read();
  AccY=Wire.read()<<8|Wire.read();
  AccZ=Wire.read()<<8|Wire.read();
  Temp=Wire.read()<<8|Wire.read();
  GyroX=Wire.read()<<8|Wire.read();
  GyroY=Wire.read()<<8|Wire.read();
  GyroZ=Wire.read()<<8|Wire.read();
  Serial.print("AccX = "); Serial.print(AccX);
  Serial.print(" || AccY = "); Serial.print(AccY);
  Serial.print(" || AccZ = "); Serial.print(AccZ);
  Serial.print(" || Temp = "); Serial.print(Temp/340.00+36.53);
  Serial.print(" || GyroX = "); Serial.print(GyroX);
  Serial.print(" || GyroY = "); Serial.print(GyroY);
  Serial.print(" || GyroZ = "); Serial.println(GyroZ);
  delay(100);
  
  
  //bmp
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
    
  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
    
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(bmp.readSealevelPressure());
  Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  Serial.print("Real altitude = ");
  Serial.print(bmp.readAltitude(102000));
  Serial.println(" meters");
    
  Serial.println();
  delay(500);
  
 
  float b;

  
  
  //Send data to Thingspeak
  ThingSpeak.setField(dataField1 , (float)AccX );
  ThingSpeak.setField(dataField2 , (float)AccY );
  ThingSpeak.setField(dataField3 , (float)AccZ );
  ThingSpeak.setField(dataField4 , (float)GyroX );
  ThingSpeak.setField(dataField5 , (float)GyroY );
  ThingSpeak.setField(dataField6 , (float)GyroZ );
  ThingSpeak.setField(dataField7 , (float)(bmp.readAltitude(102000)) );
  ThingSpeak.setField(dataField8 , (float)(bmp.readPressure()));

  ThingSpeak.writeFields( channelID, writeAPIKey );
  b=ThingSpeak.writeFields( channelID, writeAPIKey );
  //Serial.print("OH NOOOOO, Data not sent!!");
  Serial.print(b);
  Serial.print("\n");
  if (ThingSpeak.writeFields( channelID, writeAPIKey )!=-210.00)
  {
    Serial.print("OH NOOOOO, Data not sent!");
  }
  if (ThingSpeak.writeFields( channelID, writeAPIKey )==-210.00)
  {
    Serial.print("Oh YAAAY, Data Sent!!");
  }
}

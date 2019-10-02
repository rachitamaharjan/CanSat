#include <DHT.h>
#include <Wire.h>
#include <ThingSpeak.h>                   // ThingSpeak Library
#include <ESP8266WiFi.h>                  // Wifi library for ESP8266 of NodeMCU
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

// Set Network Parameters
const char* ssid     = "Brihaspati";       // WiFi Name
const char* password = "brihaspati@vidyasadan";   // WiFi password

// ThingSpeak information
char thingSpeakAddress[] = "api.thingspeak.com";
unsigned long channelID = 876109;                         // ChannelID defined in ThingSpeak profile
char* readAPIKey = "DSHFRHMOXJJG0WH9";                    // API key to read data
char* writeAPIKey = "BSGWTPOSRDF83NWL";                   // API key to write data
unsigned int dataField1 = 1;                            // Field to write temperature data
unsigned int dataField2 = 2;                            // Field to write humidity data
unsigned int dataField3 = 3;                            // Field to write altitude data
unsigned int dataField4 = 4;                            // Field to write pressure data
//unsigned int dataField5 = 5;                            // Field to write data
//unsigned int dataField6 = 6;                            // Field to write data
//unsigned int dataField7 = 7;                            // Field to write data
//unsigned int dataField8 = 8;                            // Field to write data

WiFiClient client;                        // Create client object

#define DHTPIN 12 // D6 or GPI12
#define DHTTYPE DHT11
DHT dht(DHTPIN,DHTTYPE);
// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;


void setup()
{
  
  dht.begin();
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
  delay(2000);
  float h= (float)dht.readHumidity();
  float t= (float)dht.readTemperature();
  if(isnan(h)||isnan(t))
  {
    Serial.print("\n failed to read humidity / temperature");
    return;
  }
  Serial.print("\n humidity is = ");
  Serial.println(h);
  Serial.print("temperature is = ");
  Serial.println(t);

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


  //Send data to Thingspeak
  ThingSpeak.setField(dataField1 , (float)t );
  ThingSpeak.setField(dataField2 , (float)h );
  ThingSpeak.setField(dataField3 , (float)(bmp.readAltitude(102000)) );
  ThingSpeak.setField(dataField4 , (float)(bmp.readPressure()) );
//  ThingSpeak.setField(dataField5 , Az1 );
//  ThingSpeak.setField(dataField6 , Gx1 );
//  ThingSpeak.setField(dataField7 , Gy1 );
//  ThingSpeak.setField(dataField8 , Gz1 );

  ThingSpeak.writeFields( channelID, writeAPIKey );
  float b=ThingSpeak.writeFields( channelID, writeAPIKey );
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

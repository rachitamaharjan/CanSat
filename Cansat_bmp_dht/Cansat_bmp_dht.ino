#include <DHT.h>
#include <Wire.h>
#include <ThingSpeak.h>                   // ThingSpeak Library
#include <ESP8266WiFi.h>                  // Wifi library for ESP8266 of NodeMCU
#include <SFE_BMP180.h>

// Set Network Parameters
const char* ssid     = "WELL_WAY";       // WiFi Name
const char* password = "nepal123";   // WiFi password

// ThingSpeak information
char thingSpeakAddress[] = "api.thingspeak.com";
unsigned long channelID = 871795;                         // ChannelID defined in ThingSpeak profile
char* readAPIKey = "7ROCGZ8U0YKEZQD5";                    // API key to read data
char* writeAPIKey = "W36CGX1P9RAD6SQD";                   // API key to write data
unsigned int dataField1 = 1;                            // Field to write temperature data
unsigned int dataField2 = 2;                            // Field to write humidity data
unsigned int dataField3 = 3;                            // Field to write altitude data
unsigned int dataField4 = 4;                            // Field to write pressure data
//unsigned int dataField5 = 5;                            // Field to write data
//unsigned int dataField6 = 6;                            // Field to write data
//unsigned int dataField7 = 7;                            // Field to write data
//unsigned int dataField8 = 8;                            // Field to write data

WiFiClient client;                        // Create client object

#define DHTPIN 12 // D2 or GPIO4
#define DHTTYPE DHT11
DHT dht(DHTPIN,DHTTYPE);
// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// You will need to create an SFE_BMP180 object, here called "pressure":
SFE_BMP180 pressure;
#define ALTITUDE 1655.0 // Altitude in meters

void setup()
{
  
  dht.begin();
  Serial.begin(115200);
  Serial.println("Start");
  
  // Initialize the sensor (it is important to get calibration values stored on the device).

  if (pressure.begin())
    Serial.println("/n BMP180(pressure) init success");
  else
  {
    Serial.println("/n BMP180(pressure) init fail\n\n");
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
  delay(2000);
  float h= (float)dht.readHumidity();
  float t= (float)dht.readTemperature();
  if(isnan(h)||isnan(t))
  {
    Serial.print("\n failed to read humidity / temperature");
    return;
  }
  Serial.print("humidity is = ");
  Serial.println(h);
  Serial.print("temperature is = ");
  Serial.println(t);

  char status;
  double T,P,p0,a,b;

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
  ThingSpeak.setField(dataField1 , (float)t );
  ThingSpeak.setField(dataField2 , (float)h );
  ThingSpeak.setField(dataField3 , (float)a );
  ThingSpeak.setField(dataField4 , (float)P );
//  ThingSpeak.setField(dataField5 , Az1 );
//  ThingSpeak.setField(dataField6 , Gx1 );
//  ThingSpeak.setField(dataField7 , Gy1 );
//  ThingSpeak.setField(dataField8 , Gz1 );

  ThingSpeak.writeFields( channelID, writeAPIKey );
  b=ThingSpeak.writeFields( channelID, writeAPIKey );
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

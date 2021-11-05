/*
* Project for the 2021 TSI Hackathon. This is a mobile IAQ meter that
* will incorporate a Sesnsiron Sps 30 PM sensor and Neo-6 GPS pushing data to the cloud via
* an ESP32.
*/

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

const unsigned int writeInterval = 25000; // write interval (in ms)

//gps consts
static const int GPS_RXPin = 16, GPS_TXPin = 17;

static const int GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(GPS_RXPin, GPS_TXPin); // The serial connection to the GPS device


// setup
void setup() 
{
    Serial.begin(115200);
    delay(500); 
    gpsSetup();
    Serial.println("Setup Done...");
}

void gpsSetup()
{
    ss.begin(GPSBaud);
}

// loop
void loop() 
{   
    // This sketch displays information every time a new sentence is correctly encoded.
    while (ss.available() > 0)
    {        
        if (gps.encode(ss.read()))
        {
            displayInfo();
        }
        
        if (millis() > 5000 && gps.charsProcessed() < 10)
        {
            Serial.println(F("No GPS detected: check wiring."));
            while(true);
        }
    }
}

// GPS displayInfo
void displayInfo() 
{ 
    if (gps.location.isValid()) 
    {
        Serial.println("GPS Data: ");  
        
        // print gps data
        Serial.print("Lat: "); Serial.print(gps.location.lng()); 
        Serial.print(" Long: "); Serial.println(gps.location.lat()); 

        Serial.print("Lat degree: "); Serial.print(gps.location.lat(), 6); // Latitude in degrees (double)
        Serial.print(" Long degree: "); Serial.println(gps.location.lng(), 6); // Longitude in degrees (double)

        Serial.print("Altitude meters: "); Serial.println(gps.altitude.meters()); // Altitude in meters (double)

        // delay
        delay(writeInterval);
    } 
    else 
    {
        Serial.println(F("INVALID"));
    }
}

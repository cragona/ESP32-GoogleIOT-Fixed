/************************************************************************************
 *  Copyright (c) January 2019, version 1.0     Paul van Haastrecht
 *
 *  Version 1.1 Paul van Haastrecht
 *  - Changed the I2C information / setup.
 *
 *  Version 1.1.1 Paul van Haastrecht / March 2020
 *  - Fixed compile errors and warnings.
 *
 *  Version 1.1.2 Paul van Haastrecht / March 2020
 *  - added versions level to GetDeviceInfo()
 *
 *  Version 1.1.3 Paul van Haastrecht / July 2020
 *  - added embedded support for Arduino Due
 *
 *  =========================  Highlevel description ================================
 *
 *  This basic reading example sketch will connect to an SPS30 for getting data and
 *  display the available data
 *
 *  =========================  Hardware connections =================================
 *  /////////////////////////////////////////////////////////////////////////////////
 *  ## UART UART UART UART UART UART UART UART UART UART UART UART UART UART UART  ##
 *  /////////////////////////////////////////////////////////////////////////////////
 *
 *  Sucessfully test has been performed on an ESP32:
 *
 *  Using serial port1, setting the RX-pin(25) and TX-pin(26)
 *  Different setup can be configured in the sketch.
 *
 *  SPS30 pin     ESP32
 *  1 VCC -------- VUSB
 *  2 RX  -------- TX  pin 26
 *  3 TX  -------- RX  pin 25
 *  4 Select      (NOT CONNECTED)
 *  5 GND -------- GND
 *
 *  Also successfully tested on Serial2 (default pins TX:17, RX: 16)
 *  NO level shifter is needed as the SPS30 is TTL 5V and LVTTL 3.3V compatible
 *  ..........................................................
 *  Successfully tested on ATMEGA2560 / Arduino Due
 *  Used SerialPort2. No need to set/change RX or TX pin
 *  SPS30 pin     ATMEGA
 *  1 VCC -------- 5V
 *  2 RX  -------- TX2  pin 16
 *  3 TX  -------- RX2  pin 17
 *  4 Select      (NOT CONNECTED)
 *  5 GND -------- GND
 *
 *  Also tested on SerialPort1 and Serialport3 successfully
 *  .........................................................
 *  Failed testing on UNO
 *  Had to use softserial as there is not a separate serialport. But as the SPS30
 *  is only working on 115K the connection failed all the time with CRC errors.
 *
 *  Not tested ESP8266
 *  As the power is only 3V3 (the SPS30 needs 5V)and one has to use softserial
 *
 *  //////////////////////////////////////////////////////////////////////////////////
 *  ## I2C I2C I2C  I2C I2C I2C  I2C I2C I2C  I2C I2C I2C  I2C I2C I2C  I2C I2C I2C ##
 *  //////////////////////////////////////////////////////////////////////////////////
 *  NOTE 1:
 *  Depending on the Wire / I2C buffer size we might not be able to read all the values.
 *  The buffer size needed is at least 60 while on many boards this is set to 32. The driver
 *  will determine the buffer size and if less than 64 only the MASS values are returned.
 *  You can manually edit the Wire.h of your board to increase (if you memory is larg enough)
 *  One can check the expected number of bytes with the I2C_expect() call as in this example
 *  see detail document.
 *
 *  NOTE 2:
 *  As documented in the datasheet, make sure to use external 10K pull-up resistor on
 *  both the SDA and SCL lines. Otherwise the communication with the sensor will fail random.
 *
 *  ..........................................................
 *  Successfully tested on ESP32
 *
 *  SPS30 pin     ESP32
 *  1 VCC -------- VUSB
 *  2 SDA -------- SDA (pin 21)
 *  3 SCL -------- SCL (pin 22)
 *  4 Select ----- GND (select I2c)
 *  5 GND -------- GND
 *
 *  The pull-up resistors should be to 3V3
 *  ..........................................................
 *  Successfully tested on ATMEGA2560 / Arduino Due
 *
 *  SPS30 pin     ATMEGA
 *  1 VCC -------- 5V
 *  2 SDA -------- SDA
 *  3 SCL -------- SCL
 *  4 Select ----- GND  (select I2c)
 *  5 GND -------- GND
 *
 *  ..........................................................
 *  Successfully tested on UNO R3
 *
 *  SPS30 pin     UNO
 *  1 VCC -------- 5V
 *  2 SDA -------- A4
 *  3 SCL -------- A5
 *  4 Select ----- GND  (select I2c)
 *  5 GND -------- GND
 *
 *  When UNO-board is detected the UART code is excluded as that
 *  does not work on UNO and will save memory. Also some buffers
 *  reduced and the call to GetErrDescription() is removed to allow
 *  enough memory.
 *  ..........................................................
 *  Successfully tested on ESP8266
 *
 *  SPS30 pin     External     ESP8266
 *  1 VCC -------- 5V
 *  2 SDA -----------------------SDA
 *  3 SCL -----------------------SCL
 *  4 Select ----- GND --------- GND  (select I2c)
 *  5 GND -------- GND --------- GND
 *
 *  The pull-up resistors should be to 3V3 from the ESP8266.
 *
 *  ================================= PARAMETERS =====================================
 *
 *  From line 148 there are configuration parameters for the program
 *
 *  ================================== SOFTWARE ======================================
 *  Sparkfun ESP32
 *
 *    Make sure :
 *      - To select the Sparkfun ESP32 thing board before compiling
 *      - The serial monitor is NOT active (will cause upload errors)
 *      - Press GPIO 0 switch during connecting after compile to start upload to the board
 *
 *  ================================ Disclaimer ======================================
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  ===================================================================================
 *
 *  NO support, delivered as is, have fun, good luck !!
 */

#include "sps30.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

/////////////////////////////////////////////////////////////
/*define communication channel to use for SPS30
 valid options:
 *   I2C_COMMS              use I2C communication
 *   SOFTWARE_SERIAL        Arduino variants (NOTE)
 *   SERIALPORT             ONLY IF there is NO monitor attached
 *   SERIALPORT1            Arduino MEGA2560, Due.  Sparkfun ESP32 Thing : MUST define new pins as defaults are used for flash memory)
 *   SERIALPORT2            Arduino MEGA2560, Due and ESP32
 *   SERIALPORT3            Arduino MEGA2560 and Due only for now

 * NOTE: Softserial has been left in as an option, but as the SPS30 is only
 * working on 115K the connection will probably NOT work on any device. */
/////////////////////////////////////////////////////////////
#define SP30_COMMS SERIALPORT1

/////////////////////////////////////////////////////////////
/* define RX and TX pin for softserial and Serial1 on ESP32
 * can be set to zero if not applicable / needed           */
/////////////////////////////////////////////////////////////
#define TX_PIN 22
#define RX_PIN 21

/////////////////////////////////////////////////////////////
/* define driver debug
 * 0 : no messages
 * 1 : request sending and receiving
 * 2 : request sending and receiving + show protocol errors */
 //////////////////////////////////////////////////////////////
#define DEBUG 0

const unsigned int writeInterval = 25000; // write interval (in ms)
//gps consts
static const int RXPin = 16, TXPin = 17;
static const int GPSBaud = 9600;

///////////////////////////////////////////////////////////////
/////////// NO CHANGES BEYOND THIS POINT NEEDED ///////////////
///////////////////////////////////////////////////////////////

// function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void serialTrigger(char * mess);
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfo();
bool read_all();

// create constructor
SPS30 sps30;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device
int counter = 0;
unsigned long startTime, currentTime;

void setup() 
{
  startTime = millis();
  serialMonitorSetup();
  gpsSetup();
  sps30Setup();
  //!! set up cloud
}

void serialMonitorSetup()
{
    Serial.begin(115200);
    ("Monitor Setup Done...");
}

void gpsSetup()
{
  ss.begin(GPSBaud);
  Serial.println("GPS Setup Done...");
}

void sps30Setup()
{
  // set pins to use for softserial and Serial1 on ESP32
  Serial.println("connecting sps30..."); delay(5000);
  if (TX_PIN != 0 && RX_PIN != 0) sps30.SetSerialPin(RX_PIN,TX_PIN);

  // Begin communication channel;
  if (! sps30.begin(SP30_COMMS))
    Errorloop((char *) "could not initialize communication channel.", 0);

  // check for SPS30 connection
  if (! sps30.probe()) Errorloop((char *) "could not probe / connect with SPS30.", 0);
  else  Serial.println(F("Detected SPS30."));

  // reset SPS30 connection
  if (! sps30.reset()) Errorloop((char *) "could not reset.", 0);

  // start measurement
  if (sps30.start()) Serial.println(F("Measurement started"));
  else Errorloop((char *) "Could NOT start measurement", 0);

  Serial.println("SPS30 Setup Done...");
  //delay(500); 
}

void loop() 
{
  gpsLoop();
}

void gpsLoop()
{
  while (ss.available() > 0)
  {   
    //Serial.println(ss.read());
    if (gps.encode(ss.read()))
    {
      displayInfo();
    }
    
    // if (millis() > 5000 && gps.charsProcessed() < 10)
    // {
    //   Serial.println("No GPS detected: check wiring.");
    //   //while(true);
    // }
  }
}

unsigned long previousRead = 0, currentRead = 0;

void displayInfo() 
{ 
    if (gps.location.isValid()) 
    {
        Serial.println(""); Serial.println(""); Serial.println("");
        Serial.print("======"); Serial.print(counter); Serial.println("======"); counter++;++++++++++++++++++++++++++++


























































































































        
        Serial.println("GPS Data: ");  
        
        // print gps data
        Serial.print("Lat: "); Serial.print(gps.location.lng()); 
        Serial.print(" Long: "); Serial.println(gps.location.lat()); 

        Serial.print("Lat degree: "); Serial.print(gps.location.lat(), 6); // Latitude in degrees (double)
        Serial.print(" Long degree: "); Serial.println(gps.location.lng(), 6); // Longitude in degrees (double)

        Serial.print("Altitude meters: "); Serial.println(gps.altitude.meters()); // Altitude in meters (double)
        // delay
        read_all();

        //print timer
        previousRead = currentRead;
        currentRead = millis();
        Serial.print("Previous Read: "); Serial.print(previousRead); Serial.print("ms - Current Read: "); Serial.print(currentRead);
        Serial.print("ms - Difference: "); Serial.print(currentRead - previousRead); Serial.println("ms");

        //delay(500);
    } 
    else 
    {
        Serial.println(F("INVALID"));
    }
}

/**
 * @brief : read and display device info
 */
void GetDeviceInfo()
{
  char buf[32];
  uint8_t ret;
  SPS30_version v;

  //try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == ERR_OK) {
    Serial.print(F("Serial number : "));
    if(strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get serial number", ret);

  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == ERR_OK)  {
    Serial.print(F("Product name  : "));

    if(strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get product name.", ret);

  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != ERR_OK) {
    Serial.println(F("Can not read version info"));
    return;
  }

  Serial.print(F("Firmware level: "));  Serial.print(v.major);
  Serial.print("."); Serial.println(v.minor);

  Serial.print(F("Library level : "));  Serial.print(v.DRV_major);
  Serial.print(".");  Serial.println(v.DRV_minor);
}

/**
 * @brief : read and display all values
 */
bool read_all()
{
  static bool header = true;
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {

    ret = sps30.GetValues(&val);

    // data might not have been ready
    if (ret == ERR_DATALENGTH){

        if (error_cnt++ > 3) {
          ErrtoMess((char *) "Error during reading values: ",ret);
          return(false);
        }
        delay(1000);
    }

    // if other error
    else if(ret != ERR_OK) {
      ErrtoMess((char *) "Error during reading values: ",ret);
      return(false);
    }

  } while (ret != ERR_OK);

  float mass = val.MassPM2;
  float number = val.NumPM2;

  Serial.println("Mass and Number 2.5");
  Serial.print(mass);
  Serial.print(F("\t"));
  Serial.println(number);
  Serial.print(F("\n"));
  return(true);
}

/**
 *  @brief : continued loop after fatal error
 *  @param mess : message to display
 *  @param r : error code
 *
 *  if r is zero, it will only display the message
 */
void Errorloop(char *mess, uint8_t r)
{
  if (r) ErrtoMess(mess, r);
  else Serial.println(mess);
  Serial.println(F("Program on hold"));
  for(;;) delay(100000);
}

/**
 *  @brief : display error message
 *  @param mess : message to display
 *  @param r : error code
 *
 */
void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];

  Serial.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}

/**
 * serialTrigger prints repeated message, then waits for enter
 * to come in from the serial port.
 */
void serialTrigger(char * mess)
{
  Serial.println();

  while (!Serial.available()) {
    Serial.println(mess);
    delay(2000);
  }

  while (Serial.available())
    Serial.read();
}

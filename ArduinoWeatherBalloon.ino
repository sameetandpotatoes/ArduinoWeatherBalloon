/* All Imported Libraries */
#include <SD.h>
#include "DHT.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include "TSL2561.h"
#include <SPI.h>
#include <Wire.h>
#include "RTClib.h"

/* All constant variables */
#define GPSECHO false //Off because we don't want to echo the GPS to the Serial monitor

//Constants for to calculate pressure for barometer
#define DHTPIN 22     
#define DHTTYPE DHT11
#define PRESH   0x80
#define PRESL   0x82
#define TEMPH   0x84
#define TEMPL   0x86
#define A0MSB   0x88
#define A0LSB   0x8A
#define B1MSB   0x8C
#define B1LSB   0x8E
#define B2MSB   0x90
#define B2LSB   0x92
#define C12MSB   0x94
#define C12LSB   0x96
#define CONVERT   0x24   
#define chipSelectPin 9
#define shutDown 7

/*
MPL115A1 sparkfun breakout baropressure meter
 For Arduino Uno:
 SDN       : pin 7
 CSN       : pin 10
 SDI/MOSI  : pin 11
 SDO/MISO  : pin 12
 SCK       : pin 13
 For Arduino Mega:
 SDN       : pin 7
 CSN       : pin 10 (we used Pin 9 because the SD card requires pin 10
 SDI/MOSI  : pin 51
 SDO/MISO  : pin 50
 SCK       : pin 52
*/

//Real time clock variable
RTC_DS1307 RTC;

int previousMillis = 0;
int interval = 1200000; //Time in milliseconds between writing to the SD card.

File dataFile; //Name of file we are writing to on the SD card

void setup()
{
  SD.begin(10,11,12,13); //The SD Shield uses 4 pins for the Arduino Mega
  
  pinMode(shutDown, OUTPUT);
  pinMode(chipSelectPin, OUTPUT);
  
  digitalWrite(chipSelectPin, HIGH);
  digitalWrite(shutDown, HIGH);
  
  Wire.begin();
  RTC.begin();
  
  //For debugging the code
  Serial.begin(9600);
  
  RTC.adjust(DateTime(__DATE__, __TIME__));
} // end of setup

void loop()
{
  int currentMillis = millis();
  if (currentMillis - previousMillis > interval) //Writing to SD card only after the interval of time has passed
  {      
    dataFile = SD.open("datalog.txt", FILE_WRITE); //Opening and closing the SD Card once, instead of after reading each sensor
    int i = 0;    
    for (i = 0; i < 8; i++)
    {
      switch (i)
      {
        case 0: 
          getGeigerData();
          break;
        case 1: 
          getGasData();
          break;
        case 2: 
          getGPSData();
          break;
        case 3: 
          getHumidityData();
          break;
        case 4: 
          getBarometerData();
          break;
        case 5: 
          getAccelerometerData(); 
          break;
        case 6: 
          getLuminosityData(); 
          break;
        case 7: 
          getClockData(); 
          break;
        }
    }
    dataFile.close(); //Closing data file
    Serial.println("Data was written.");
    previousMillis = currentMillis;  //Resetting timer   
  }
} // end of loop
void getGeigerData()
{
  dataFile.print("Geiger: ");
  int data = abs(analogRead(13) - 1024);
  dataFile.println(data);
} //end of getGeigerData()
void getGasData()
{
  dataFile.print("Outside Gas Sensor: ");
  dataFile.println(analogRead(15));
} //end of getGas2Data()
void getGPSData()
{  
  Adafruit_GPS GPS(&Serial1);
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // Setting the update rate to 1 Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
 
  if (GPS.newNMEAreceived()) // if a sentence is received, we can check the checksum, parse it...
  {
     GPS.lastNMEA();
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }    
  dataFile.print("Fix: "); dataFile.print((int)GPS.fix);
  dataFile.print(" quality: "); dataFile.println((int)GPS.fixquality); 
  if (GPS.fix) //If we get a fix, print the rest of the data
  {
      dataFile.print("Location: ");
      dataFile.print((GPS.latitude / 100.0), 4); dataFile.print(GPS.lat);
      dataFile.print(", "); 
      dataFile.print((GPS.longitude / 100.0), 4); dataFile.println(GPS.lon);
      dataFile.print("Speed (mph): "); dataFile.println(GPS.speed * 1.15078); //Converting knots to miles per hour
      dataFile.print("Angle: "); dataFile.println(GPS.angle);
      dataFile.print("Altitude: "); dataFile.println(GPS.altitude);
      dataFile.print("Satellites: "); dataFile.println((int)GPS.satellites);
  }
} //end of getGPSData()
void getClockData()
{
   DateTime now = RTC.now(); 
   dataFile.print(now.year(), DEC);
   dataFile.print('/');
   dataFile.print(now.month(), DEC);
   dataFile.print('/');
   dataFile.print(now.day(), DEC);
   dataFile.print(' ');
   dataFile.print(now.hour(), DEC);
   dataFile.print(':');
   dataFile.print(now.minute(), DEC);
   dataFile.print(':');
   dataFile.print(now.second(), DEC);
   dataFile.println();
   dataFile.println(); //Extra new line for increased readability on the SD Card
} //end of getClockData()
//Helper method for reading the barometer
unsigned int readRegister(byte thisRegister)
{
    unsigned int result = 0;   // result to return
    digitalWrite(chipSelectPin, LOW);
    delay(10);
    SPI.transfer(thisRegister);
    result = SPI.transfer(0x00);
    digitalWrite(chipSelectPin, HIGH);
    return(result);
} // end of readRegister()
void getBarometerData()
{
    float A0_;
    float B1_;
    float B2_;
    float C12_;

    SPI.begin();   

    // Reading registers that contain the chip-unique parameters to do the math  
    // These constants are defined at the top of this program and are special for the barometer only
    unsigned int A0H = readRegister(A0MSB);
    unsigned int A0L = readRegister(A0LSB);
    A0_ = (A0H << 5) + (A0L >> 3) + (A0L & 0x07) / 8.0;

    unsigned int B1H = readRegister(B1MSB);
    unsigned int B1L = readRegister(B1LSB);
    B1_ = ( ( ( (B1H & 0x1F) * 0x100)+B1L) / 8192.0) - 3 ;

    unsigned int B2H = readRegister(B2MSB);
    unsigned int B2L = readRegister(B2LSB);
    B2_ = ( ( ( (B2H - 0x80) << 8) + B2L) / 16384.0 ) - 2 ;

    unsigned int C12H = readRegister(C12MSB);
    unsigned int C12L = readRegister(C12LSB);
    C12_ = ( ( ( C12H * 0x100 ) + C12L) / 16777216.0 );

    digitalWrite(chipSelectPin, LOW);
    delay(3);
    SPI.transfer(0x24);
    SPI.transfer(0x00);
    digitalWrite(chipSelectPin, HIGH);
    delay(3);
    digitalWrite(chipSelectPin, LOW);
    SPI.transfer(PRESH);
    unsigned int presH = SPI.transfer(0x00);
    delay(3);
    SPI.transfer(PRESL);
    unsigned int presL = SPI.transfer(0x00);
    delay(3);
    SPI.transfer(TEMPH);
    unsigned int tempH = SPI.transfer(0x00);
    delay(3);
    SPI.transfer(TEMPL);
    unsigned int tempL = SPI.transfer(0x00);
    delay(3);
    SPI.transfer(0x00);
    delay(3);
    
    digitalWrite(chipSelectPin, HIGH);
  
    unsigned long press = ((presH *256) + presL)/64;
    unsigned long temp  = ((tempH *256) + tempL)/64;
  
    float pressure = A0_+(B1_+C12_*temp)*press+B2_*temp;
    float preskPa = pressure*  (65.0/1023.0)+50.0; //Converting pressure to kilopascals
    if (preskPa != 50.00)
    {
        dataFile.print("Barometer Presure (kPa): ");
        dataFile.println(preskPa);
    }
} //end of getBarometerData()
void getAccelerometerData()
{
  int xPin = 8; //Pin for x axis direction
  int yPin = 9; //Pin for y axis direction
  int zPin = 10; //Pin for z axis direction
  
  /*
    The minimum and maximum values that came from the accelerometer. These constants vary on each Arduino,
    so expect to change it based on your own accelerometer. To calculate these values, analogRead each of
    the pins and see what values are outputted to the Serial Monitor.
  */
  int minValx = 333;
  int maxValx = 438;
  int minValy = 331;
  int maxValy = 664;
  int minValz = 410;
  int maxValz = 508;

  //These variables will hold the calculated values
  double x;
  double y;
  double z;

  //Reading raw data from each of the pins
  int xRead = analogRead(xPin);
  int yRead = analogRead(yPin);
  int zRead = analogRead(zPin);

  //In case we get even higher readings from the accelerometer
  if (xRead > maxValx)
  {
    maxValx = xRead;
  }
  if (yRead > maxValy)
  {
    maxValy = yRead;
  }
  if (zRead > maxValz)
  {
    maxValz = zRead;
  }
  
  //convert read values to degrees -90 to 90 - Needed for atan2
  int xAng = map(xRead, minValx, maxValx, -90, 90);
  int yAng = map(yRead, minValy, maxValy, -90, 90);
  int zAng = map(zRead, minValz, maxValz, -90, 90);

  dataFile.print("Orientation: ");
  dataFile.print(xAng);
  dataFile.print(" ");
  dataFile.print(yAng);
  dataFile.print(" ");
  dataFile.print(zAng);
  dataFile.println();
} //end of getAccelerometerData()
void getLuminosityData()
{
  TSL2561 tsl(TSL2561_ADDR_FLOAT); 

   // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
  tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);  // longest integration time (dim light)
  
  // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 13-402 milliseconds! Uncomment whichever of the following you want to read
  uint16_t x = tsl.getLuminosity(TSL2561_VISIBLE);     
  //uint16_t x = tsl.getLuminosity(TSL2561_FULLSPECTRUM);
  //uint16_t x = tsl.getLuminosity(TSL2561_INFRARED);
  
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF; 
  dataFile.print("IR: "); dataFile.print(ir);   dataFile.print("\t");
  dataFile.print("Full: "); dataFile.print(full);   dataFile.print("\t");
  dataFile.print("Visible: "); dataFile.print(full - ir);   dataFile.print("\t");
  dataFile.print("Lux: "); dataFile.println(tsl.calculateLux(full, ir));
  
  
} //End of getLuminosityData()
void getHumidityData()
{
  DHT dht(DHTPIN, DHTTYPE);
  dht.begin();
  // Reading temperature or humidity takes about 250 milliseconds
  // Sensor readings may also be up to 2 seconds because this is a digital sensor
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  dataFile.print("Humidity: ");
  dataFile.println(h);
  dataFile.print("Temperature (Celsius): ");
  dataFile.println(t);
  dataFile.print("Temperature (Fahrenheit): ");
  t = (t * (1.8)) + 32;
  dataFile.println(t);
} //End of getHumidityData()

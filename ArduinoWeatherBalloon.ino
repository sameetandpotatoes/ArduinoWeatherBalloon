/* All Imported Libraries */
#include <SD.h>
#include "DHT.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561.h>
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

//Real time clock variable
RTC_DS1307 RTC;

int previousMillis = 0;
int interval = 1200000; //Time in milliseconds between writing to the SD card.

File dataFile; //Name of file we are writing 

void setup()
{
  SD.begin(10,11,12,13); //The SD Shield uses 4 pins for the Arduino Mega
  
  pinMode(shutDown, OUTPUT);
  pinMode(chipSelectPin, OUTPUT);
  
  digitalWrite(chipSelectPin, HIGH);
  digitalWrite(shutDown, HIGH);
  
  Wire.begin();
  RTC.begin();
  
  if (!RTC.isrunning())
  {
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
}

void loop()
{
  int currentMillis = millis();
  if (currentMillis - previousMillis > interval) //Writing to SD card only after the interval of time has passed
  {      
    dataFile = SD.open("datalog.txt", FILE_WRITE); //Opening and closing the SD Card once, instead of after reading each sensor
    int i = 0;    
    for (i = 0; i < 9; i++)
    {
      switch (i)
      {
        case 0: 
          getGeigerData();  
          break;
        case 1: 
          getGas1Data();
          break;
        case 2: 
          getGas2Data();
          break;
        case 3: 
          getGPSData();
          break;
        case 4: 
          getHumidityData();
          break;
        case 5: 
          getBarometerData();
          break;
        case 6: 
          getAccelerometerData(); 
          break;
        case 7: 
          getLuminosityData(); 
          break;
        case 8: 
          getClockData(); 
          break;
        }
    }
    dataFile.close(); //Closing data file
    previousMillis = currentMillis;   
  }
}

void getGeigerData()
{
  dataFile.print("Geiger: ");
  dataFile.println(analogRead(13));
}
void getGas1Data()
{
  dataFile.print("Inside Gas Sensor: ");
  dataFile.println(analogRead(14));
}
void getGas2Data()
{
  dataFile.print("Outside Gas Sensor: ");
  dataFile.println(analogRead(15));
}
void getGPSData()
{  
  Adafruit_GPS GPS(&Serial1);
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // Setting the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate 
 
  if (GPS.newNMEAreceived()) // if a sentence is received, we can check the checksum, parse it...
  {
     GPS.lastNMEA();
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }    
  dataFile.print("Fix: "); dataFile.print((int)GPS.fix);
  dataFile.print(" quality: "); dataFile.println((int)GPS.fixquality); 
  if (GPS.fix)
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
}
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
   dataFile.println(); //Extra new line for increased readability
}
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
}
void getBarometerData()
{
    float A0_;
    float B1_;
    float B2_;
    float C12_;

    SPI.begin();   

    // read registers that contain the chip-unique parameters to do the math
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
    float preskPa = pressure*  (65.0/1023.0)+50.0;
    dataFile.print("Barometer Presure (pa): ");
    dataFile.println(preskPa);
}
void getAccelerometerData()
{
  const int xPin = 8; //Pin for x axis direction
  const int yPin = 9; //Pin for y axis direction
  const int zPin = 10; //Pin for z axis direction
  
  /*
    The minimum and maximum values that came from
    the accelerometer. These
    constants vary from person to person, so expect
    to change it based on your own accelerometer.
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
}

void getLuminosityData()
{
  Adafruit_TSL2561 tsl = Adafruit_TSL2561(TSL2561_ADDR_FLOAT, 12345);

  tsl.enableAutoGain(true);          /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      // fast but low resolution
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  // medium resolution and speed
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  // 16-bit data but slowest conversions

  /* Get a new sensor event */
  sensors_event_t event;
  tsl.getEvent(&event);
  
  dataFile.print("Luminosity: ");
  dataFile.println(event.light);
}

void getHumidityData()
{
  DHT dht(DHTPIN, DHTTYPE);
  dht.begin();
  // Reading temperature or humidity takes about 250 milliseconds
  // Sensor readings may also be up to 2 seconds
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  dataFile.print("Humidity: ");
  dataFile.println(h);
  dataFile.print("Temperature: ");
  dataFile.println(t);
}

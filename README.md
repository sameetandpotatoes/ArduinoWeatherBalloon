# Purpose
  The goal of this project is to launch a balloon containing a payload that uses various sensors powered by an Arduino Mega to record variables from the environment at 100,000 feet. The program writes the outputs of all of these sensors to an 8 GB SD Card on the Arduino Mega itself. The Arduino Mega uses a solar panel to recharge its battery. 

# Hardware Parts List
- [Arduino Mega](http://www.adafruit.com/products/191)
- [Data Logger Shield Kit](http://www.adafruit.com/products/1141)
- [Geiger Counter](https://www.sparkfun.com/products/11345)
- [Gas Sensors](https://www.sparkfun.com/products/10916)
- [GPS Flora](http://www.adafruit.com/products/1059)
- [Humidity and Temperature Sensor](http://www.adafruit.com/products/386)
- [Barometer Sensor](https://www.sparkfun.com/products/9721)
- [Accelerometer Sensor](http://www.adafruit.com/products/1120)
- [Luminosity Sensor](http://www.adafruit.com/products/439)
- Real Time Clock (comes with Data Logger Shield Kit)

# Wiring

![ScreenShot](https://raw.github.com/sameetandpotatoes/ArduinoWeatherBalloon/master/Pictures/IMG_0010.JPG)
![ScreenShot](https://raw.github.com/sameetandpotatoes/ArduinoWeatherBalloon/master/Pictures/IMG_0024.JPG)
![ScreenShot](https://raw.github.com/sameetandpotatoes/ArduinoWeatherBalloon/master/Pictures/IMG_0026.JPG)
![ScreenShot](https://raw.github.com/sameetandpotatoes/ArduinoWeatherBalloon/master/Pictures/IMG_0029.JPG)
![ScreenShot](https://raw.github.com/sameetandpotatoes/ArduinoWeatherBalloon/master/Pictures/IMG_0039.JPG)

# Schematic
![ScreenShot](https://raw.github.com/sameetandpotatoes/ArduinoWeatherBalloon/master/Breadboard.png)
![ScreenShot](https://raw.github.com/sameetandpotatoes/ArduinoWeatherBalloon/master/Schematic.png)

# Sample Data (from DATALOG.txt file on SD Card)

Geiger: 22
Outside Gas Sensor: 633
Fix: 0 quality: 0 (no GPS signal when we tried)
Humidity: 39.00
Temperature (Celsius): 27.00
Temperature (Fahrenheit): 80.60
Barometer Presure (kPa): 99.04
Orientation: 90 11 90
IR: 28    Full: 59    Visible: 31    Lux: 15
2013/7/18 10:49:14


# Extra

This project was added on [this repository](https://github.com/erichaddleton/porkycam) for a weather balloon that takes pictures every ten seconds.

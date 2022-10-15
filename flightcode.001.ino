#include <SoftwareSerial.h>
#include <Arduino_LSM6DSOX.h>
#include <BMP388_DEV.h> 

//sensors
float temperature, pressure, altitude;
BMP388_DEV bmp388;
SoftwareSerial SDCard (0, 1):

void setup() 
{
  Serial.begin(115200);
  SDCard.begin(115200);
  IMU.begin();
  bmp388.begin();
  bmp388.setTimeStandby(TIME_STANDBY_1280MS);
  bmp388.startNormalConversion();
  SDCard.println("temperature (C), pressure (hpa), altitude (m), Gyro (x), Gyro (y), Gyro (z)");
}

void loop() 
{
  if (bmp388.getMeasurements(temperature, pressure, altitude))  //bmp388 data
  {
    SDCard.print(temperature); SDcard.print(", ");   
    SDCard.print(pressure);  SDcard.print(", ");    
    SDCard.print(altitude);  SDcard.print(", "); 
  }
  
  float x, y, z;
  if (IMU.gyroscopeAvailable()) {                               //IMU data (in deg/sec)
    
    IMU.readGyroscope(x, y, z);

    SDCard.print(x); SDcard.print(", "); 
    SDCard.print(y); SDcard.print(", "); 
    SDCard.println(z); SDcard.println(", "); 
  }

  delay(250);
}

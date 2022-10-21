/*RCS Team 2 Program
 * IF YOU ARE TESTING BE SURE TO COMMENT OUT PARTS OF CODE YOU ARE NOT USING
 * IF USING THE SDCard MONITOR TO READ DATA, CHANGE ALL SDCard.PRINT TO SDCard.PRINT OTHERWISE YOU WILL NOT SEE ANYTHING
 */

#include <SoftwareSerial.h>
#include <Arduino_LSM6DSOX.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <BMP388_DEV.h>   

float temperature, pressure, altitude;            // Create the temperature, pressure and altitude variables
BMP388_DEV bmp388;    
float startingaltitude; 
float maxalt;
float seconds;

SoftwareSerial SDCard(0, 1);
SFE_UBLOX_GNSS myGNSS;

int const ledPin = 8;                                            //New Energy Led Pin
int const SolenoidClockwise = 10;                                 //pin for Solenoid Clockwise
int const SolenoidCounterClockwise = 5;                          //pin for Solenoid Counter Clockwise
int const cameraPicture = 14;                                     //pin for camera picture
int const cameraVideo = 12;                                       //pin for camera video
float altForStabilization = 25000;                               //altitude that satbilization will start
float gyroscopestop = 0.5f;                                      //gyroscope limiter to detect when it has stopped moving
float maxDegSec = 30;                                            //target degrees/second of spin, enable solenoids when passed in the positive or negative direction
float timeBetweenFires = 1;
int ledcounter = 4;                                              //the amount of time between blinks (ledcounter/4) = the amount of blinks per second
int packetcount;                                                 //amount of packets we've collected
int launchState;                                                 //what state the vehicle is in
bool stabilizealt;                                               //bool to signal if the altitude has been passed to start stabilizing
bool ascending;                                                  //bool signaling if the vehicle is ascending or not
bool stoppedVideo;
bool firstCameraStart;
long lastTime = 0;                                               //long for ZOE (I don't actually know how this works, it was just in the example)
long pictureTime = 0;

void setup() {
  
  Serial.begin(115200);                                          //baud rate is 115200, if something doesn't work, check baud first
  SDCard.begin(115200);                                          //IF TESTING ON COMPUTER CHANGE ALL SDCard.PRINT TO SDCard.PRINT AND COMMENT OUT PARTS YOU DON'T NEED AT THE MOMENT
  Wire.begin(); 
  IMU.begin();
  //myGNSS.setI2COutput(COM_TYPE_UBX);
  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  bmp388.begin();                                 // Default initialisation, place the BMP388 into SLEEP_MODE 
  bmp388.setTimeStandby(TIME_STANDBY_1280MS);     // Set the standby time to 1.3 seconds
  bmp388.startNormalConversion();        
  
  pinMode(SolenoidClockwise, OUTPUT);
  pinMode(SolenoidCounterClockwise, OUTPUT);
  pinMode(cameraPicture, OUTPUT);
  pinMode(cameraVideo, OUTPUT);
  pinMode(ledPin, OUTPUT);  
  stabilizealt = false;
  stoppedVideo = false;
  //videoOnMain = false;
  firstCameraStart = false;
  packetcount = 0; launchState = 0; pictureTime = millis();
  /*
  while (!myGNSS.begin() || !IMU.begin() || !bmp388.begin()) {                                  //Connect to the u-blox module using Wire port
    SDCard.println(F("sensors did not work you idiot. Get the god damn wiring right once in a while. Freezing."));
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(50);
    while (1);
  }
  */
  SDCard.println("TEAM_ID, PACKET_COUNT, MISSION_TIME, SW_STATE, CAM_STATE, REAL_TIME, LAT, LONG, SIV, PRESSURE, ALTITUDE, TEMP, ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z");
  //startingaltitude = bmp388.getAltitude();
}

void loop() {                                                   //loop function, make sure last print statement is println, but no other print statements are

//recording and logging data -------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS TESTING

  SDCard.print("RCS Team 2 (HMMRHD), ");           //can combine if we want
  SDCard.print(packetcount); SDCard.print(", ");
  SDCard.print(seconds); SDCard.print(", ");
  SDCard.print(launchState); SDCard.print(", ");
  SDCard.print("Camera state"); SDCard.print(", ");

  if (bmp388.getMeasurements(temperature, pressure, altitude))    // Check if the measurement is complete
  {
    SDCard.print(temperature); SDCard.print(", ");                    // Display the results    
    SDCard.print(pressure); SDCard.print(", ");    
    SDCard.print(altitude); SDCard.print(", "); 
  } else { SDCard.print(", , , "); }
  
  float ax, ay, az;
  if (IMU.accelerationAvailable()) {                            //IMU data acceleration
    IMU.readAcceleration(ax, ay, az);

    SDCard.print(ax); SDCard.print(", ");
    SDCard.print(ay); SDCard.print(", ");
    SDCard.print(az); SDCard.print(", ");
  } else { SDCard.print(", , , "); }

  if (millis() - lastTime > 1000)                               //GPS Data (Lat, Long, SIV)
  {
    lastTime = millis();

    /*SDCard.print(myGNSS.getHour()); SDCard.print(":");
    SDCard.print(myGNSS.getMinute()); SDCard.print(":");
    SDCard.print(myGNSS.getSecond()); SDCard.print(", ");
    
    long latitude = myGNSS.getLatitude();
    SDCard.print(latitude); SDCard.print(", ");

    long longitude = myGNSS.getLongitude();
    SDCard.print(longitude); SDCard.print(", ");

    byte SIV = myGNSS.getSIV();
    SDCard.print(SIV); SDCard.print(", ");*/
  } //else { SDCard.print(", , , "); }
  
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {                               //IMU data gyro(in deg/sec)
    
    IMU.readGyroscope(gx, gy, gz);

    SDCard.print(gx); SDCard.print(", "); 
    SDCard.print(gy); SDCard.print(", "); 
    SDCard.print(gz); SDCard.println(", ");
  } else { SDCard.println(", , , "); }
  
  if(altitude > maxalt){ maxalt = altitude; }

//determine state of vehicle ------------------------------------------------------------------------------------------------------------------------------------------------ NEEDS TESTING

  if(launchState == 0 && ((altitude+25) > startingaltitude)) { 
    launchState = 1;                                                                                            //ascending
  }
  if(launchState == 1 && (altitude > altForStabilization)) { 
    launchState = 2;                                                                                            //control active
  }
  if(launchState == 2 && (maxalt < (altitude+50))) { 
    launchState = 3;                                                                                            //desending
  }
  if(launchState == 3 && (gy <= gyroscopestop) && (gx <= gyroscopestop) && (gz <= gyroscopestop)) {
    launchState = 4;                                                                                            //landed
  }

//stabilization -------------------------------------------------------------------------------------------------------------------------------------------------------------  NEEDS TESTING *change to ||
/*
  if(altitude >= altForStabilization) {stabilizealt = true);                               //turn on at altitude
  else { stabilizealt = false; }
  
  if(gy >= maxDegSec && stabilizealt && ((timeBetweenFires/4) >= 1)) {                       //determine if we need to turn on Clockwise Solenoid
    digitalWrite(SolenoidClockwise, HIGH);
    timeBetweenFires = 0;
  } else { digitalWrite(SolenoidClockwise, LOW); }
 
  if(gy <= -maxDegSec && stabilizealt && ((timeBetweenFires/4) >= 1)) {                      //determine if we need to turn on Counter Clockwise Solenoid
    digitalWrite(SolenoidCounterClockwise, HIGH);
    timeBetweenFires = 0;
  } else { digitalWrite(SolenoidCounterClockwise, LOW); }

  timeBetweenFires += 1;
*/
//camera -------------------------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS TESTING

  /*if(!firstCameraStart) {                                     //start video on startup
    digitalWrite(cameraVideo, HIGH);
    delay(150);
    digitalWrite(cameraVideo, LOW);
    firstCameraStart = true;
  }*/
/*
 if(launchState != 4 || launchState != 0) {                                      //take pictures anytime vehicle has not landed
    if((pictureTime) >= 40) { 
      digitalWrite(cameraPicture, HIGH);
      delay(50);
      digitalWrite(cameraPicture, LOW);
      pictureTime = 0;
    } else { 
      pictureTime += 1;
      delay(50);
    }
  }
*/
  if(launchState != 4) {  

  }
  
    digitalWrite(cameraPicture, HIGH);
    delay(5000);
    digitalWrite(cameraPicture, LOW);
    delay(2000);
  
  int currentTime = int(millis());
  if (currentTime - pictureTime >= 2000){ 
    pictureTime = int(millis());
  }
  
  /*if(launchState == 5) { 
    if(!stoppedVideo) {
      digitalWrite(cameraVideo, HIGH);
      delay(550);
      digitalWrite(cameraVideo, LOW);
      stoppedVideo = true;
    }
  }*/

//extra ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS TESTING

  if(ledcounter >= 8) {                                        //blink leds every 1/4 second, if ledcounter is increased, then time between blinks increases (ex. ledcounter = 8 then leds will blink every 2 seconds
    digitalWrite(ledPin, HIGH);
    ledcounter = 1;
  } else { 
    digitalWrite (ledPin, LOW);
    ledcounter += 1;
  }

  packetcount += 1;
  seconds = (millis()/1000);
  delay(200);                                               //delay 0.20 sec - ends up being 0.25 with the picture delay
}

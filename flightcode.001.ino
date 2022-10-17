/*RCS Team 2 Program
 * IF YOU ARE TESTING BE SURE TO COMMENT OUT PARTS OF CODE YOU ARE NOT USING
 * IF USING THE SERIAL MONITOR TO READ DATA, CHANGE ALL Serial.PRINT TO SERIAL.PRINT OTHERWISE YOU WILL NOT SEE ANYTHING
 */

#include <SoftwareSerial.h>
#include <Arduino_LSM6DSOX.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <BMP388_DEV.h>                           // Include the BMP388_DEV.h library

float temperature, pressure, altitude;            // Create the temperature, pressure and altitude variables
BMP388_DEV bmp388;

float flighttime, startingaltitude, maxalt; //sensor values, do not need to be assigned
SoftwareSerial SDCard(0, 1);
SFE_UBLOX_GNSS myGNSS;

int const ledPin = 3;                                            //New Energy Led Pin
int const SolenoidClockwise = 4;                                 //pin for Solenoid Clockwise
int const SolenoidCounterClockwise = 5;                          //pin for Solenoid Counter Clockwise
int const camera = 6;                                            //pin for camera
float altForStabilization = 25000;                               //altitude that satbilization will start
float gyroscopestop = 0.5f;                                      //gyroscope limiter to detect when it has stopped moving
float maxDegSec = 30;                                            //target degrees/second of spin, enable solenoids when passed in the positive or negative direction
int ledcounter = 4;                                              //the amount of time between blinks (ledcounter/4) = the amount of blinks per second
int pictureOnTime = 50;                                          //the delay between signals for picture taking
int videoOnTime = 200;                                           //the delay between signals for starting and stopping video
int videoWaitTime = 1200;                                        //the length of the video we want to record (videoWaitTime/240) = the amount of time in minutes that we are recording
int packetcount;                                                 //amount of packets we've collected
int launchstate;                                                 //what state the vehicle is in
bool stabilize;                                                  //bool to enable stabilization
bool stabilizealt;                                               //bool to signal if the altitude has been passed to start stabilizing
bool ascending;                                                  //bool signaling if the vehicle is ascending or not
bool pictureOn;                                                  //bool to take pictures
bool videoOnUpdate;                                              //bool to take videos
bool videoOnMain;                                                //determines if we are starting a new video or stopping a old one
long lastTime = 0;                                               //long for ZOE (I don't actually know how this works, it was just in the example)

void setup() {
  
  Serial.begin(115200);                                          //baud rate is 115200, if something doesn't work, check baud first
  SDCard.begin(115200);                                          //IF TESTING ON COMPUTER CHANGE ALL Serial.PRINT TO SERIAL.PRINT AND COMMENT OUT PARTS YOU DON'T NEED AT THE MOMENT
  Wire.begin(); 
  bmp388.begin();                                 // Default initialisation, place the BMP388 into SLEEP_MODE 
  bmp388.setTimeStandby(TIME_STANDBY_1280MS);     // Set the standby time to 1.3 seconds
  bmp388.startNormalConversion();   
  
  pinMode(SolenoidClockwise, OUTPUT);
  pinMode(SolenoidCounterClockwise, OUTPUT);
  pinMode(camera, OUTPUT);
  pinMode(ledPin, OUTPUT);  
  stabilize = false;
  stabilizealt = false;
  videoOnMain = false;
  flighttime = 0; packetcount = 0; launchstate = 0;
  
  IMU.begin();                                                  //begin sensors
  //startingaltitude = bmp388.getMeasurements(altitude);
  if (myGNSS.begin() == false) { //Connect to the u-blox module using Wire port
    Serial.println(F("GPS did not work you idiot. Freezing."));
    digitalWrite(ledPin, HIGH);
    while (1);
  }
  while(!IMU.begin()){
    Serial.println("sensor initiation failed");
    digitalWrite(ledPin, HIGH);
    while (1);
  }  
  
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  
  Serial.println("TEAM_ID, MISSION_TIME, PACKET_COUNT, SW_STATE, CAM_STATE, REAL_TIME, LAT, LONG, SIV, PRESSURE, ALTITUDE, TEMP, ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z");
}

void loop() {                                                   //loop function, make sure last print statement is println, but no other print statements are

//recording and logging data -------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS TESTING

  Serial.print("RCS Team 2 (HMMRHD), ");           //can combine if we want
  Serial.print(flighttime + ", ");                 //record time into flight
  Serial.print(packetcount + ", ");
  Serial.print(launchstate + ", ");
  Serial.print("Camera state" + ", ");

  if (millis() - lastTime > 1000)                               //GPS Data (Lat, Long, SIV)
  {
    lastTime = millis();

    Serial.print(myGNSS.getHour() + ":");
    Serial.print(myGNSS.getMinute() + ":");
    Serial.print(myGNSS.getSecond() ", ");
    
    long latitude = myGNSS.getLatitude();
    Serial.print(latitude + ", "); 

    long longitude = myGNSS.getLongitude();
    Serial.print(longitude + ", ");

    byte SIV = myGNSS.getSIV();
    Serial.print(SIV + ", ");
  } else { Serial.print("-, -, -, "); }

  if (bmp388.getMeasurements(temperature, pressure, altitude))    // Check if the measurement is complete
  {
    Serial.print(temperature + ", ");
    Serial.print(pressure + ", ");    
    Serial.print(altitude + ", ");
  } else { Serial.print("-, -, -, "); }
  
  float ax, ay, az;
  if (IMU.accelerationAvailable()) {                            //IMU data acceleration
    IMU.readAcceleration(ax, ay, az);

    Serial.print(ax + ", ");
    Serial.print(ay + ", ");
    Serial.print(az + ", ");
  } else { Serial.print("-, -, -, "); }
  
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {                               //IMU data gyro(in deg/sec)
    
    IMU.readGyroscope(gx, gy, gz);

    Serial.print(gx + ", "); 
    Serial.print(gy + ", "); 
    Serial.print(gz + ", ");
  } else { Serial.print("-, -, -, "); }
  
  if(altitude > maxalt){ maxalt = altitude; }

//determine state of vehicle ------------------------------------------------------------------------------------------------------------------------------------------------ NEEDS TESTING

  if(launchstate == 0 && (altitude+25) > startingalt) { 
    launchstate = 1;                                                                                            //ascending
  }
  if(launchstate == 1 && altitude > altforstabilization) { 
    launchstate = 2;                                                                                            //control active
  }
  if(launchstate == 2 && !stabilization) { 
    launchstate = 3;                                                                                            //control deactive
  }
  if(launchstate == 3 && stabilization) { 
    launchstate = 2;                                                                                            //control deactive
  }
  if(launchstate == 2 && maxalt < (altitude+25)) { 
    launchstate = 4;                                                                                            //desending
  }
  if(launchstate == 3 && maxalt < (altitude+25)) { 
    launchstate = 4;                                                                                            //desending
  }
  if(launchstate == 4 && gy <= gyroscopestop && gx <= gyroscopestop && gz <= gyroscopestop) {
    launchstate = 5;                                                                                            //landed
  }

//stabilization -------------------------------------------------------------------------------------------------------------------------------------------------------------  NEEDS TESTING *change to ||

  if(altitude >= altForStabilization) {stabilizealt = true);                               //turn on at altitude
  else { stabilizealt = false; }
  
  if(gy >= maxDegSec && stabilizealt) {                                                    //determine if we need to turn on Clockwise Solenoid
    digitalWrite(SolenoidClockwise, HIGH);
  } else { digitalWrite(SolenoidClockwise, LOW); }
 
  if(gy <= -maxDegSec && stabilizealt) {                                                   //determine if we need to turn on Counter Clockwise Solenoid
    digitalWrite(SolenoidCounterClockwise, HIGH);
  } else { digitalWrite(SolenoidCounterClockwise, LOW); }


//camera -------------------------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS WORK

  unsigned long currentMillis = millis();

  if(altitude <= 15000 && altitude >= (startingaltitude + 100) && launchstate == 1){      //pictures only this should take a picture everytime loop plays
    if (currentMillis - previousMillis >= pictureOnTime) {
      previousMillis = currentMillis;

     if (pictureOn == LOW) {
        pictureOn = HIGH;
      } else {
        pictureOn = LOW;
      }
    digitalWrite(camera, picutreOn);
    
  } else if (altitude >= 15000 && launchstate == 1){                                      //video and pictures - so far is just video (same thing as below)
     if(videoWaitTime >= 1200) {
        if (currentMillis - previousMillis >= videoOnTime) {
        previousMillis = currentMillis;

        if (videoOnUpdate == LOW) {
            videoOnUpdate = HIGH;
          } else {
            videoOnUpdate = LOW;
          }
          digitalWrite(camera, videoOnUpdate);
          if(videoOnMain) {
            videoWaitTime = 1175;
          } else { 
            videoWaitTime = 0;
          }
          videoOnMain = !videoOnMain
        }    
     }
     videoWaitTime += 1;
  } else if (altitude >= altForStabilization && launchstate != 1) {                       //video only this should loop through taking a new video every 5 minutes
      if(videoWaitTime >= 1200) {
        if (currentMillis - previousMillis >= videoOnTime) {
        previousMillis = currentMillis;

        if (videoOnUpdate == LOW) {
            videoOnUpdate = HIGH;
          } else {
            videoOnUpdate = LOW;
          }
          digitalWrite(camera, videoOnUpdate);
          if(videoOnMain) {
            videoWaitTime = 1175;
          } else { 
            videoWaitTime = 0;
          }
          videoOnMain = !videoOnMain
        }    
     }
     videoWaitTime += 1;
  }

//extra ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS TESTING

  if(ledcounter >= 4) {                                        //blink leds every 1/4 second, if ledcounter is increased, then time between blinks increases (ex. ledcounter = 8 then leds will blink every 2 seconds
    digitalWrite(ledPin, HIGH);
    ledcounter = 1;
  } else { 
    digitalWrite (ledPin, LOW);
    ledcounter += 1;
  }

  packetcount += 1;
  flighttime += 0.25;                                         //update to millis
  delay(250);                                                 //delay 0.25 sec (maybe change this to be a milli delay like the picture
}

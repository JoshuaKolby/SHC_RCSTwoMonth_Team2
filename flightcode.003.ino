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
float startingaltitude; 
float maxalt;
float seconds;
                                     //sensor values, do not need to be assigned
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
long pictureTime = 0;
float timeBetweenFires = 1;
int ledcounter = 4;                                              //the amount of time between blinks (ledcounter/4) = the amount of blinks per second
int packetcount;                                                 //amount of packets we've collected
int launchState;                                                 //what state the vehicle is in
bool stabilizealt;                                               //bool to signal if the altitude has been passed to start stabilizing
bool ascending;                                                  //bool signaling if the vehicle is ascending or not
bool stoppedVideo;
bool firstCameraStart;
long lastTime = 0;                                               //long for ZOE (I don't actually know how this works, it was just in the example)
long latitude;
long longitude;
bool videoOnMain;
byte SIV;

  

//Variables that need to be output to the card
String TEAM_ID = "RCS TEAM 2 (HMMRHD)";
float last_sec = millis();
float REAL_TIME_S_2 = 00.00;
float REAL_TIME_M_2 = 00.00;
float REAL_TIME_H_2 = 00.00;
int PACKET_COUNT = 1;
const char* SW_STATE = "N/A";
const char* CAM_STATE = "N/A";
int TEMP = 0;
float ACC_X = 00.00;
float ACC_Y = 00.00;
float ACC_Z = 00.00;
float GYRO_X = 00.00;
float GYRO_Y = 00.00;
float GYRO_Z = 00.00;
float REAL_TIME_S = 00.00;
float REAL_TIME_M = 00.00;
float REAL_TIME_H = 00.00;
float PRESSURE = 00.00;
float LAT = 00.00;
float LONG = 00.00;
int packtime = 0;
float LED_TIME = millis();




void setup() {
  
  Serial.begin(115200);                                          //baud rate is 115200, if something doesn't work, check baud first
  SDCard.begin(115200);                                          //IF TESTING ON COMPUTER CHANGE ALL Serial.PRINT TO SERIAL.PRINT AND COMMENT OUT PARTS YOU DON'T NEED AT THE MOMENT
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
  packetcount = 0; 
  packtime = millis();
  
                                                 //begin sensors
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
  
  SDCard.println("TEAM_ID, PACKET_COUNT, SW_STATE, CAM_STATE, ALTITUDE, TEMP, ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z, REAL_TIME_H:REAL_TIME_M:REAL_TIME_S, PRESSURE, LAT, LONG, SIV");
  //startingaltitude = bmp388.getMeasurements(altitude);

}
void loop() {                           //loop function, make sure last print statement is println, but no other print statements are

//recording and logging data -------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS TESTING
  
  REAL_TIME_S = myGNSS.getSecond();
  REAL_TIME_M = myGNSS.getMinute();
  REAL_TIME_H = myGNSS.getHour();

  if (millis() - lastTime >= 200)                               //GPS Data
  {                                                             //Checks the altitude, temperature, pressure, latitude, longitude, and SIV
    lastTime = millis();
    atplls();

   }

  
  float ax, ay, az;
  if (IMU.accelerationAvailable()) {                            //IMU data acceleration
    IMU.readAcceleration(ax, ay, az);

    ACC_X = ax;
    ACC_Y = ay;
    ACC_Z = az;
  
  } else { 
    ACC_X = 0;
    ACC_Y = 0;
    ACC_Z = 0;
   }
  
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {                               //IMU data gyro(in deg/sec)
    
    IMU.readGyroscope(gx, gy, gz);

    GYRO_X = gx;
    GYRO_Y = gy;
    GYRO_Z = gz; 
   
  } else { 
    GYRO_X = 0;
    GYRO_Y = 0;
    GYRO_Z = 0; 
    }
  
  if(altitude > maxalt){ 
    maxalt = altitude; 
    }

//determine state of vehicle ------------------------------------------------------------------------------------------------------------------------------------------------ NEEDS TESTING
  
  if(launchState == 0 && ((altitude+30) > startingaltitude)) { 
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
  if(altitude >= altForStabilization) {stabilizealt = true;}                               //turn on at altitude
  else { stabilizealt = false;}
  
  if(gy >= maxDegSec && stabilizealt && (timeBetweenFires/4) >= 1) {                       //determine if we need to turn on Clockwise Solenoid
    digitalWrite(SolenoidClockwise, HIGH);
    timeBetweenFires = 0;                                                  
  } else { digitalWrite(SolenoidClockwise, LOW); }
 
  if(gy <= -maxDegSec && stabilizealt && (timeBetweenFires/4) >= 1) {                      //determine if we need to turn on Counter Clockwise Solenoid
    digitalWrite(SolenoidCounterClockwise, HIGH);
    timeBetweenFires = 0;
  } else { digitalWrite(SolenoidCounterClockwise, LOW); }

  timeBetweenFires += 1;
*/
//camera -------------------------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS TESTING

 if(launchState != 4) {   
  if (millis() - pictureTime >= 2000){  
    digitalWrite(cameraPicture, LOW);
    delay(50);
    digitalWrite(cameraPicture, HIGH);
    pictureTime = millis();
  }
 }

//extra ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS TESTING
if(millis() - LED_TIME >= 200) {
  
 if(ledcounter >= 8) {                                        //blink leds every 1/4 second, if ledcounter is increased, then time between blinks increases (ex. ledcounter = 8 then leds will blink every 2 seconds
    digitalWrite(ledPin, HIGH);
    ledcounter = 1;
    LED_TIME = millis();
  } else { 
    digitalWrite (ledPin, LOW);
    ledcounter += 1;
    LED_TIME = millis();
   
  }
}

  if (millis() - packtime >= 250){
    packtime = millis();
    send_Packet();
  }
                                       
}


//Function to Send Packets 
void send_Packet() {
  SDCard.println(String(TEAM_ID) + "," + String(millis()) + ", " + String(PACKET_COUNT) + ", " + String(SW_STATE) + ", " + String(CAM_STATE) + ", " + String(altitude) + ", " + String(TEMP) + ", " + String(ACC_X) + ", " + String(ACC_Y) + ", " + String(ACC_Z) + ", " + String(GYRO_X) + ", " + String(GYRO_Y) + ", " + String(GYRO_Z) + ", " + String(REAL_TIME_H) + ":" + String(REAL_TIME_M) + ":" + String(REAL_TIME_S) + ", " + String(PRESSURE) + ", " + String(LAT) + ", " + String(LONG) + ", " + String(SIV));
    packetcount += 1;
}

//Function to Pull Data Measurements from bmp388
void atplls() {
  if (bmp388.getMeasurements(temperature, pressure, altitude))    // Check if the measurement is complete
  {
    TEMP = temperature;
    PRESSURE = pressure;
    altitude = altitude;

  } else { 
    TEMP = 0;
    PRESSURE = 0;
    altitude = 0;
   }
  
  /*
  if (myGNSS.checkUblox()){                               //GPS Data (Lat, Long, SIV)
    
    latitude = myGNSS.getLatitude();       //Has been tested and works, leaving call but removing print
    longitude = myGNSS.getLongitude();       //Has been tested and works, leaving call but removing print
    SIV = myGNSS.getSIV();         //Has been tested and works, leaving call but removing print

  } else { 
    latitude = 0;
    longitude = 0;
    SIV = 0;
   }
*/

}








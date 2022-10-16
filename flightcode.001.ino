#include <SoftwareSerial.h>
#include <Arduino_LSM6DSOX.h>
#include <BMP388_DEV.h> 
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

//information
float temperature, pressure, altitude, flighttime, startingaltitude, maxalt;
BMP388_DEV bmp388;
SoftwareSerial SDCard (0, 1):
SFE_UBLOX_GNSS myGNSS;

int const LedPin = 3, SolenoidClockwise = 4, SolenoidCounterClockwise = 5, camera = 6;
float altForStabilization = 25000, gyroscopestop = 0.5f;
int ledcounter = 4, packetcount, launchstate;
bool stabilize, stabilizealt, assending;
long lastTime = 0; //for ZOE

void setup() 
{
  Serial.begin(115200);                                          //baud rate is 115200, if something doesn't work, check baud first
  SDCard.begin(115200);                                          //IF TESTING ON COMPUTER CHANGE ALL SDCARD.PRINT TO SERIAL.PRINT AND COMMENT OUT PARTS YOU DON'T NEED AT THE MOMENT
  Wire.begin();

  pinMode(SolenoidClockwise, OUTPUT);
  pinMode(SolenoidCounterClockwise, OUTPUT);
  pinMode(camera, OUTPUT);
  pinMode(LedPin, OUTPUT);  
  stabilize = false;
  stabilizealt = false;
  flighttime = 0; packetcount = 0; launchstate = 0;
  
  IMU.begin();                                                  //begin sensors
  bmp388.begin();
  bmp388.setTimeStandby(TIME_STANDBY_1280MS);
  bmp388.startNormalConversion();
  startingaltitude = bmp388.getMeasurements(altitude);
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port {
    SDCard.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  while(!IMU.begin() || !bmp388.begin()){
    SDcard.println("sensor initiation failed")
    digitalWrite(LedPin, HIGH);
  }  
  
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  
  SDCard.println("TEAM_ID, MISSION_TIME, PACKET_COUNT, SW_STATE, CAM_STATE, PRESSURE, ALTITUDE, TEMP, ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z, LAT, LONG, SIV");
}

void loop() {                                                   //loop function, make sure last print statement is println, but no other print statements are

//recording and logging data -------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS TESTING

  SDCard.print("RCS Team 2 (HMMRHD), ");
  SDcard.print(flighttime); SDcard.print(", ");                 //record time into flight
  SDCard.print(packetcount); SDcard.print(", ");
  SDCard.print(launchstate); SDcard.print(", ");
  SDCard.print("Camera state"); SDcard.print(", ");
  
  if (bmp388.getMeasurements(temperature, pressure, altitude))  //bmp388 data
  { 
    SDCard.print(pressure);  SDcard.print(", ");    
    SDCard.print(altitude);  SDcard.print(", ");
    SDCard.print(temperature); SDcard.print(", ");   
  } else { SDCard.print("-, -, -, ");

  float ax, ay, az;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);

    SDCard.print(ax); SDcard.print(", ");
    SDCard.print(ay); SDcard.print(", ");
    SDCard.print(az); SDcard.print(", ");
  }
  
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {                               //IMU data (in deg/sec)
    
    IMU.readGyroscope(gx, gy, gz);

    SDCard.print(gx); SDcard.print(", "); 
    SDCard.print(gy); SDcard.print(", "); 
    SDCard.println(gz); SDcard.print(", "); 
  } else { SDCard.print("-, -, -, ");
  
  if (millis() - lastTime > 1000)                               //GPS Data (Lat, Long, SIV)
  {
    lastTime = millis();
    
    long latitude = myGNSS.getLatitude();
    SDCard.print(latitude); SDcard.print(", "); 

    long longitude = myGNSS.getLongitude();
    SDCard.print(longitude); SDcard.print(", "); 

    byte SIV = myGNSS.getSIV();
    SDCard.println(SIV); SDcard.println(", "); 
  } else { SDCard.println("-, -, -, "); }

  if(altitude > maxalt){ maxalt = altitude; }

//determine state of vehicle ------------------------------------------------------------------------------------------------------------------------------------------------ NEEDS TESTING

if(launchstate == 0 && (altitude+25) > startingalt) { launchstate = 1; }                                      //assending
if(launchstate == 1 && altitude > altforstabilization) { launchstate = 2; }                                   //control active
if(launchstate == 2 && !stabilization) {launchstate = 3; }                                                    //control deactive
if(launchstate == 3 && stabilization) { launchstate = 2; }                                                    //control deactive
if(launchstate == 2 && maxalt < (altitude+25)) { launchstate = 4; }                                           //desending
if(launchstate == 3 && maxalt < (altitude+25)) { launchstate = 4; }                                           //desending
if(launchstate == 4 && gy <= gyroscopestop && gx <= gyroscopestop && gz <= gyroscopestop) {launchstate = 5;}  //landed

//stabilization -------------------------------------------------------------------------------------------------------------------------------------------------------------  NEEDS TESTING

  if(altitude >= altForStabilization) {stabilizealt = true);                 //turn on at altitude
  else { stabilizealt = false; }
  
  if(gy >= 30 && stabilizealt) {                               //determine if we need to turn on Clockwise Solenoid
    digitalWrite(SolenoidClockwise, HIGH);
  } else { digitalWrite(SolenoidClockwise, LOW); }
  
  if(gy <= -30 && stabilizealt) {                               //determine if we need to turn on Counter Clockwise Solenoid
    digitalWrite(SolenoidCounterClockwise, HIGH);
  } else { digitalWrite(SolenoidCounterClockwise, LOW); }


//camera -------------------------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS WORK

if(altitude <= 15000 && altitude >= (startingaltitude + 100) && launchstate == 1){ 
  //take pictures
} else if (altitude >= 15000 && launchstate == 1){
  //start record / picture process
} else if (altitude >= altForStabilization && launchstate != 1) {
  //record video for stabilization and while falling
}

//extra ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- NEEDS TESTING

  if(ledcounter == 4) {                                        //blink leds every 1/4 second, if ledcounter is increased, then time between blinks increases (ex. ledcounter = 8 then leds will blink every 2 seconds
    digitalWrite(LedPin, HIGH);
    ledcounter = 1;
  } else { 
    digitalWrite (LedPin, LOW);
    ledcounter += 1;
  }

  packetcount += 1;
  flighttime += 0.25;                                         //add 0.25 sec to flight time
  delay(250);                                                 //delay 0.25 sec
}

/* This project
 *
 *
 *
 */

// Include library for timers
#include "SoftTimer.h"
 //Library for Xbee Communication
#include "SoftwareSerial.h"
SoftwareSerial Xbee(2,3); //RX TX
// Include library for GPS
#include "Adafruit_GPS.h"
// Include library for IMU
//#include "LPS.h"
//#include "LSM303.h"
#include "L3G.h"
#include "wire.h"
//Include arduino servo library
#include "servo.h"
//Main task deals with tail and wing angle, variables: heading, tailDir, wingAngle
Task mainTask(50, main);
Task getCurrentGPSTask(1000, getCurrentGPS);
Task getGoalGPS(1000, getGoalGPS);
Task readEnvironmentTask(100, readEnvironment);

double goalGPSLat;
double goalGPSLon;
double currGPSLat;
double currGPSLon;
double headingLon;
double headingLat;
double windDir;
double windSpeed;
double tailDir;
double wingAngle;


void setup(Task*me){
  Serial.begin(9600);
  SoftTimer.add(&mainTask);
  Xbee.begin(9600);
  servo.attach(pin);
  servo.attach(pin);
  wire.begin();
  compass.init();
  compass.enableDefault();

  //include various configurations, need to set up the actual arduino
  //include pinModes
}

void getGoalGPS(Task*me){

//Communicates with Xbee
  goalGPSlon= serial.read();
  goalGPSlat= serial.read();
 } 

 void getCurrentGPS(Task* me){

  // Do stuff to talk to GPS module

  currGPSLat =GPS.latitude() ;
  currGPSLon =GPS.longitude() ;
}

void readEnvironment(){
  windSpeed=
  windDir=
}


//Main task controls the motion of the boat
void main(Task* me){
  // Get heading from IMU
  heading=compass.read();
  headingLon=
  headingLat=
  // Calculate the heading to Goal GPS location
  double lonDiff = goalGPSLon - currGPSLon;
  double latDiff = goalGPSLat - currGPSLat;

 // Compare to possible headings and select best. This will use the wind vector 

  double goalHeading=


  // Calculate optimal tail and rudder angles to point boat to goal heading
tailangle=
  // Set tail angle
  servo.write();

  //Set rudder angle
  servo.write();
}



//read sail angle from rotary encoder

// Updates global variables 


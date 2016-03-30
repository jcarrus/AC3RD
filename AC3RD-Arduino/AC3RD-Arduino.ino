/* This project
 *
 *
 *
 */


// Include library for GPS
//#include "Adafruit_GPS.h"

//Include library for IMU
#include "LPS.h"
#include "LSM303.h"
#include "L3G.h"
#include "wire.h"

 
//create variables
double currGPSLat;
double currGPSLon;
double currGPSangle;
float heading;
double windDir;
double tailDir;
double wingAngle;
double rudderAngle;

// put a glass over the wind sendor to calibrate it. adjust the zeroWindAdjustmet until sensor reads about zero
//wind sensor calibration- need a 5V power source to wind sensor
//const float zeroWindAdjustment =  0; // negative numbers yield smaller wind speeds and vice versa.

//int TMP_Therm_ADunits;  //temp termistor value from wind sensor
//float RV_Wind_ADunits;    //RV output from wind sensor 
//float RV_Wind_Volts;
//unsigned long lastMillis;
//int TempCtimes100;
//float zeroWind_ADunits;
//float zeroWind_volts;
//float WindSpeed_MPH;

void setup(){
  Serial.begin(9600);

  IMU
  wire.begin();
  compass.init();
  compass.enableDefault();
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

  // GPS
  //GPS.begin(9600)

  //include various configurations, need to set up the actual arduino
  //include pinModes
}

/*
 void getCurrentGPS(Task* me){

  // Do stuff to talk to GPS module- even if the position stays the same, want the data
  //parse data given in a NMEA string
  GPS.parse(GPS.lastNMEA());
  //get desired data
  currGPSLat =GPS.latitude() ;
  currGPSLon =GPS.longitude() ;
  currGPSangle=GPS.angle();
}
*/

//Main task controls the motion of the boat
void loop(){
  // Get heading from IMU
  heading=compass.read();
  headingLon=
  headingLat=
  // Calculate the heading to Goal GPS location
  //double lonDiff = goalGPSLon - currGPSLon;
 // double latDiff = goalGPSLat - currGPSLat;

 // Compare to possible headings and select best. This will use the wind vector 
/*
  double goalHeading= //vector sum lonDiff and latDiff
  // need the angle difference between the goal heading and the current heading
  headingAngleDiff= goalHeading- heading
  //turn tail by the negative of the headingAngleDiff, assuming we are looking at that ratio still
  tailAngle= -headingAngleDiff
  // Set tail angle
  tailservo.write(tailAngle);

  //Set rudder angle
  //rudderservo.write(rudderAngle);
}

*?

//read sail angle from rotary encoder

// Updates global variables 


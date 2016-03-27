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
#include "LSM303.h"
//#include "L3G.h"
#include "wire.h"

//Include arduino servo library
#include "servo.h"
//Main task deals with tail and wing angle, variables: heading, tailDir, wingAngle
Task mainTask(50, main);
Task getCurrentGPSTask(1000, getCurrentGPS);
Task getGoalGPS(1000, getGoalGPS);
Task readEnvironmentTask(100, readEnvironment);

//create servo objects
Servo tailservo
Servo rudderservo 
//create variables
double goalGPSLat=0;
double goalGPSLon=0;
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
const float zeroWindAdjustment =  0; // negative numbers yield smaller wind speeds and vice versa.

int TMP_Therm_ADunits;  //temp termistor value from wind sensor
float RV_Wind_ADunits;    //RV output from wind sensor 
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;

void setup(Task*me){
  Serial.begin(9600);
  SoftTimer.add(&mainTask);
  //Xbee
  Xbee.begin(9600);
  //servo
  servo.attach(pin);
  servo.attach(pin);
  //IMU
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
  GPS.begin(9600)

  //include various configurations, need to set up the actual arduino
  //include pinModes
}

void getGoalGPS(Task*me){

//Communicates with Xbee granted input "lat,lon"
  while (Serial.available()>0)
  goalGPSlon= Serial.parseInt();
  goalGPSlat= Serial.parseInt();
 } 

 void getCurrentGPS(Task* me){

  // Do stuff to talk to GPS module- even if the position stays the same, want the data
  //parse data given in a NMEA string
  GPS.parse(GPS.lastNMEA());
  //get desired data
  currGPSLat =GPS.latitude() ;
  currGPSLon =GPS.longitude() ;
  currGPSangle=GPS.angle();
}

void readEnvironment(){
  //wind sensor- arduino code from https://github.com/moderndevice/Wind_Sensor/blob/master/WindSensor/WindSensor.ino 
  //temp info might unnecessary
      
      TMP_Therm_ADunits = analogRead(analogPinForTMP);
      RV_Wind_ADunits = analogRead(analogPinForRV);
      RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);

      // these are all derived from regressions from raw data as such they depend on a lot of experimental factors
      // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
      TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  

      zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

      zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  

      // This from a regression from data in the form of 
      // Vraw = V0 + b * WindSpeed ^ c
      // V0 is zero wind at a particular temperature
      // The constants b and c were determined by some Excel wrangling with the solver.
    
      WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265); 
 
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

  double goalHeading= //vector sum lonDiff and latDiff
  // need the angle difference between the goal heading and the current heading
  headingAngleDiff= goalHeading- heading
  //turn tail by the negative of the headingAngleDiff, assuming we are looking at that ratio still
  tailAngle= -headingAngleDiff
  // Set tail angle
  tailservo.write(tailAngle);

  //Set rudder angle
  rudderservo.write(rudderAngle);
}



//read sail angle from rotary encoder

// Updates global variables 


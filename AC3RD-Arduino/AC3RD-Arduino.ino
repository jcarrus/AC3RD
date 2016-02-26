/* This project
 *
 *
 *
 */

// Include library for timers
#include "SoftTimer.h"

// Include library for GPS

Task mainTask(50, main);
Task getCurrentGPSTask(1000, getCurrentGPS);

double goalGPSLat;
double goalGPSLon;
double currGPSLat;
double currGPSLon;

void setup(){
  Serial.begin(9600);
  SoftTimer.add(&mainTask);
}

// 
void main(Task* me){
  // Calculate the goal heading
  double lonDiff = goalGPSLon - currGPSLon;

  // Compare to possible headings and select best


  // Calculate optimal tail and rudder angles

  // Set tail and rudder angles

}

// Updates global variables 
void getCurrentGPS(Task* me){

  // Do stuff to talk to GPS module

  currGPSLat = something;
  currGPSLon = somethingelse;
}
/*
 * This spring 2016 project for 2.017 by Ali Trueworthy, Brian
 * Gilligan, Jorlyn LeGarrec, Justin Carrus, Trevor Day, and Val Peng.
 */

///////////////////
// Timer Library //
///////////////////
#include "SoftTimer.h"

//////////
// XBee //
//////////
#include "SoftwareSerial.h"
SoftwareSerial Xbee(2,3); //RX TX
void setupXBee(){
	XBee.begin(9600);
}

/////////
// GPS //
/////////
#include "Adafruit_GPS.h"
int deadZone = (int) 45 * PI / 180;
double goalGPSTolerance;
float goalGPSLat;
float goalGPSLon;
int headingToGoal;
float gpsLat; 
float gpsLon;
int gpsHeading;
void getGPS(){
  // Do stuff to talk to GPS module- even if the position stays the same, want the data
  //parse data given in a NMEA string
  GPS.parse(GPS.lastNMEA());
  //get desired data
  gpsLat     = GPS.latitude();
  gpsLon     = GPS.longitude();
  gpsHeading = GPS.angle();
}
void measure(double lat1, double lon1, double lat2, double lon2){
	double R = 6378.137; // Radius of earth in KM
	double dLat = (lat2 - lat1) * PI / 180;
	double dLon = (lon2 - lon1) * PI / 180;
	double a = pow(sin(dLat/2), 2) + cos(lat1 * PI / 180) * cos(lat2 * PI / 180) * pow(sin(dLon/2), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	distToGoal = R * c * 1000; // meters
	double x = cos(lat2) * sin(dLon);
	double y = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);							
	headingToGoal = (int) (atan2(x, y) / PI * 180); // degrees
}


/////////
// IMU //
/////////
#include "LSM303.h"
#include "wire.h"
LSM303 imu;
float heading;
void setupIMU(){	
	wire.begin();
  imu.init();
  imu.enableDefault();
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  imu.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  imu.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
	SoftTimer.add(&taskIMU);
}
void taskIMU(Task* me){
	heading=imu.read();
	imu.read();
	Xbee.print(imu.a.x);
	Xbee.print(',');
	Xbee.print(imu.a.y);
	Xbee.print(',');
	Xbee.print(imu.a.z);
	Xbee.print('\n');
}

/////////////////
// Wind Sensor //
/////////////////
// code from:
// https://github.com/moderndevice/Wind_Sensor/blob/master/WindSensor/WindSensor.ino
//
// put a glass over the wind sendor to calibrate it.  adjust the
// zeroWindAdjustmet until sensor reads about zero wind sensor
// calibration- need a 5V power source to wind sensor
float zeroWindAdjustment =  0;
float temp_raw;
float RV_Wind_ADunits;
float RV_Wind_Volts;
float TempC;
float zeroWind_ADunits;
float zeroWind_volts;
float windSpeed;
void setupWind(){
}
void getWind(){
	temp_raw = (float) analogRead(WIND_TMP_PIN);
	RV_Wind_ADunits = (float) analogRead(WIND_SPD_PIN);
	RV_Wind_Volts = (RV_Wind_ADunits * 0.0048828125);
	TempCtimes100 = ((0.005 * (temp_raw * temp_raw)) - (16.862 * temp_raw) + 9075.4) / 100;
	zeroWind_ADunits = -0.0006*(temp_raw * temp_raw) + 1.0727 * temp_raw + 47.172;
	zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  
	windSpeed =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265) * 0.44704; 
} 

////////////
// Servos //
////////////
#include "servo.h"
#define TAIL_PIN 5
#define RUDDER_PIN 6
Servo tailServo;
Servo rudderServo;
int tailAngle;
int rudderAngle;
int kRudder; 
int rudderControl;
void setupServo(){
	tailServo.attach(TAIL_PIN);
  rudderServo.attach(RUDDER_PIN);
}
void setHeading(int h){
	while (h < 0){
		h += 360;
	}
	headingDesired = h % 360;
}
void setTailAngle(int a){
	tailServo.write(a);
	tailAngle = a;
}
void setRudderAngle(int a){
	rudderServo.write(a);
	rudderAngle = a;
}

////////////////////
// Potentiometers //
////////////////////
#define POT_PIN_1 3
#define POT_PIN_2 4
int windDir; // any value between 0 and 359, points into the wind
double wingAngle;
int frontPotAngle;
int rearPotAngle;
void getWindAngle(){
	frontPotAngle = map(analogread(POT_PIN_1), 0, 1023, 0, 330);
	rearPotAngle  = map(analogread(POT_PIN_2), 0, 1023, 0, 330);
  if (frontPotAngle >= 0 && frontPotAngle <= 200){
		wingAngle = frontPotAngle - 90;
	} else {
		wingAngle = rearPotAngle + 90;
	}
  windDir = (heading + wingAngle + tailAngle) % 360; // Points into the wind
}


///////////////////////
// Task Declarations //
///////////////////////
Task fastTask(10, fast);
Task mainTask(50, main);
Task commTask(200, comm);
Task rudderTask(10, rudder);

///////////
// Setup //
///////////

void setup(Task*me){
	setupXBee();
	setupIMU();
	setupServo();
	SoftTimer.add(&fastTask);
	SoftTimer.add(&commTask);
  SoftTimer.add(&mainTask);
}

float headingDirect;
float headingDesired;

void main(Task* me){
  getGPS();
  measure(gpsLat, gpsLon, goalGPSLat, goalGPSLon);
	// If we are at the destination, stop
	if (distToGoal < goalGPSTolerance){
		rudderServo.write(rudderServoMax);
		tailServo.write(tailServoZero);
		return;
	}

  // we cannot go anywhere withing the deadZone of the wind
	// if the desired, most direct heading is not within 45 degrees of the wind,
	// want to go in the direction of desired heading

	
	// If we want to sail in the deadZone
	if (abs(headingToGoal - windDir) <= deadZone || abs(headingToGoal - windDir) >= 360 - deadZone){
		// and were previously on a direct route
		if (routeChoice == 0){
			if ((abs(headingToGoal - windDir) < 180 && headingToGoal > windDir) ||
					(abs(headingToGoal - windDir) > 180 && headingToGoal < windDir)){
				setHeading(windDir + deadZone);
				routeChoice = 1;
			} else {
				setHeading(windDir - deadZone);
				routeChoice = -1;
			}
		} else {
			if (routeChoice == 1){
				setHeading(windDir + deadZone);
			} else {
				setHeading(windDir - deadZone);
			}
		}
	} else {
		routeChoice = 0;
		setHeading(headingToGoal);
	}
	
  //generate a force in the forward direction
  if ((abs(headingDesired - windDir) < 180 && headingDesired > windDir) ||
			(abs(headingDesired - windDir) > 180 && headingDesired < windDir)){
		setTailAngle(-liftTailAngle);
	}	else {
		setTailAngle( liftTailAngle);
	}
}

void rudder(Task* me){
  if rudderControl==1{
	// Simple proportional controller with feedback
	 if ((abs(heading - headingDesired) < 180 && heading > headingDesired) ||
   		abs(heading - headingDesired) > 180 && heading < headingDesired)){
		  setRudderAngle(-kRudder * abs(heading - headingDesired));
	 }  
    else {
		  setRudderAngle( kRudder * abs(heading - headingDesired));
	 }
  }


  if rudderControl==2{
  //Another type of rudder control

  }
}

char buf[8];
void comm(Task* me){
	if (XBee.available() >= 7){
		for (int i = 0; i < 3; i++){
			buf[i] = (char) XBee.read();
		}
		switch (buffer){
		// setglat: set goal latitude
		case "setglat":
      goalGPSLat = commReadInt();
			break;
    //getglat: get goal latitude
		case "getglat":
      Xbee.read();
      Xbee.println(goalGPSLat);
			break;
		// setglon: set goal longitude
		case "setglon":
      goalGPSLon = commReadInt();
			break;
    //getglon: gets goal GPS Longitude from Xbee
		case "getglon":
      Xbee.read();
      Xbee.println(goalGPSLon);
			break;
    //setdzne: sets the value of the deadzone
		case "setdzne":
      deadZone = commReadInt();
		  break;
    //gets the value of the deadzone
		case "getdzne":
      XBee.read();
      XBee.println(deadZone);
			break;
    //setrcon: sets the type of rudder control we are using
		case "setrcon":
			break;
    //getrcon: gets the # associated with the type of rudder control in use
		case "getrcon":
			break;
    //setkrud: sets the rudder gain for rudder controller
		case "setkrud":
			kRudder = commReadInt();
			break;
    //getkrud: gets the rudder gain from Xbee
		case "getkrud":
			XBee.read();
			XBee.println(kRudder);
			break;
		}
	}
}

int commReadInt(){
	String str = "";
	while(XBee.peek() != '\n'){
		str += XBee.read();
	}
	XBee.read();
	return str.toInt();
}

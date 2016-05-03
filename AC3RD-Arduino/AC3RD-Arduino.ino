/*
 * This spring 2016 project for 2.017 by Ali Trueworthy, Brian
 * Gilligan, Jorlyn LeGarrec, Justin Carrus, Trevor Day, and Val Peng.
 */

//////////
// Comm //
//////////
//#include <SoftwareSerial.h>
//SoftwareSerial Serial(2,3); //RX TX
void setupComm(){
	Serial.begin(19200);
  Serial.flush();
}

/////////
// GPS //
/////////

#include "./Adafruit_GPS.h"
SoftwareSerial gpsSerial(3, 2);
Adafruit_GPS GPS(&gpsSerial);
#define PI 3.14159
#define RADIUS_EARTH 6378.137
int deadZone = 25;
double goalGPSTolerance;
float goalGPSLat;
float goalGPSLon;
int headingToGoal;
float gpsLat; 
float gpsLon;
void readGPS(){
  // Do stuff to talk to GPS module- even if the position stays the same, want the data
  //parse data given in a NMEA string
  GPS.parse(GPS.lastNMEA());
  //get desired data
  gpsLat     = GPS.latitude;
  gpsLon     = GPS.longitude;
}
int gpsDistanceTo(double lat1, double lon1, double lat2, double lon2){
	double dLat = (lat2 - lat1) * PI / 180;
	double dLon = (lon2 - lon1) * PI / 180;
	double a = pow(sin(dLat/2), 2) + cos(lat1 * PI / 180) * cos(lat2 * PI / 180) * pow(sin(dLon/2), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	return (int) (RADIUS_EARTH * c * 1000); // meters
}
int gpsAngleTo(double lat1, double lon1, double lat2, double lon2){
	double dLon = (lon2 - lon1) * PI / 180;
	double x = cos(lat2) * sin(dLon);
	double y = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);							
	return (int) (atan2(x, y) / PI * 180); // degrees
}

/////////
// IMU //
/////////
#include <Wire.h>
#include "./LSM303.h"
#include "./L3G.h"
LSM303 imu;
L3G gyro;
int heading;
void setupIMU(){	
	Wire.begin();
  if (!imu.init()){
		Serial.println("904");
	}
  imu.enableDefault();
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  imu.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  imu.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
	if (!gyro.init()){
		Serial.println("905");
	}
	gyro.enableDefault();
}
void readHeading(){
	imu.read();
	heading = imu.heading((LSM303::vector<int>){-1, 0, 0});
}
void readGyro(){
	gyro.read();
}
int calibrationLength = 15000;
void calibrateMagnetometer(){
  imu.m_min = (LSM303::vector<int16_t>) {-32767, -32767, -32767};
  imu.m_max = (LSM303::vector<int16_t>) { 32767,  32767,  32767};
  long int endTime = millis() + calibrationLength;
	LSM303::vector<int16_t> running_min = { 32767,  32767,  32767};
	LSM303::vector<int16_t> running_max = {-32768, -32768, -32768};
  while (millis() < endTime){
		imu.read();
		running_min.x = min(running_min.x, imu.m.x);
		running_min.y = min(running_min.y, imu.m.y);
		running_min.z = min(running_min.z, imu.m.z);
		running_max.x = max(running_max.x, imu.m.x);
		running_max.y = max(running_max.y, imu.m.y);
		running_max.z = max(running_max.z, imu.m.z);
  }
	imu.m_min = running_min;
  imu.m_max = running_max;
	Serial.print(imu.m_min.x);
	Serial.print(',');
	Serial.print(imu.m_min.y);
	Serial.print(',');
	Serial.print(imu.m_min.z);
	Serial.print(',');
	Serial.print(imu.m_max.x);
	Serial.print(',');
	Serial.print(imu.m_max.y);
	Serial.print(',');
	Serial.print(imu.m_max.z);
	Serial.println();
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
#define WIND_TMP_PIN 0
#define WIND_SPD_PIN 0
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
	float TempCtimes100 = ((0.005 * (temp_raw * temp_raw)) - (16.862 * temp_raw) + 9075.4) / 100;
	zeroWind_ADunits = -0.0006*(temp_raw * temp_raw) + 1.0727 * temp_raw + 47.172;
	zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  
	windSpeed =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265) * 0.44704; 
} 

////////////
// Servos //
////////////
#include <Servo.h>
#define TAIL_PIN 9
#define RUDDER_PIN 10
Servo tailServo;
Servo rudderServo;
int rudderServoOffset = 95;
int rudderServoDir = 1;
int tailServoOffset = 75;
int tailServoDir = -1;
void setupServo(){
	tailServo.attach(TAIL_PIN);
  rudderServo.attach(RUDDER_PIN);
}
void zeroServos(){
  tailServo.detach();
  rudderServo.detach();
  delay(5000);
  setupServo();
}
void tailServoFlip(){
  tailServoDir *= -1;
}
void rudderServoFlip(){
  rudderServoDir *= -1;
}

////////////////////
// Potentiometers //
////////////////////
#define POT_PIN_1 1
#define POT_PIN_2 2
int windDir; // any value between 0 and 359, points into the wind
int wingAngle;
int frontPotAngle;
int rearPotAngle;
void readWingAngle(){
	frontPotAngle = map(analogRead(POT_PIN_1), 0, 1023, 0, 330);
	rearPotAngle  = map(analogRead(POT_PIN_2), 0, 1023, 0, 330);
  if (true || frontPotAngle >= 90 && frontPotAngle <= 270){
		wingAngle = ((frontPotAngle - 120) + 360) % 360;
	} else {
		wingAngle = ((rearPotAngle + 60) + 360) % 360;
	}
}


/////////////////////
// State Variables //
/////////////////////
int rudderAngle;
int tailAngle;


///////////
// Sense //
///////////

void sense(){
  readWingAngle();
  readHeading();
  readGyro();
  windDir = (heading + wingAngle + tailAngle) % 360; // Points into the wind
}

//////////////
// Navigate //
//////////////

int distToGoal = 999999;
int headingDesired;
int tailServoMax = 90;
int tailServoZero;
int liftTailAngle = 15;
int routeChoice = 0;
int navigationMode = 1;
void setHeading(int h){
  while (h < 0){
    h += 360;
  }
  headingDesired = h % 360;
}
int suggestHeading(int h){
  // If we want to sail in the deadZone
  if (abs(h - windDir) <= deadZone || abs(h - windDir) >= 360 - deadZone){
    // and were previously on a direct route
    if (routeChoice == 0){
      if ((abs(h - windDir) < 180 && h > windDir) || (abs(h - windDir) > 180 && h < windDir)){
        routeChoice = 1;
        return windDir + deadZone;
      } else {
        routeChoice = -1;
        return windDir - deadZone;
      }
    } else {
      if (routeChoice == 1){
        return windDir + deadZone;
      } else {
        return windDir - deadZone;
      }
    }
  } else {
    routeChoice = 0;
    return h;
  }
}
void navigate(){
  switch (navigationMode) {
    case 0:
			// Passive Sailing
      setHeading(heading);
      break;
    case 1:
			// Maintain a heading (managed by communicate)
      setHeading(suggestHeading(headingToGoal));
      break;
    case 2:
			// GPS navigation
      gpsDistanceTo(gpsLat, gpsLon, goalGPSLat, goalGPSLon);
      // If we are at the destination, stop
      if (distToGoal < goalGPSTolerance){
      }
    default:
      Serial.println("901");
    break;
  }
}

//////////
// Sail //
//////////

int tailControl = 1;
void setTailAngle(int a){
  tailServo.write(tailServoOffset + tailServoDir*a);
  tailAngle = a;
}
void sail(){
  switch (tailControl) {
    case 0:
			// Manual control depends on serial communication
      break;
    case 1:
			// Sail for forward motion
      if ((abs(headingDesired - windDir) < 180 && headingDesired > windDir) ||
          (abs(headingDesired - windDir) > 180 && headingDesired < windDir)){
        setTailAngle(-liftTailAngle);
      } else {
        setTailAngle( liftTailAngle);
      }
      break;
    default:
      Serial.println("902");
    break;
  }
}

//////////
// Turn //
//////////

int rudderControl = 1;
int rudderServoMax;
int rudderServoZero;
int kRudder = 5;
int maxRudderAngle = 45;
void setRudderAngle(int a){
  a = min(a, maxRudderAngle);
  a = max(a, -maxRudderAngle);
  rudderServo.write(rudderServoOffset + rudderServoDir*a);
  rudderAngle = a;
}
void turn(){
  switch (rudderControl){
  	case 0:
			// Manual Control depends on serial commands
		  break;
	  case 1:
	    // Simple proportional controller with feedback
	    if (((abs(heading - headingDesired) < 180) && (heading > headingDesired)) ||
   		    ((abs(heading - headingDesired) > 180) && (heading < headingDesired))){
		    setRudderAngle(-kRudder * abs(heading - headingDesired));
	    }  
      else {
		    setRudderAngle( kRudder * abs(heading - headingDesired));
	    }
     break;
    case 2:
      //PD controller
      if (((abs(heading - headingDesired) < 180) && (heading > headingDesired)) ||
         ((abs(heading - headingDesired) > 180) && (heading < headingDesired))){
        setRudderAngle(-kRudder * abs(heading - headingDesired)-0); // Needs more stuff
      } else {
        setRudderAngle( kRudder * abs(heading - headingDesired)-0); // Needs more stuff
      }
      break;
    default:
      Serial.println("903");
    break;
  }
}

/////////////////
// Communicate //
/////////////////

String str;
int commCode;
int commReadInt(){
  str = "";
  while((char) Serial.peek() != '\n'){
    str += (char) Serial.read();
  }
  Serial.read();
  return str.toInt();
}
void gobbleCommand(){
	while (Serial.read() != '\n'){}
}
void communicate(){
  if (Serial.available()){
    str = "";
    while ((char) Serial.peek() != ','){
      if ((char) Serial.peek() == '\n'){
        Serial.read();
        return;
      }
      str += (char) Serial.read();
    }
    Serial.read();
    commCode = str.toInt();
		Serial.println(commCode);
    switch (commCode){
      //  0 -  9 are navigation commands
      case 1:
        headingToGoal = commReadInt();
        break;
      case 2:
        setTailAngle(commReadInt());
        break;
		  case 3:
				setRudderAngle(commReadInt());

      // 10 - 30 are configuration commands
      case 10:
        zeroServos();
        gobbleCommand();
        break;
      case 11:
        tailServoFlip();
        gobbleCommand();
        break;
      case 12:
        rudderServoFlip();
        gobbleCommand();
        break;
      case 13:
        setTailAngle(commReadInt());
        delay(5000);
        break;
      case 14:
        setRudderAngle(commReadInt());
        delay(5000);
        break;
      case 15:
        deadZone = commReadInt();
        break;
      case 16:
        liftTailAngle = commReadInt();
        break;
      case 17:
        maxRudderAngle = commReadInt();
        break;
		  case 18:
				tailControl = commReadInt();
		  case 19:
				rudderControl = commReadInt();
      case 20:
        calibrateMagnetometer();
        gobbleCommand();
		  default:
        Serial.println("902");
      break;
    }
  }
	
  Serial.print(millis());
  Serial.print(',');
  Serial.print(tailAngle);
  Serial.print(',');
  Serial.print(rudderAngle);
  Serial.print(',');
  Serial.print(heading);
  Serial.print(',');
  Serial.print(headingDesired);
  Serial.print(',');
  Serial.print(wingAngle);
  Serial.print(',');
  Serial.print(windDir);
  Serial.print(',');
  Serial.print(routeChoice);
  Serial.print(',');
  Serial.print(imu.a.x);
  Serial.print(',');
  Serial.print(imu.a.y);
  Serial.print(',');
  Serial.print(imu.a.z);
  Serial.print(',');
  Serial.print(imu.m.x);
  Serial.print(',');
  Serial.print(imu.m.y);
  Serial.print(',');
  Serial.print(imu.m.z);
  Serial.print(',');
  Serial.print(gyro.g.x * 8.75);
  Serial.print(',');
  Serial.print(gyro.g.y * 8.75);
  Serial.print(',');
  Serial.print(gyro.g.z * 8.75);
  Serial.print('\n');
}

////////////////////
// Setup and Loop //
////////////////////

void setup(){
  setupComm();
  setupIMU();
  setupServo();
}

void loop(){
  sense();
  navigate();
  sail();
  turn();
  communicate();
}

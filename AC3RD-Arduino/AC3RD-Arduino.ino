/*
 * This spring 2016 project for 2.017 by Ali Trueworthy, Brian
 * Gilligan, Jorlyn LeGarrec, Justin Carrus, Trevor Day, and Val Peng.
 */

////////////
// Timing //
////////////
#include "./SimpleTimer.h"
SimpleTimer timer;


/////////////////////
// State Variables //
/////////////////////
int rudderAngle;
int tailAngle;


//////////
// Comm //
//////////
//#include <SoftwareSerial.h>
//SoftwareSerial Serial(2,3); //RX TX
#define XBee Serial1
void setupComm(){
	Serial.begin(19200);
	Serial.flush();
}


/////////
// GPS //
/////////
#include "./Adafruit_GPS.h"
#define gpsSerial Serial2
Adafruit_GPS GPS(&gpsSerial);
#define RADIUS_EARTH 6378.137
int deadZone = 25;
double goalGPSTolerance;
float goalGPSLat;
float goalGPSLon;
int headingToGoal;
float gpsLat; 
float gpsLon;
int gpsFix;
int gpsQuality;
int gpsSpeed;
int gpsInterval = 100;
int gpsTimer;
void setupGPS(){
	GPS.begin(9600);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
	timer.setInterval(2, pollGPS);
	startGPS();
}
void pollGPS(){
	GPS.read();
}
void startGPS(){
	gpsTimer = timer.setInterval(gpsInterval, readGPS);
}
void readGPS(){
  if (GPS.newNMEAreceived()) {
		if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
	gpsFix = GPS.fix;
	gpsQuality = GPS.fixquality;
	gpsLat = GPS.latitudeDegrees;
	gpsLon = GPS.longitudeDegrees;
	gpsSpeed = GPS.speed * 0.514444;
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
#include "SparkFunLSM9DS1.h"
LSM9DS1 imu;
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW
#define DECLINATION -14.75 // Declination (degrees) in Boulder, CO.
int heading;
int imuInterval = 10;
int imuTimer;
void setupIMU(){	
	imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin()){
		Serial.println("904");
		delay(1000);
	}
	startIMU();
}
void startIMU(){
	imuTimer = timer.setInterval(imuInterval, readIMU);
}
void readIMU(){
	imu.readAccel();
	imu.readGyro();
	imu.readMag();
	float myheading;
  if (imu.my == 0)
    myheading = (imu.mx < 0) ? 180 : 0;
  else
    myheading = atan2(imu.mx, imu.my);
  myheading -= DECLINATION * PI / 180.0;  
  if (myheading > PI) myheading -= (2 * PI);
  else if (myheading < -PI) myheading += (2 * PI);
  else if (myheading < 0) myheading += 2 * PI;  
  // Convert everything from radians to degrees:
  heading = (int) (myheading * 180.0 / PI);
}


/////////////////
// Wind Sensor //
/////////////////
// code from:
// https://github.com/moderndevice/Wind_Sensor/blob/master/WindSensor/WindSensor.ino
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
	//float TempCtimes100 = ((0.005 * (temp_raw * temp_raw)) - (16.862 * temp_raw) + 9075.4) / 100;
	zeroWind_ADunits = -0.0006*(temp_raw * temp_raw) + 1.0727 * temp_raw + 47.172;
	zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  
	windSpeed =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265) * 0.44704; 
} 


////////////
// Servos //
////////////
#include <Servo.h>
#define TAIL_PIN 11
#define RUDDER_PIN 12
Servo tailServo;
Servo rudderServo;
int rudderServoOffset = 115;
int rudderServoDir = 1;
int tailServoOffset = 70;
int tailServoDir = 1;
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
#define POT_PIN_2 0
int windDir; // any value between 0 and 359, points into the wind
int wingAngle;
int frontPotAngle;
int rearPotAngle;
int wingInterval = 100;
int wingTimer;
void setupWingAngle(){
	startWingAngle();
}
void startWingAngle(){
	wingTimer = timer.setInterval(wingInterval, readWingAngle);
}
void readWingAngle(){
	frontPotAngle = map(analogRead(POT_PIN_1), 0, 1023, 0, 330);
	rearPotAngle  = map(analogRead(POT_PIN_2), 0, 1023, 0, 330);
  if (frontPotAngle >= 90 && frontPotAngle <= 270){
		wingAngle = ((frontPotAngle - 120) + 360) % 360;
	} else {
		wingAngle = ((rearPotAngle + 60) + 360) % 360;
	}
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
int navigationMode = 2;
int navigateInterval = 100;
int navigateTimer;
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
    case 1:
			// Passive Sailing
      setHeading(heading);
      break;
    case 2:
			// Maintain a heading (managed by communicate)
      setHeading(suggestHeading(headingToGoal));
      break;
    case 3:
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
void startNavigate(){
	navigateTimer = timer.setInterval(navigateInterval, navigate);
}


//////////
// Sail //
//////////
int tailControl = 1;
int sailInterval = 100;
int sailTimer;
void setTailAngle(int a){
  tailServo.write(tailServoOffset + tailServoDir*a);
  tailAngle = a;
}
void sail(){
  switch (tailControl) {
    case 1:
			// Manual control depends on serial communication
      break;
    case 2:
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
void startSail(){
	sailTimer = timer.setInterval(sailInterval, sail);
}

//////////
// Turn //
//////////
int rudderControl = 1;
int rudderServoMax;
int rudderServoZero;
int kRudder = 1;
int maxRudderAngle = 45;
int turnInterval = 100;
int turnTimer;
void setRudderAngle(int a){
  a = min(a, maxRudderAngle);
  a = max(a, -maxRudderAngle);
  rudderServo.write(rudderServoOffset + rudderServoDir*a);
  rudderAngle = a;
}
void turn(){
  switch (rudderControl){
  	case 1:
			// Manual Control depends on serial commands
		  break;
	  case 2:
	    // Simple proportional controller with feedback
	    if (((abs(heading - headingDesired) < 180) && (heading > headingDesired)) ||
   		    ((abs(heading - headingDesired) > 180) && (heading < headingDesired))){
		    setRudderAngle(-kRudder * abs(heading - headingDesired));
	    }  
      else {
		    setRudderAngle( kRudder * abs(heading - headingDesired));
	    }
     break;
    case 3:
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
void startTurn(){
	turnTimer = timer.setInterval(turnInterval, turn);
}


/////////
// Log //
/////////
int logInterval = 100;
int logTimer;
void log(){
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
  Serial.print(atan2(imu.ay, imu.az) * 180 / PI);
  Serial.print(',');
  Serial.print(atan2(-imu.ax, sqrt(imu.ay * imu.ay + imu.az * imu.az)) * 180 / PI);
  Serial.print(',');
  Serial.print(heading);
  Serial.print(',');
  Serial.print(imu.calcAccel(imu.ax));
  Serial.print(',');
  Serial.print(imu.calcAccel(imu.ay));
  Serial.print(',');
  Serial.print(imu.calcAccel(imu.az));
  Serial.print(',');
  Serial.print(imu.calcMag(imu.mx));
  Serial.print(',');
  Serial.print(imu.calcMag(imu.my));
  Serial.print(',');
  Serial.print(imu.calcMag(imu.mz));
  Serial.print(',');
  Serial.print(imu.calcGyro(imu.gx));
  Serial.print(',');
  Serial.print(imu.calcGyro(imu.gy));
  Serial.print(',');
  Serial.print(imu.calcGyro(imu.gz));
  Serial.print(',');
  Serial.print(gpsLat);
  Serial.print(',');
  Serial.print(gpsLon);
  Serial.print(',');
  Serial.print(gpsFix);
  Serial.print(',');
  Serial.print(gpsQuality);
  Serial.print(',');
  Serial.print(gpsSpeed);
  Serial.print('\n');
}
void startLog(){
	logTimer = timer.setInterval(logInterval, log);
}


/////////////////
// Communicate //
/////////////////
String str;
int commCode;
int communicateInterval = 100;
int communicateTimer;
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
      //  1 -  9 are navigation commands
      case 1:
        headingToGoal = commReadInt();
        break;
      case 2:
        setTailAngle(commReadInt());
        break;
		  case 3:
				setRudderAngle(commReadInt());
				break;
      // 10 - 19 are configuration commands
		  case 10:
				tailControl = commReadInt();
				break;
		  case 11:
				rudderControl = commReadInt();
				break;
      case 12:
        liftTailAngle = commReadInt();
        break;
      case 13:
        maxRudderAngle = commReadInt();
        break;
      case 14:
        deadZone = commReadInt();
        break;
      case 15:
        kRudder = commReadInt();
        break;
			// 20 - 29 are logging commands
  		case 20:
				navigateInterval = commReadInt();
				timer.deleteTimer(navigateTimer);
				startNavigate();
	  		break;
  		case 21:
				sailInterval = commReadInt();
				timer.deleteTimer(sailTimer);
				startSail();
	  		break;
  		case 22:
				turnInterval = commReadInt();
				timer.deleteTimer(turnTimer);
				startTurn();
	  		break;
  		case 23:
				communicateInterval = commReadInt();
				timer.deleteTimer(communicateTimer);
				startCommunicate();
	  		break;
  		case 24:
				logInterval = commReadInt();
				timer.deleteTimer(logTimer);
				startLog();
	  		break;
  		case 25:
				gpsInterval = commReadInt();
				timer.deleteTimer(gpsTimer);
				startGPS();
	  		break;
  		case 26:
				imuInterval = commReadInt();
				timer.deleteTimer(imuTimer);
				startIMU();
	  		break;
  		case 27:
				wingInterval = commReadInt();
				timer.deleteTimer(wingTimer);
				startWingAngle();
 	  		break;
			// 30 - 39 are calibration commands
      case 30:
        imu.calibrateMag();
        gobbleCommand();
				break;
      case 31:
        tailServoFlip();
        gobbleCommand();
        break;
      case 32:
        rudderServoFlip();
        gobbleCommand();
        break;
		  default:
        Serial.println("902");
      break;
    }
  }
}
void startCommunicate(){
	communicateTimer = timer.setInterval(communicateInterval, communicate);
}

////////////////////
// Setup and Loop //
////////////////////

void setup(){
  setupComm();
	setupGPS();
  setupIMU();
	setupWingAngle();
  setupServo();
	startNavigate();
	startSail();
	startTurn();
	startLog();
	startCommunicate();
}

void loop(){
	timer.run();
}

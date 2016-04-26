#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

///GPS Readings////
SoftwareSerial gpsSerial(8, 7);
Adafruit_GPS GPS(&gpsSerial);
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean);
String latitude = "0";
String longitude = "0";
String gpsSpeed = "0";
String fixQuality = "0";

/////Auto/Manual Control/////
boolean autoSwitch;

/////Servo Control/////
Servo tailServo;
Servo rudderServo;
int tailZero = 75; //zero angle for servo
int rudderZero = 75; //zero angle for zero
int tailRange = 20; // +/- angle range for servo, offset angle to keep positive integers, must alter also in Processing code
int rudderRange = 45; // + or - angle range for servo, offset angle to keep positive integers, must alter also in Processing code
int tailAngle = tailZero; //init initial angle
int rudderAngle = rudderZero; //init intial angle
String controlInput = "";
boolean controlComplete = false;

//////Sail Angle Readings/////
#define potPin1 A3 //analog
#define potPin2 A2 //analog
int potPin1Val;
int potPin2Val;

//////IMU Readings///////
L3G gyro;
LSM303 compass;
const int chipSelect = 10;
char report[80];
//SDA is A4 and SCL is A5///

//////Wind readings///////
// put a glass over the wind sendor to calibrate it. adjust the zeroWindAdjustment until sensor reads about zero
//wind sensor calibration- need a 5V power source to wind sensor
const float zeroWindAdjustment =  0; // negative numbers yield smaller wind speeds and vice versa.
int TMP_Therm_ADunits;  //temp termistor value from wind sensor
float RV_Wind_ADunits;    //RV output from wind sensor 
float RV_Wind_Volts;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;
#define analogPinForRV A1
#define analogPinForTMP A0 

//Timer//
uint32_t startMicros;
uint32_t lastMillis;
int servoTimer; //timer to prevent spamming the servo write command


void setup() {
  //Init serial communication with Xbee//
  Serial.begin(19200);
  
  //Init GPS//
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //we just want minimum and fix quality
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //update rate of 1Hz
  
  //Init Gyro and Compass//
   Wire.begin();
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.enableDefault();
  compass.init();
  compass.enableDefault();

  //Init servos//
  tailServo.attach(9);
  rudderServo.attach(10);
  tailServo.write(tailZero);
  rudderServo.write(rudderZero);

  //Data header///
  String dataHeader = "GyroX,GyroY,GyroZ,CompassAX,CompassAY,CompassAZ,CompassMX,CompassMY,CompassMZ,Pot1Angle,Pot2Angle,LatDeg,LongDeg,Velocity,fixQuality,windspeedMPH";
  Serial.println(dataHeader);

}

void loop() {
    startMicros = micros();

    //////GPS///////
    if (! usingInterrupt) {
        // read data from the GPS in main
        char c = GPS.read();
        // for debug
        if (GPSECHO)
          if (c) Serial.print(c);
      }
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    if (GPS.fix){
      latitude = String(GPS.latitudeDegrees,4);
      longitude = String(GPS.longitudeDegrees,4);
      gpsSpeed = String(GPS.speed);
      fixQuality = String((int)GPS.fixquality);
    }
    else { //set default strings
      latitude = "N/a"; 
      longitude = "N/a";
      gpsSpeed = "N/a";
      fixQuality = "0";
    }
    //////Read potentionmenters and map to angles//////
    potPin1Val = map(analogRead(potPin1), 0, 1023, 0, 330);
    potPin2Val = map(analogRead(potPin2), 0, 1023, 0, 330);
    
    /////Gyro+Compass Read//////
    gyro.read();
    compass.read();
    snprintf(report, sizeof(report), "%6d,%6d,%6d,%6d,%6d,%6d",
    compass.a.x, compass.a.y, compass.a.z,
    compass.m.x, compass.m.y, compass.m.z);

    ///////Windspeed read///////
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

    ////Send a data string at 10HZ////
    if (millis() - lastMillis > 100) { 
      lastMillis = millis(); // reset the timer
      String dataString = "";
      dataString = String((int)gyro.g.x) + "," + String((int)gyro.g.y) + "," + String((int)gyro.g.z) //gyro data 
      + "," + report + ","+ String(potPin1Val) + "," + String(potPin2Val)+"," //Compass and pot data
      + latitude + ","+ longitude + "," + gpsSpeed + "," + fixQuality ///GPS data
      + "," + String(WindSpeed_MPH);
      Serial.println(dataString);
    }
    
    ////Update the servo positions every 10ms/////  
    if (millis() - servoTimer > 10) { 
      servoTimer = millis(); // reset the timer
      tailServo.write(tailAngle);
      rudderServo.write(rudderAngle);
    }
    
    /////Update angle of servos from control//////
    if (controlComplete) {
      String tailInput = controlInput.substring(0,2); //first two chars
      String rudderInput = controlInput.substring(3,5); //second two chars
      String button = controlInput.substring(6); //last value of string
      int buttonInt = button.toInt();
      if (buttonInt == 1){
        autoSwitch = true;
      }
      else{
        autoSwitch = false;
      }
      ///Use boolen to set control scheme to parse manual inputs or automatic calculatons
      int ts = tailInput.toInt(); 
      int rs = rudderInput.toInt();
      tailAngle = tailZero-tailRange+ts;
      rudderAngle = rudderZero-rudderRange+rs;
      controlInput = "";
      controlComplete = false;
    }

}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    controlInput += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      controlComplete = true;
    }
  }
}

/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low gyro data registers.
They can be converted to units of dps (degrees per second) using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An L3GD20H gives a gyro X axis reading of 345 with its
default full scale setting of +/- 245 dps. The So specification
in the L3GD20H datasheet (page 10) states a conversion factor of 8.75
mdps/LSB (least significant bit) at this FS setting, so the raw
reading of 345 corresponds to 345 * 8.75 = 3020 mdps = 3.02 dps.
*/


// Gyro offset: X: 550 Y: 530 Z: 0
// Accelerometer Offset: X: 700 Y: -190 Z: 17000
// Magnetometer: Min: {-2270, -2301, -2963}  Max: {3053, 2638, -131}

#include <Wire.h>
#include <L3G.h>
#include <SD.h>
#include <LSM303.h>
#include <Servo.h>

Servo tailServo;
Servo rudderServo;

L3G gyro;
LSM303 compass;

const int chipSelect = 10;
char report[80];
int tailZero = 75;
int rudderZero = 75;
int tailOffset = 20;
int tailAngle;
int rudderOffset = 45;
int rudderAngle;
int ts = 0;
int rs = 0;
String input = "";
boolean stringComplete = false;

int potpin1 = 1; //analog
int potpin2 = 2; //analog
int potpin1val;
int potpin2val;
int rotaryvcc = 8; //extra voltage pin for the encoder, if needed

#define analogPinForRV 1
#define analogPinForTMP 0 

// put a glass over the wind sendor to calibrate it. adjust the zeroWindAdjustmet until sensor reads about zero
//wind sensor calibration- need a 5V power source to wind sensor
const float zeroWindAdjustment =  0; // negative numbers yield smaller wind speeds and vice versa.

int TMP_Therm_ADunits;  //temp termistor value from wind sensor
float RV_Wind_ADunits;    //RV output from wind sensor 
float RV_Wind_Volts;
unsigned long startMicros;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
  compass.init();
  compass.enableDefault();
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
  pinMode(rotaryvcc, OUTPUT);
  digitalWrite(rotaryvcc, HIGH);
  tailServo.attach(9);
  rudderServo.attach(10);
  tailServo.write(tailZero);
  rudderServo.write(rudderZero);

//  input.reserve(200);
}

void loop() {
    startMicros = micros();
    potpin1val = map(analogRead(potpin1), 0, 1023, 0, 330);
    potpin2val = map(analogRead(potpin2), 0, 1023, 0, 330);
    

    gyro.read();
    compass.read();
    snprintf(report, sizeof(report), "A: %6d %6d %6d    M: %6d %6d %6d",
    compass.a.x, compass.a.y, compass.a.z,
    compass.m.x, compass.m.y, compass.m.z);
    String dataString = "";
    dataString = "Gyro: " + String((int)gyro.g.x) + ", " + String((int)gyro.g.y) + ", " + String((int)gyro.g.z) 
    + "  " + report + " Pot 1: " + String(potpin1val) + " Pot 2: " + String(potpin2val);

    File dataFile = SD.open("datalog3.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

  if (stringComplete) {
    //Serial.println(input);
    String tailA = input.substring(0,2);
    String rudderA = input.substring(3);
    ts = tailA.toInt();
    rs = rudderA.toInt();
    tailAngle = tailZero-tailOffset+ts;
    rudderAngle = rudderZero-rudderZero+rs;
    input = "";
    stringComplete = false;
    Serial.print("Here");
  }
  tailServo.write(tailAngle);
  rudderServo.write(rudderAngle);

// Windspeed
//      TMP_Therm_ADunits = analogRead(analogPinForTMP);
//      RV_Wind_ADunits = analogRead(analogPinForRV);
//      RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);
//
//      // these are all derived from regressions from raw data as such they depend on a lot of experimental factors
//      // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
//      TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  
//
//      zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39
//
//      zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  
//
//      // This from a regression from data in the form of 
//      // Vraw = V0 + b * WindSpeed ^ c
//      // V0 is zero wind at a particular temperature
//      // The constants b and c were determined by some Excel wrangling with the solver.
//    
//      WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265); 
//      Serial.print(" Windspeed MPH: ");
//      Serial.print((float)WindSpeed_MPH);

      Serial.print(" Timestep: ");
      Serial.println(micros());

  delay(100);
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    input += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

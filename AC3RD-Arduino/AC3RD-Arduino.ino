#include <Servo.h>


//define pins
#include "RotaryEncoder.h"
#define analogPinForRV 1
#define analogPinForTMP 0 

Servo myservo; //create servo object
int tailPosition=90; //variable to store servo position

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
double tailAngle;

RotaryEncoder encoder(11, 12);


void setup(){
  Serial.begin(57600);
  
  //rotary interrupt
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.
  
  //attach servo
  myservo.attach(9);
  
}

ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
}


void loop(){
  //wind sensor- arduino code from https://github.com/moderndevice/Wind_Sensor/blob/master/WindSensor/WindSensor.ino 
  //temp info might unnecessary
      if (millis() - lastMillis > 200){      // read every 200 ms - printing slows this down further
    
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
   
      //Serial.print("  TMP volts ");
      //Serial.print(TMP_Therm_ADunits * 0.0048828125);
    
      //Serial.print(" RV volts ");
      //Serial.print((float)RV_Wind_Volts);

      //Serial.print("\t  TempC*100 ");
      //Serial.print(TempCtimes100 );

        Serial.print("   ZeroWind volts ");
        Serial.print(zeroWind_volts);

        Serial.print("   WindSpeed MPH ");
        Serial.println((float)WindSpeed_MPH);
        lastMillis = millis();    
      }

    //read rotary encoder 
  static int Rotarypos = 0;

  int newRotaryPos = encoder.getPosition();
  if (Rotarypos != newRotaryPos) {
    Serial.print(newRotaryPos);
    Serial.println();
    Rotarypos = newRotaryPos;
  
  //servo set angle
 //myservo.write(tailPosition);
 
  if (Serial.available() > 0) {
      tailPosition=Serial.parseInt();
      myservo.write(tailPosition);
      myservo.write(Serial.parseInt());
      Serial.println("Tail Angle");
      Serial.println (tailPosition);
      }
      
    }

  
}


  //myservo.write(desiredTail);
  //delay(1000);
  //desiredTail = desiredTail + 5;


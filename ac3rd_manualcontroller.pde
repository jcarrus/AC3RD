import net.java.games.input.*;
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;

import processing.serial.*;

PrintWriter textLog; //create a log for our data
String [] headerList = {"GyroX","GyroY","GyroZ","CompassAX","CompassAY","CompassAZ","CompassMX","CompassMY","CompassMZ","Pot1Angle","Pot2Angle","LatDeg","LongDeg","Velocity","fixQuality","windspeedMPH","tailAngle","rudderAngle","autoControl"};
Serial myPort;  // Create object from Serial class
String inString = " , , , , , , , , , , , , , , , , , , , ";  // Data received from the serial port

ControlIO control; //set up controller object
ControlDevice ac3rd_controller2; //our controller
float AOA; //tail angle from controller
float R_A; //rudder angle from controller
int autoSwitch = 0;
int tailOffset = 20;
int rudderOffset = 45;

int timer;

void setup() 
{
  fullScreen();
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 19200);
  myPort.bufferUntil('\n');

  String logString = "AC3RDlog_"+str(hour())+"_"+str(minute())+"_"+str(second())+"_"+str(day())+"_"+str(month())+"_"+str(year())+".txt";
  textLog = createWriter(logString);
  
  control = ControlIO.getInstance(this);
  ac3rd_controller2 = control.getMatchedDevice("ac3rd_controller2");
  if (ac3rd_controller2 == null) {
    println("No suitable device configured");
    System.exit(-1); //exit if no controller found
  }
}

void draw() {
    getUserInput();    
    background(0);
    textSize(40);
    textAlign(CENTER,CENTER);
    fill(255);
    stroke(255);
    strokeWeight(5);
    String[] dataList = split(inString, ',');
    
    int heightInterval = height/(dataList.length);
    int widthInterval = width/4;
    
    line(width/2,0,width/2,height);
    
    for (int i = 0; i <= 17; i++){
      line(0,heightInterval*i,width,heightInterval*i);
      text(headerList[i],widthInterval,heightInterval*(i+1)-heightInterval/2);
      text(dataList[i],widthInterval*3,heightInterval*(i+1)-heightInterval/2);
    }
    line(0,heightInterval*18,width,heightInterval*18);
    text(headerList[18],widthInterval,heightInterval*(18+1)-heightInterval/2);
    text(dataList[18],widthInterval*3,heightInterval*(18+1)+heightInterval/4);
    
    String AOA_string = nf(int(AOA),2);
    String R_A_string = nf(int(R_A),2);
    String input = AOA_string+','+R_A_string+','+autoSwitch+'\n';
    if (millis()-timer > 5){
      timer=millis();
      myPort.write(input);
    }
}

public void getUserInput(){
  AOA = map(ac3rd_controller2.getSlider("AOA").getValue(), -1, 1, 0, 40);
  R_A = map(ac3rd_controller2.getSlider("R_A").getValue(), -1, 1, 0, 90);
}

void serialEvent(Serial p) { 
  inString = p.readString(); 
  textLog.print(inString);
} 

void keyPressed(){
  if (key == ENTER || key == RETURN){
    if (autoSwitch == 0){
      autoSwitch = 1;
    }
    else {
      autoSwitch = 0;
    }
  }
  if (key == ESC){
    textLog.flush();
  exit();
  }
}
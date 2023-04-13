
#include "TimerOne.h"
#include <Wire.h>
#include <PID_v1.h>


#define num_motors 2
#define id 2

int ENCA[] = {2,18};
int ENCB[] = {3, 19};
int pinPWM[] = {5,7,9};
int pinDIR[] = {6,8,10};

volatile long pos_i[] = {0,0,0};
volatile float velocity_i[] = {0,0,0};
volatile float external_speed[] = {0,0,0};
volatile float external_angle[] = {0,0,0};
volatile float tours[] = {0,0,0};
volatile long prevT_i[] = {0,0,0};
double setpoint[] = {0,0,0};
double input[] = {0,0,0};
double output[] = {0,0,0};
volatile double input_copy[] = {0,0,0};


float resolution = 0.01;
int reduction = 172;
boolean isPID = false;
double Kp=2500, Ki=0.2, Kd=0.001;
PID myPID1(&input[0], &output[0], &setpoint[0], Kp, Ki, Kd, DIRECT);
PID pids[]={myPID1,myPID1,myPID1};

void ISR_timerone() { 
  for (int i = 0; i < num_motors; i++) {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" input ");
      Serial.print(" ");
      Serial.print(input[i]);
      Serial.print(" - set point ");
      Serial.print(" ");
      Serial.print(setpoint[i]);
      Serial.print(" - output ");
      Serial.print(" ");
      Serial.println(output[i]);
  }
}

void setup() {
  Wire.begin(id);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);    // start serial for output

  for (int i = 0; i < num_motors; i++) {
    pinMode(ENCA[i],INPUT);
    pinMode(ENCB[i],INPUT);
    pinMode(pinPWM[i],OUTPUT);
    pinMode(pinDIR[i],OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[1]), readEncoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[2]), readEncoder3,RISING);
  
  Timer1.initialize(2000000);
  Timer1.attachInterrupt(ISR_timerone);
  
  for (int i = 0; i < num_motors; i++) {
    pids[i] = PID(&input[i], &output[i], &setpoint[i], Kp, Ki, Kd, DIRECT);
    pids[i].SetMode(AUTOMATIC); // set the mode to automatic
    pids[i].SetOutputLimits(-255, 255); // set the output limits
  }
  delay(1000);
}

void loop() {
  if(Serial.available()>0){
    char command = Serial.read();
    if(command == 'b') {
      int index = Serial.parseInt()-1;
      setMotor(0, pinDIR[index], pinPWM[index]);
      isPID = false;
    }
    else if (command == 'l'){
      int index = Serial.parseInt()-1;
      setMotor(-255, pinDIR[index], pinPWM[index]);
      Serial.print("command");
      Serial.println(pinDIR[index]);
      isPID = false;
    }
    else if (command == 'r'){
      int index = Serial.parseInt()-1;
      setMotor(255, pinDIR[index], pinPWM[index]);
      isPID = false;
    }
    else if (command == 'a'){
      isPID = true;
    }
    else if (command == 's'){
      String string = Serial.readString();
      for (int i = 0; i < num_motors; i++) {
        setpoint[i] = getValue(string, ',', i).toFloat();  
      }
      Serial.println(string);
    }
  }
  
  noInterrupts();
  for (int i = 0; i < num_motors; i++) {
    input[i] = input_copy[i];  
  }
  interrupts();
  
  for (int i = 0; i < num_motors; i++) {
    pids[i].Compute();
  } 
 
  if(isPID){
    for (int i = 0; i < num_motors; i++) {
      //Serial.println(output[i]);
      setMotor(output[i], pinDIR[i], pinPWM[i]);
    }  
  } 
}



boolean checkIfDone(){
  for (int i = 0; i < num_motors; i++) {
    if(abs(setpoint[i] - input[i]) > resolution){
      return false;
    }
  } 
  return true;
}


void readEncoder(int i){
  int b = digitalRead(ENCB[i]);
  int increment = 0;
  if(b>0){
    increment = 1;
  }
  else{
    increment = -1;
  }
  pos_i[i] = pos_i[i] + increment;
  tours[i] = pos_i[i]/12.0/reduction;
  input_copy[i] = tours[i];
  external_angle[i] = (pos_i[i] /12.0)*2*PI / reduction ; 
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i[i]))/1.0e6;
  velocity_i[i] = increment/deltaT;
  velocity_i[i] =(velocity_i[i] / 12)*2*PI;
  external_speed[i] = velocity_i[i]/reduction ;
  prevT_i[i] = currT;
}

void setMotor(int pwmVal, int pinDIR, int pinPWM){
  int dir = 1;
  if(pwmVal != 0) {
    dir = abs(pwmVal)/pwmVal;
    pwmVal = abs(pwmVal);
  }
  analogWrite(pinPWM,pwmVal); // Motor speed
  if(dir == 1){ 
    digitalWrite(pinDIR,HIGH);
  }
  else if(dir == -1){
    digitalWrite(pinDIR,LOW);
  }
}

 String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}


void readEncoder1(){
  readEncoder(0);
}
void readEncoder2(){
  readEncoder(1);
}
void readEncoder3(){
  readEncoder(2);
}


// function that executes whenever data is received from master
void receiveEvent() {
//  input_string = "";
//  while (Wire.available()) { // loop through all but the last
//    char c = Wire.read();       // receive byte as a character
//    //Serial.println(c);
//    input_string += c;
//  }
//  Serial.println(input_string);
}

// function that executes whenever data is requested by master
void requestEvent() {
  //Wire.write(value); // respond with message of 1 byte
}
void handle_input(){
  
}

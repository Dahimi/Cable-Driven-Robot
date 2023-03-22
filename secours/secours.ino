
#include <util/atomic.h>
#include "TimerOne.h"

// Pins
#define ENCA 2
#define ENCB 3
#define pinPWM 5
#define pinDIR 6


float tableau[300];
float times[300];
// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile float external_speed = 0;
volatile float external_angle = 0;
volatile float angle_i = 0;
volatile long prevT_i = 0;
int reduction = 172;
int dir = 1;
int pwr = 200;
int count = 0;
int j = 0;

void ISR_timerone() {  
  Timer1.detachInterrupt();
  Serial.print("tours ");
  Serial.println(pos_i/12);
  
  Timer1.attachInterrupt(ISR_timerone);
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(pinPWM,OUTPUT);
  pinMode(pinDIR,OUTPUT);
  //Timer1.initialize(100000);
  //Timer1.attachInterrupt(ISR_timerone);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder,RISING);
  delay(2000);
  setMotor(-1,255);
  delay(1000);
//  Serial.println(j);
//  for(int i = 0 ; i < 300 ; i++){
//    Serial.print(",");
//    Serial.print(tableau[i]);
//  }
//  Serial.println();
//  Serial.println("time");
//  for(int i = 0 ; i < 300 ; i++){
//    Serial.print(",");
//    Serial.print(times[i]);
//  }
//  delay(5000);
//  j = 0 ;
//  delay(1000);
//  Serial.println(j);
//  for(int i = 0 ; i < 300 ; i++){
//    Serial.print(",");
//    Serial.print(tableau[i]);
//  }
//  Serial.println();
//  Serial.println("time");
//  for(int i = 0 ; i < 300 ; i++){
//    Serial.print(",");
//    Serial.print(times[i]);
//  }
}

void loop() {
  
}

void setMotor(int dir, int pwmVal){
  analogWrite(pinPWM,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(pinDIR,HIGH);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(pinDIR,LOW);
  }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;
  angle_i = (pos_i / 12)*2*PI;
  external_angle = angle_i / reduction ; 
  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  velocity_i =(velocity_i / 12)*2*PI;
  external_speed = velocity_i/reduction ;
  prevT_i = currT;
  if(j < 300){
    tableau[j] = velocity_i;
    times[j] = currT;
  }
  j++;
  
}

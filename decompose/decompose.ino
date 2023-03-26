
#include "DueTimer.h"
#include <PID_v1.h>

// Pins
#define ENCA1 52
#define ENCB1 53
#define pinPWM1 5
#define pinDIR1 6

#define ENCA2 50
#define ENCB2 51
#define pinPWM2 7
#define pinDIR2 8


#define num_motors 1

int num_targets, current_target_index ;
float targets[num_motors][200];
float resolution = 0.01;
// Use the "volatile" directive for variabless
// used in an interrupt
volatile int pos_i1 = 0, pos_i2 = 0;
String targetString = "";
volatile float velocity_i1 = 0,velocity_i2 = 0;;
volatile float external_speed1 = 0, external_speed2 = 0;;
volatile float external_angle1 = 0, external_angle2 = 0;;
volatile float tours1 = 0, tours2 = 0;
volatile long prevT_i1 = 0, prevT_i2 = 0;;
int reduction = 172;
boolean isPID = false;
volatile double input1_copy, input2_copy;
double setpoint1,input1, output1;
double setpoint2,input2, output2;
double Kp=2500, Ki=0.2, Kd=0.001;
PID myPID1(&input1, &output1, &setpoint1, Kp, Ki, Kd, DIRECT);
PID myPID2(&input2, &output2, &setpoint2, Kp, Ki, Kd, DIRECT);


void ISR_timerone() {  
  Serial.print("tours ");
  Serial.println(tours1);
  Serial.print("input");
  Serial.println(input1);
  Serial.print("set point ");
  Serial.println(setpoint1);
  Serial.print("is Done ? ");
  Serial.println(checkIfDone());
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCA1,INPUT);
  pinMode(ENCB1,INPUT);
  pinMode(pinPWM1,OUTPUT);
  pinMode(pinDIR1,OUTPUT);

  pinMode(ENCA2,INPUT);
  pinMode(ENCB2,INPUT);
  pinMode(pinPWM2,OUTPUT);
  pinMode(pinDIR2,OUTPUT);
  
  Timer3.attachInterrupt(ISR_timerone).start(1000000);
  
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2,RISING);
  
  setpoint1 = 1;
  setpoint2 = 2;
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-255,255);
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-255,255);
  delay(1000);
}

void loop() {
  if(Serial.available()>0){
    char command = Serial.read();
    if(command == 'b') {
      setMotor(0, pinDIR1, pinPWM1);
      isPID = false;
    }
    else if (command == 'l'){
      setMotor(-255, pinDIR1, pinPWM1);
      isPID = false;
    }
    else if (command == 'r'){
      setMotor(255, pinDIR1, pinPWM1);
      isPID = false;
    }
    else if (command == 'a'){
      isPID = true;
    }
    else if (command == 's'){
      current_target_index = 0;
      String string = Serial.readString();
      num_targets = getValue(string, '-', 0).toInt();
      targetString = getValue(string, '-', 1);
      decompose_target(targetString);
      Serial.println(targetString);
    }
  }
  
  noInterrupts();
  input1 = input1_copy;
  input2 = input2_copy;
  interrupts();
  
  update_setpoint1();
  myPID1.Compute();
  if(isPID){
    setMotor(output1, pinDIR1, pinPWM1);
  } 
}



boolean checkIfDone(){
  return abs(setpoint1 - input1) <= resolution;
}

void update_setpoint1(){
  
  if(checkIfDone() && current_target_index < num_targets){
    setpoint1 = targets[0][current_target_index];
    current_target_index++;
  }
}
void decompose_target(String targetString){
  for(int j = 0 ; j < num_motors ; j++){
    String  string_j = getValue(targetString, ',', j);
    for(int i = 0 ; i < num_targets ; i++){
     targets[j][i]= getValue(string_j, ' ', i).toFloat();
  }    
  }
  
  update_setpoint1();
}


void readEncoder1(){
  int b = digitalRead(ENCB1);
  int increment = 0;
  if(b>0){
    increment = 1;
  }
  else{
    increment = -1;
  }
  pos_i1 = pos_i1 + increment;
  tours1 = pos_i1/12.0/reduction;
  input1_copy = tours1;
  external_angle1 = (pos_i1 /12.0)*2*PI / reduction ; 
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i1))/1.0e6;
  velocity_i1 = increment/deltaT;
  velocity_i1 =(velocity_i1 / 12)*2*PI;
  external_speed1 = velocity_i1/reduction ;
  prevT_i1 = currT;
}

void readEncoder2(){
  int b = digitalRead(ENCB2);
  int increment = 0;
  if(b>0){
    increment = 1;
  }
  else{
    increment = -1;
  }
  pos_i2 = pos_i2 + increment;
  tours2 = pos_i2/12.0/reduction;
  input2_copy = tours2;
  external_angle2 = (pos_i2 /12.0)*2*PI / reduction ; 
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i2))/1.0e6;
  velocity_i2 = increment/deltaT;
  velocity_i2 =(velocity_i2 / 12)*2*PI;
  external_speed2 = velocity_i2/reduction ;
  prevT_i2 = currT;
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

#include "TimerOne.h"
#include <Wire.h>
#include <PID_v1.h>


#define num_motors 2

int ENCA[] = {2,18};
int ENCB[] = {3, 19};
int pinPWM[] = {5, 7};
int pinDIR[] = {6,8};
volatile long pos_i[] = {0,0};
volatile float velocity_i[] = {0,0};
volatile float external_speed[] = {0,0};
volatile float external_angle[] = {0,0};
volatile float tours[] = {0,0};
volatile long prevT_i[] = {0,0};
double setpoint[] = {0,0};
double input[] = {0,0};
double output[] = {0,0};
volatile double input_copy[] = {0,0};

int num_targets, current_target_index ;
float targets[8][200];
float resolution = 0.01;
String targetString = "";
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
  //Serial.print("is Done ? ");
  //Serial.println(checkIfDone());
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  for (int i = 0; i < num_motors; i++) {
    pinMode(ENCA[i],INPUT);
    pinMode(ENCB[i],INPUT);
    pinMode(pinPWM[i],OUTPUT);
    pinMode(pinDIR[i],OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[1]), readEncoder2,RISING);
  
  Timer1.initialize(1000000);
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
      if(index < 2){
      setMotor(0, pinDIR[index], pinPWM[index]);
      isPID = false;
      }
      else{
        share_command(index , command);
      }
    }
    else if (command == 'l'){
      int index = Serial.parseInt()-1;
      if(index < 2){
      setMotor(-255, pinDIR[index], pinPWM[index]);
      isPID = false;
      }
      else{
        share_command(index , command);
      }
    }
    else if (command == 'r'){
      int index = Serial.parseInt()-1;
      if(index < 2){
      setMotor(255, pinDIR[index], pinPWM[index]);
      isPID = false;
      }
      else{
        share_command(index , command);
      }
    }
    else if (command == 'a'){
      isPID = true;
      send_to_slave(1,"a");
      send_to_slave(2,"a");
    }
    else if (command == 's'){
      current_target_index = 0;
      String string = Serial.readString();
      num_targets = getValue(string, '-', 0).toInt();
      targetString = getValue(string, '-', 1);
      decompose_target(targetString);
    }
  }
  
  noInterrupts();
  for (int i = 0; i < num_motors; i++) {
    input[i] = input_copy[i];  
  }
  interrupts();
  
  update_setpoint();
  for (int i = 0; i < num_motors; i++) {
    pids[i].Compute();
  } 
 
  if(isPID){
    for (int i = 0; i < num_motors; i++) {
      setMotor(output[i], pinDIR[i], pinPWM[i]);
    }  
  } 
}


void share_command(int index,char command ){
  index = index - 2;
  int quotient = index/2;
  int new_index = index - 2*quotient + 1;
  int id = quotient + 1;
  String shared_string = "";
  shared_string +=command;
  shared_string +=String(new_index);
  send_to_slave(id,shared_string);
}

boolean checkIfDone(){
  for (int i = 0; i < num_motors; i++) {
    if(abs(setpoint[i] - input[i]) > resolution){
      return false;
    }
  }
  return check_slave_done(1) && check_slave_done(2);
  
}

boolean check_slave_done(int id){
  return request_from_slave(id) == 1;
}

void update_setpoint(){
  
  if(checkIfDone() && current_target_index < num_targets){
    for (int i = 0; i < num_motors; i++) {
      setpoint[i] = targets[i][current_target_index];  
    }
    current_target_index++;
  }
}

void update_slaves(){
  
}

void decompose_target(String targetString){
  for(int j = 0 ; j < 8 ; j++){
    String  string_j = getValue(targetString, ',', j);
    Serial.println(string_j);
    for(int i = 0 ; i < num_targets ; i++){
     targets[j][i]= getValue(string_j, ' ', i).toFloat();
  }    
  }
  for (int i = 0; i < num_motors; i++) {
      setpoint[i] = targets[i][current_target_index];  
    }
  update_slaves();
    
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



void send_to_slave(int id, String str){
  int str_len = str.length() + 1; 
  char char_array[str_len];
  str.toCharArray(char_array, str_len);
  Wire.beginTransmission(id);
  Wire.write(char_array);        
  Wire.endTransmission();
  Serial.print(str);
  Serial.println(" done");
}

byte request_from_slave(int id){
//  Wire.requestFrom(id, 1);
//  byte c;
//  while (Wire.available()) { // slave may send less than requested
//    c = Wire.read(); // receive a byte as character
//    Serial.print("Slave value : ");
//    Serial.println(c);         // print the character
//  }
//  return c;

    return 1;
}

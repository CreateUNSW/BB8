#include <Servo.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

//RC channels
#define CH1 18
#define CH2 19
#define CH3 2
#define CH4 3
#define CH5 20
#define CH6 21

// CH1 right up-down

int ch1_value = 1500, ch2_value = 1500, ch3_value = 1500, ch4_value = 1500, ch5_value = 1500, ch6_value = 1500;
bool ch1_ready = false, ch2_ready = false, ch3_ready = false, ch4_ready = false, ch5_ready = false, ch6_ready = false;
int ch1_value_start, ch2_value_start, ch3_value_start, ch4_value_start, ch5_value_start, ch6_value_start;

//PINS and addresses
#define ST_PITCH_ROLL_PIN 11
#define ST_HEAD_YAW_PIN 12

#define SERVO_PITCH 10
#define SERVO_ROLL 9

#define BNO_BODY 0x28
#define BNO_HEAD 0x29

// motor ids
#define PITCH_MOTOR 1
#define ROLL_MOTOR 2

#define YAW_MOTOR 1
#define HEAD_MOTOR 2

// ROLL PID
#define ROLL_P 2
#define ROLL_I 5
#define ROLL_D 1

// PITCH PID
#define PITCH_P 12
#define PITCH_I 2
#define PITCH_D 1

// ROLL PID
#define HEAD_ROLL_P 0.4
#define HEAD_ROLL_I 0
#define HEAD_ROLL_D 0

// PITCH PID
#define HEAD_PITCH_P 12
#define HEAD_PITCH_I 2
#define HEAD_PITCH_D 1

// PITCH PID
#define HEAD_YAW_P 12
#define HEAD_YAW_I 2
#define HEAD_YAW_D 1

//Define Variables we'll be connecting to
double rollSetpoint = 0,rollInput,rollOutput;
double pitchSetpoint = 0,pitchInput,pitchOutput;
// for head
double headRollSetpoint = 0,headRollInput,headRollOutput;
double headPitchSetpoint = 0,headPitchInput,headPitchOutput;
double headYawSetpoint = 0,headYawInput,headYawOutput;

Servo s_roll, s_pitch;

//PID
// change the values
//order of Ks : p,i,d
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, ROLL_P, ROLL_I, ROLL_D, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, PITCH_P, PITCH_I, PITCH_D, DIRECT);

//head PIDs
PID headRollPID(&headRollInput, &headRollOutput, &headRollSetpoint, HEAD_ROLL_P, HEAD_ROLL_I, HEAD_ROLL_D, DIRECT);
PID headPitchPID(&headPitchInput, &headPitchOutput, &headPitchSetpoint, HEAD_PITCH_P, HEAD_PITCH_I, HEAD_PITCH_D, DIRECT);
PID headYawPID(&headYawInput, &headYawOutput, &headYawSetpoint, HEAD_YAW_P, HEAD_YAW_I, HEAD_YAW_D, DIRECT);

//everything for Sabre tooth
SoftwareSerial Pin1(NOT_A_PIN, ST_PITCH_ROLL_PIN); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified st_pitch_roll(Pin1); // Use SWSerial as the serial port.
SoftwareSerial Pin2(NOT_A_PIN, ST_HEAD_YAW_PIN); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified st_head_yaw(Pin2); // Use SWSerial as the serial port.

//BNO stuff
sensors_event_t bodyEvent,headEvent;
Adafruit_BNO055 bodyBno = Adafruit_BNO055(BNO_BODY,BNO_BODY);
Adafruit_BNO055 headBno = Adafruit_BNO055(BNO_HEAD,BNO_HEAD);

//flag to check if func needs to be called
bool call = false;

////stuff to handle serial input
//int input = 0;
//bool inputComplete = false;

//call this every 100ms before any other func
void funcStart( ) {
  bodyBno.getEvent(&bodyEvent);
  headBno.getEvent(&headEvent);
}

// call this after every other func
void funcEnd() {
  Serial.println(" > ");
  call = false;
}

void pitchCalc(void)
{

  //NEED TO BE CHANGED ACCORDING TO ORIENTATION
  pitchInput = (float)bodyEvent.orientation.y;
  pitchPID.Compute();
  st_pitch_roll.motor(PITCH_MOTOR,pitchOutput);
}

void rollCalc(void) {
  //NEED TO BE CHANGED ACCORDING TO ORIENTATION
  rollInput = bodyEvent.orientation.x;
  rollPID.Compute();
  st_pitch_roll.motor(ROLL_MOTOR,rollOutput);
}

void headPitchCalc(void)
{
  //NEED TO BE CHANGED ACCORDING TO ORIENTATION
  headPitchInput = (float)headEvent.orientation.y;
  headPitchPID.Compute();
  s_pitch.write(headPitchOutput);
}

void headRollCalc(void) {
  //NEED TO BE CHANGED ACCORDING TO ORIENTATION
  headRollInput = -(float)headEvent.orientation.x;
  Serial.print(headRollInput);
  Serial.print(" ");
  headRollPID.Compute();
  Serial.print(headRollOutput);
  Serial.print(" ");
  s_roll.write(headRollOutput);
  
}

void headYawCalc(void) {
  //NEED TO BE CHANGED ACCORDING TO ORIENTATION
  headYawInput = headEvent.orientation.z;
  headYawPID.Compute();
  st_head_yaw.motor(HEAD_MOTOR, headYawOutput);
}

void Interrupt() {
  call = true;
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(CH1,INPUT);
  pinMode(CH2,INPUT);
  pinMode(CH3,INPUT);
  pinMode(CH4,INPUT);
  pinMode(CH5,INPUT);
  pinMode(CH6,INPUT);

 attachInterrupt(digitalPinToInterrupt(CH1),ch1_handler,CHANGE);
 attachInterrupt(digitalPinToInterrupt(CH2),ch2_handler,CHANGE);
 attachInterrupt(digitalPinToInterrupt(CH3),ch3_handler,CHANGE);
// attachInterrupt(digitalPinToInterrupt(CH4),ch4_handler,CHANGE);
// attachInterrupt(digitalPinToInterrupt(CH5),ch5_handler,CHANGE);
// attachInterrupt(digitalPinToInterrupt(CH6),ch6_handler,CHANGE);

  Pin1.begin(9600);
  Pin2.begin(9600);

  Serial.begin(9600);
  Serial.println("Starting..."); Serial.println("");

//  s_roll.attach(SERVO_ROLL);
//  s_pitch.attach(SERVO_PITCH);

//  if(!headBno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no HEAD BNO detected ... Check your wiring or I2C ADDR!");
//  }
//  if(!bodyBno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no BODY BNO detected ... Check your wiring or I2C ADDR!");
//  }

//  pitchPID.SetOutputLimits(-127,127);
//  rollPID.SetOutputLimits(-127,127);
//  headYawPID.SetOutputLimits(-127,127);
//
//  headPitchPID.SetOutputLimits(0,180);
//  headRollPID.SetOutputLimits(0,180);

//  pitchPID.SetMode(AUTOMATIC);
//  rollPID.SetMode(AUTOMATIC);
////  headRollPID.SetMode(AUTOMATIC);
////  headPitchPID.SetMode(AUTOMATIC);
////  headYawPID.SetMode(AUTOMATIC);
//  Timer1.initialize(100000);
//  Timer1.attachInterrupt(Interrupt);
}


void loop() {

  
  Serial.print(map(ch1_value,1000,2000,-120,120));
  Serial.print(" ");
  Serial.print(map(ch2_value,1000,2000,-120,120));
  Serial.print(" ");
  Serial.print(map(ch3_value,1000,2000,-120,120));
  Serial.print(" ");
  Serial.print(map(ch4_value,1000,2000,-120,120));
  Serial.print(" ");
  Serial.print(ch5_value);
  Serial.print(" ");
  Serial.print(ch6_value);
  Serial.println(" ");
  delay(100);
  st_pitch_roll.motor(1,map(ch1_value,1000,2000,-120,120));
  
  st_head_yaw.motor(2,map(ch2_value,1000,2000,-120,120));
  st_head_yaw.motor(1,map(ch3_value,1000,2000,-120,120));
  if(call) {
//    funcStart();
//    pitchCalc();
//    rollCalc();
//    headPitchCalc();
//    headRollCalc();
//    headYawCalc();
//    funcEnd();
  }
}

//void serialEvent() {
//  while (Serial.available()) {
//    // get the new byte:
//    char inChar = (char)Serial.read();
//    if (inChar == '\n') {
//      inputComplete = true;
//    } else {
//      int digit = inChar - '0';
//      input = input*10 + digit;
//    }
//  }
//}
void ch1_handler() {
  if(digitalRead(CH1) == HIGH)
  {
    ch1_value_start = micros();
  }
  else
  {
    ch1_value = (uint16_t)(micros() - ch1_value_start);
    ch1_ready = true;
  }
}

void ch2_handler() {
  if(digitalRead(CH2) == HIGH)
  {
    ch2_value_start = micros();
  }
  else
  {
    ch2_value = (uint16_t)(micros() - ch2_value_start);
    ch2_ready = true;
  }
}
void ch3_handler() {
  if(digitalRead(CH3) == HIGH)
  {
    ch3_value_start = micros();
  }
  else
  {
    ch3_value = (uint16_t)(micros() - ch3_value_start);
    ch3_ready = true;
  }
}
void ch4_handler() {
  if(digitalRead(CH4) == HIGH)
  {
    ch4_value_start = micros();
  }
  else
  {
    ch4_value = (uint16_t)(micros() - ch4_value_start);
    ch4_ready = true;
  }
}

void ch5_handler() {
  if(digitalRead(CH5) == HIGH)
  {
    ch5_value_start = micros();
  }
  else
  {
    ch5_value = (uint16_t)(micros() - ch5_value_start);
    ch5_ready = true;
  }
}
void ch6_handler() {
  if(digitalRead(CH6) == HIGH)
  {
    ch6_value_start = micros();
  }
  else
  {
    ch6_value = (uint16_t)(micros() - ch6_value_start);
    ch6_ready = true;
  }
}

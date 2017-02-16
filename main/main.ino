#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

//PINS and addresses
#define ST_PITCH_ROLL_PIN 11
#define ST_HEAD_YAW_PIN 12

#define BNO_BODY 0x29
#define BNO_HEAD 0x28

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

//Define Variables we'll be connecting to
double rollSetpoint = 0,rollInput,rollOutput;
double pitchSetpoint = 0,pitchInput,pitchOutput;

//everything for Sabre tooth
SoftwareSerial Pin1(NOT_A_PIN, ST_PITCH_ROLL_PIN); // RX on no pin (unused), TX on pin 11 (to S1).
SoftwareSerial Pin2(NOT_A_PIN, ST_HEAD_YAW_PIN); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified st_pitch_roll(Pin1); // Use SWSerial as the serial port.
SabertoothSimplified st_head_yaw(Pin2); // Use SWSerial as the serial port.

//PID
// change the values
//order of Ks : p,i,d
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, ROLL_P, ROLL_I, ROLL_D, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, PITCH_P, PITCH_I, PITCH_D, DIRECT);

sensors_event_t bodyEvent,headEvent;
Adafruit_BNO055 bodyBno = Adafruit_BNO055(BNO_BODY,BNO_BODY);
Adafruit_BNO055 headBno = Adafruit_BNO055(BNO_HEAD,BNO_HEAD);

//flag to check if func needs to be called
bool call = false;

//stuff to handle serial input
int input = 0;
bool inputComplete = false;

//call this every 100ms before any other func
void funcStart( ) {
  bodyBno.getEvent(&bodyEvent);
  headBno.getEvent(&headEvent);
  Serial.print("test");
  if(inputComplete) {
    pitchSetpoint = input;
    input = 0;
    inputComplete = false;
  }
}

// call this after every other func
void funcEnd() {
//  Serial.print("Pitch Set Point> ");
//  Serial.println(pitchSetpoint);
}

void pitchCalc(void)
{

  //NEED TO BE CHANGED ACCORDING TO ORIENTATION
  pitchInput = (float)bodyEvent.orientation.y;
  Serial.print("z: ");
  Serial.print(pitchInput);
  pitchPID.Compute();
  Serial.print(" MAIN OUTPUT: ");
  Serial.println(pitchOutput);
  st_pitch_roll.motor(PITCH_MOTOR,pitchOutput);
}

void rollCalc(void) {
  //NEED TO BE CHANGED ACCORDING TO ORIENTATION
  rollInput = bodyEvent.orientation.x;
  rollPID.Compute();
  Serial.print("SIDE OUTPUT: ");
  Serial.println(rollOutput);
//  st_pitch_roll.motor(ROLL_MOTOR,rollOutput);
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

  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

//  if(!headBno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no MAIN BNO detected ... Check your wiring or I2C ADDR!");
//    while(1);
//  }
  if(!bodyBno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BODY BNO detected ... Check your wiring or I2C ADDR!");
    while(1);
  }


  pitchPID.SetOutputLimits(-127,127);
  pitchPID.SetMode(AUTOMATIC);
//  rollPID.SetMode(AUTOMATIC);
  Timer1.initialize(100000);
  Timer1.attachInterrupt(Interrupt);
}


void loop() {
  if(call) {
    funcStart();
     pitchCalc();
//     rollCalc();
    funcEnd();
    call = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      inputComplete = true;
    } else {
      int digit = inChar - '0';
      input = input*10 + digit;
    }
  }
}

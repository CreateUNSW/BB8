#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Sabertooth.h>
#include <Encoder.h>
#include <TimerOne.h>
#include <Math.h>

#define SAMPLE_TIME 5000
#define TIMEOUT 100

#define BB8_MANUAL 0
#define BB8_SPIN 1
#define BB8_SPINNING 2
#define BB8_AI 3

bool BluetoothEnabled = 1;

#if BluetoothEnabled
  #include <usbhub.h>
  #include <PS3BT.h>
#endif

#if !BluetoothEnabled
  #include <usbhub.h>
  #include <PS3USB.h>
#endif

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

#if BluetoothEnabled
  USB Usb;
  //USBHub Hub1(&Usb); // Some dongles have a hub inside
  BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
  PS3BT PS3(&Btd, PAIR);
#endif

#if !BluetoothEnabled
  USB Usb;
  PS3USB PS3(&Usb);
  //PS3USB PS3(&Usb,0x00,0x15,0x83,0x3D,0x0A,0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch
#endif

Sabertooth ST(128, Serial1);

// Controller Heartbeat
int Heartbeat = 0; 

int ControlMode = BB8_MANUAL;

// Body yaw variables
int BodyHeading = 0;
int desiredSpin = 0;

//Define constants for Body Motors
int motor_Body_P = 1;
int motor_Body_R = 2;
int motorDir_Body_Y = 3;
int motorSpeed_Body_Y = 4;

//Define Pins for Head Motors
int motorSpeed_Head_P = 5;
int motorDir_Head_P = 6;
int motorSpeed_Head_R = 7;
int motorDir_Head_R = 8;
int motorSpeed_Head_Y = 9;
int motorDir_Head_Y = 10;

//Define Variables we'll be connecting to for Body PID
double Setpoint_Body_P, Input_Body_P, Output_Body_P;
double Setpoint_Body_R, Input_Body_R, Output_Body_R;
double Setpoint_Body_Y, Input_Body_Y, Output_Body_Y;

//Define Variables we'll be connecting to for Head PID
double Setpoint_Head_P, Input_Head_P, Output_Head_P;
double Setpoint_Head_R, Input_Head_R, Output_Head_R;
double Setpoint_Head_Y, Input_Head_Y, Output_Head_Y;

//Specify the links and initial tuning parameters for Body PID
double Kp_Body_P = 3.66, Ki_Body_P = 0.2, Kd_Body_P = 0.05;
double Kp_Body_R = 3.66, Ki_Body_R = 0.2, Kd_Body_R = 0.05;
double Kp_Body_Y = 3.66, Ki_Body_Y = 0.2, Kd_Body_Y = 0.05;

//Specify the links and initial tuning parameters for Head PID
double Kp_Head_P = 3.66, Ki_Head_P = 0.2, Kd_Head_P = 0.05;
double Kp_Head_R = 3.66, Ki_Head_R = 0.2, Kd_Head_R = 0.05;
double Kp_Head_Y = 3.66, Ki_Head_Y = 0.2, Kd_Head_Y = 0.05;

// Encoder Count Variables
long newPosition;
long oldPosition;

// PID Constructors Body
PID PID_Body_P(&Input_Body_P, &Output_Body_P, &Setpoint_Body_P, Kp_Body_P, Ki_Body_P, Kd_Body_P, DIRECT);
PID PID_Body_R(&Input_Body_R, &Output_Body_R, &Setpoint_Body_R, Kp_Body_R, Ki_Body_R, Kd_Body_R, DIRECT);
PID PID_Body_Y(&Input_Body_Y, &Output_Body_Y, &Setpoint_Body_Y, Kp_Body_Y, Ki_Body_Y, Kd_Body_Y, DIRECT); // Desired spin change PID output feeds into PID_Reaction

// PID Constructors Head
PID PID_Head_P(&Input_Head_P, &Output_Head_P, &Setpoint_Head_P, Kp_Head_P, Ki_Head_P, Kd_Head_P, DIRECT);
PID PID_Head_R(&Input_Head_Y, &Output_Head_R, &Setpoint_Head_R, Kp_Head_R, Ki_Head_R, Kd_Head_R, DIRECT);
PID PID_Head_Y(&Input_Head_Y, &Output_Head_Y, &Setpoint_Head_Y, Kp_Head_Y, Ki_Head_Y, Kd_Head_Y, DIRECT);

// IMU Constructors
Adafruit_BNO055 IMU_Body = Adafruit_BNO055(55,0x28);
Adafruit_BNO055 IMU_Head = Adafruit_BNO055(55,0x29);

// Encoder Constructors
Encoder myEnc(2, 3);  // Feedback on reaction wheel

void setup() {  
  Serial.begin(9600);   // Serial communication to Terminal
  Serial1.begin(9600);  // Serial commmunication to Sabertooth Motor Controller

  #if !defined(__MIPSEL__)
    while (!Serial && !Serial1); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif
  
  /* Initialise the IMU sensors */
  if(!IMU_Body.begin() && !IMU_Head.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("\r\nBNO055 modules not detected");
    while(1); // Halt
  }
  
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }

  IMU_Body.setExtCrystalUse(true);

  // Timer1 ISR initialisation
  Timer1.initialize(SAMPLE_TIME); // set a timer of length 50000 microseconds (or 0.05 sec - or 20Hz)s
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here

  /* Get a new sensor event Body IMU */ 
  sensors_event_t Body_event;
  IMU_Body.getEvent(&Body_event);

  /* Get a new sensor event Head IMU */ 
  sensors_event_t Head_event;
  IMU_Head.getEvent(&Head_event);
  
  // Initialise PID parameters for body pitch
  Input_Body_P = Body_event.orientation.x;
  Setpoint_Body_P = 0; // Desired angle
  PID_Body_P.SetOutputLimits(-127, 127); // Scale PID output
  PID_Body_P.SetSampleTime(SAMPLE_TIME/100); // Set sample time equal to ISR
  PID_Body_P.SetMode(AUTOMATIC);  // PID ON
  
  // Initialise PID parameters for body roll
  Input_Body_R = Body_event.orientation.y;
  Setpoint_Body_R = 0; // Desired angle
  PID_Body_R.SetOutputLimits(-127, 127); // Scale PID output
  PID_Body_R.SetSampleTime(SAMPLE_TIME/100); // Set sample time equal to ISR
  PID_Body_R.SetMode(AUTOMATIC);  // PID ON

  // Initialise PID parameters for body yaws
  Input_Body_Y = 0;
  Setpoint_Body_Y = 0; 
  PID_Body_Y.SetOutputLimits(-255, 255); // Scale PID output
  PID_Body_Y.SetSampleTime(SAMPLE_TIME/100); // Set sample time equal to ISR
  PID_Body_Y.SetMode(MANUAL); // PID OFF

  // Initialise PID parameters for Head Pitch
  Input_Head_P = Head_event.orientation.x;
  Setpoint_Head_P = 0; // Desired angle
  PID_Head_P.SetOutputLimits(-255, 255); // Scale PID output
  PID_Head_P.SetSampleTime(SAMPLE_TIME/100); // Set sample time equal to ISR
  PID_Head_P.SetMode(AUTOMATIC);  // PID ON
  
  // Initialise PID parameters for Head Roll
  Input_Head_R = Head_event.orientation.y;
  Setpoint_Head_R = 0; // Desired angle
  PID_Head_R.SetOutputLimits(-255, 255); // Scale PID output
  PID_Head_R.SetSampleTime(SAMPLE_TIME/100); // Set sample time equal to ISR
  PID_Head_R.SetMode(AUTOMATIC);  // PID ON

  // Initialise PID parameters for Head Yaw
  Input_Head_Y = Head_event.orientation.z;
  Setpoint_Head_Y = Body_event.orientation.z; // Desired angle
  PID_Head_Y.SetOutputLimits(-255, 255); // Scale PID output
  PID_Head_Y.SetSampleTime(SAMPLE_TIME/100); // Set sample time equal to ISR
  PID_Head_Y.SetMode(MANUAL); // PID OFF

  Serial.print(F("\r\nInitiased...Entering main loop"));
  delay(1000);
  ControlMode = BB8_MANUAL; // enter manual control mode
  PS3.setLedOn(LED1);
  PS3.setLedOff(LED2);
  PS3.setLedOff(LED3);
  PS3.setLedOff(LED4);
}

void loop() 
{
  Usb.Task();

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    Heartbeat = 0;  // Reset controller heartbeat

    if(ControlMode == BB8_MANUAL){
      
      if (PS3.getAnalogHat(LeftHatX) > 137 || PS3.getAnalogHat(LeftHatX) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117) {
        Setpoint_Body_P = sqrt(pow(PS3.getAnalogHat(LeftHatX), 2) + pow(PS3.getAnalogHat(LeftHatY), 2));  // Polar radius of L Stick drives body
        Setpoint_Body_R = PS3.getAnalogHat(LeftHatX); // X component of L stick steers body
      } else {
        Setpoint_Body_P = 0;
        Setpoint_Body_R = 0;
      }
  
      if (PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117) {
        Setpoint_Head_P = PS3.getAnalogHat(RightHatY);  // Move head's pitch
        Setpoint_Head_R = PS3.getAnalogHat(RightHatX);  // Move head's roll
      } else {
        Setpoint_Head_P = 0;
        Setpoint_Head_R = 0;
      }

      // Maybe add a speed controller for manual head control (Encoder with PID)
      if (PS3.getAnalogButton(R2) && !PS3.getAnalogButton(L2) && !PID_Head_Y.GetMode()) {  // If only L2 and PID not on
        digitalWrite(motorDir_Head_Y, HIGH);
        analogWrite(motorSpeed_Head_Y, PS3.getAnalogButton(R2));  // Spin head CW
      } else if (PS3.getAnalogButton(L2) && !PS3.getAnalogButton(R2) && !PID_Head_Y.GetMode()) {  // If only R2 and PID not on
        digitalWrite(motorDir_Head_Y, LOW);
        analogWrite(motorSpeed_Head_Y, PS3.getAnalogButton(L2));  // Spin head CCW
      } else if (PS3.getButtonClick(R3) && PID_Head_Y.GetMode()) {  // If PID not already on
        Setpoint_Head_Y = BodyHeading; // align head with body
        PID_Head_Y.SetMode(AUTOMATIC); // Turn head yaw pid on 
      }

      if (PS3.getButtonClick(CIRCLE)){  // Enter spin ball mode
        Setpoint_Body_P = 0;  // Origin all axises of freedom
        Setpoint_Body_R = 0;
        Setpoint_Head_P = 0;
        Setpoint_Head_R = 0;
        Setpoint_Head_Y = BodyHeading; // align head with body
        PID_Head_Y.SetMode(AUTOMATIC); // Turn head yaw pid on

        ControlMode = BB8_SPIN;
        PS3.setLedOn(LED2); //Indicator light for head aligning to body;
        PS3.setLedOff(LED1);
        PS3.setLedOff(LED3);
        PS3.setLedOff(LED4);
      }
      
    } else if (ControlMode == BB8_SPIN) {
      if (!PID_Head_Y.GetMode()) {
        PS3.setLedOn(LED3); // LED indicates head finished aligning
        PS3.setLedOn(LED1);
        PS3.setLedOn(LED2);
        PS3.setLedOn(LED4);
        
        if (PS3.getButtonClick(CIRCLE)) {  // If circle pressed and head finished aligning
          // Generate desired angle origined at vertical -180 to 180 from R-Stick       
          if ((PS3.getAnalogHat(LeftHatX) > 137 && PS3.getAnalogHat(LeftHatY) > 137) || (PS3.getAnalogHat(LeftHatX) < 117 && PS3.getAnalogHat(LeftHatY) < 117)) {  // 1st/4th Quadrant
            desiredSpin = 90 - atan((PS3.getAnalogHat(LeftHatX)-127)/(PS3.getAnalogHat(LeftHatY)-127));
          } else if ((PS3.getAnalogHat(LeftHatX) < 117 && PS3.getAnalogHat(LeftHatY) > 137) || (PS3.getAnalogHat(LeftHatX) < 117 && PS3.getAnalogHat(LeftHatY) < 117)) {  // 2nd/3rd Quadrant
            desiredSpin = -90 - atan((PS3.getAnalogHat(LeftHatX)-127)/(PS3.getAnalogHat(LeftHatY)-127));
          } else {
            desiredSpin = 0;
          }
  
          Setpoint_Body_Y = BodyHeading + desiredSpin;
          
          PID_Body_Y.SetMode(AUTOMATIC);  // Activate reaction wheel PID
          ControlMode = BB8_SPINNING;
          PS3.setLedOn(LED1); // LED indicators while BB8 IS BUSY driving spin
          PS3.setLedOn(LED2);
          PS3.setLedOn(LED3);
          PS3.setLedOn(LED4);
        }   
      }
      
    } else if (ControlMode == BB8_SPINNING) {
      
      if (PID_Body_Y.GetMode()) { // Check if spinning operation complete
        ControlMode = BB8_MANUAL; // return to manual control mode
        PS3.setLedOn(LED1);
        PS3.setLedOff(LED2);
        PS3.setLedOff(LED3);
        PS3.setLedOff(LED4);
      }
      
    } else if (ControlMode == BB8_AI) {
      
      if (PS3.getButtonClick(TRIANGLE)) {
         ControlMode = BB8_MANUAL; // return to manual control mode
        PS3.setLedOn(LED1);
        PS3.setLedOff(LED2);
        PS3.setLedOff(LED3);
        PS3.setLedOff(LED4);
      }
      // AI Code
      
    }
  } else {
    Heartbeat++;  // Increase heartbeat
  }

  if (Heartbeat > TIMEOUT){  // If controller disconnected too long
    Setpoint_Body_P = 0;  // Send BB8 into stabilise steady state mode
    Setpoint_Body_R = 0;
    Setpoint_Head_P = 0;
    Setpoint_Head_R = 0;
    PID_Body_Y.SetMode(MANUAL); // Turn off the reaction wheel PID and motor
    digitalWrite(motorDir_Body_Y, LOW);
    analogWrite(motorSpeed_Body_Y, 0);
    PID_Head_Y.SetMode(MANUAL); // Turn off the head yaw PID and motor
    digitalWrite(motorDir_Head_Y, LOW);
    analogWrite(motorSpeed_Head_Y, 0);

    Serial.println("Controller Disconnected!");
  }
}

void timerIsr()   // Timer Interrupt 20Hz
{
  // Reaction wheel encoder update 
  newPosition = myEnc.read();
  
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
  }
  
  /* Get a new sensor event Body IMU */ 
  sensors_event_t Body_event;
  IMU_Body.getEvent(&Body_event);

  /* Get a new sensor event Head IMU */ 
  sensors_event_t Head_event;
  IMU_Head.getEvent(&Head_event);

  // Update PID Body inputs
  Input_Body_P = Body_event.orientation.x;
  Input_Body_R = Body_event.orientation.y;
  Input_Body_Y = Body_event.orientation.z;
  BodyHeading = Body_event.orientation.z;

  // Update PID Head inputs
  Input_Head_P = Head_event.orientation.x;
  Input_Head_R = Head_event.orientation.y;
  Input_Head_Y = Head_event.orientation.z;
  
  // Begin PID computations for body
  if(PID_Body_P.Compute()) {  // If computation successful and AUTO on
    ST.motor(motor_Body_P, Output_Body_P);
  }

  if (PID_Body_R.Compute()) { // If computation successful and AUTO on
    ST.motor(motor_Body_R, Output_Body_R);
  }
  
  if (PID_Body_Y.Compute()) { // If computation successful and AUTO on    
      digitalWrite(motorDir_Body_Y, abs(Output_Body_Y)/Output_Body_Y);
      analogWrite(motorSpeed_Body_Y, abs(Output_Body_Y)); 

      if (Output_Body_Y == 0) { // Check if spin operation complete
        PID_Body_Y.SetMode(MANUAL); // turn off PID
      }
  }

  // Begin PID computations for head
  if (PID_Head_P.Compute()) { // If computation successful and AUTO on
    digitalWrite(motorDir_Head_P, abs(Output_Head_P)/Output_Head_P);
    analogWrite(motorSpeed_Head_P, abs(Output_Head_P)); 
  }
  
  if (PID_Head_R.Compute()) { // If computation successful and AUTO on
    digitalWrite(motorDir_Head_R, abs(Output_Head_R)/Output_Head_R);
    analogWrite(motorSpeed_Head_R, abs(Output_Head_R)); 
  }
  
  if (PID_Head_Y.Compute()) { // If computation successful
    digitalWrite(motorDir_Head_Y, abs(Output_Head_Y)/Output_Head_Y);
    analogWrite(motorSpeed_Head_Y, abs(Output_Head_Y)); 

    if (Output_Head_Y == 0) { // Check if approximately stabilised
      PID_Head_Y.SetMode(MANUAL); // turn off PID
    }
  }
  
  /* Display the floating point data */
  /*
  Serial.print("BODY IMU DATA:\t\t\t\tHEAD IMU DATA:\n");
  Serial.print("X: ");
  Serial.print(Body_event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(Body_event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(Body_event.orientation.z, 4);
  Serial.print("\t\tX: ");
  Serial.print(Head_event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(Head_event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(Head_event.orientation.z, 4);
  Serial.println("");
  */
}

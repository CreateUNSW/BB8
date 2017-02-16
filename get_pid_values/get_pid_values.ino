#include <PID_v1.h>

#define PIN_INPUT 0
#define RELAY_PIN 6

//Define Variables we'll be connecting to
double SetPoint, Input, Output;

//Specify the links and initial tuning parameters
PID* myPID;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void func (int Kp, int Ki,int  Kd) {
  int Kreal = 0.85;
  myPID = new PID(&Input, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);
  myPID->SetMode(AUTOMATIC);
  Serial.print(",Kp:");
  Serial.print(Kp);
  Serial.print(",Ki:");
  Serial.print(Ki);
  Serial.print("Kd:");
  Serial.println(Kd);

  Input = 20;
  SetPoint = 100;

  for(int i=0;i< 100;i++) {
    myPID->Compute();
    Serial.print(",Input:");
    Serial.print(Input);
    Serial.print(",Output:");
    Serial.print(Output);
    Serial.print(",SetPoint:");
    Serial.println(SetPoint);

    Input += 1;
  }
  
  
  delete myPID;
  
}

void loop() {
  int Kp,Ki,Kd;
  for(Kp = 1;Kp <= 10; Kp ++ ) {
    for(Ki = 1; Ki <= 10; Ki ++ ) {
      for (Kd = 1; Kd <= 10; Kd ++ ) {
        func(Kp,Ki,Kd);
      }
    }
  }
  while(1) {
    
  }
}

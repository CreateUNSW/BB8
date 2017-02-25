#include <Servo.h>


Servo s_roll, s_pitch;
bool inputComplete = false;
int input = 0;

#define SERVO_PITCH 10
#define SERVO_ROLL 9

int current = 35;

void s_roll_wrapper(int _input) {
  int diff = _input - current;
  if(diff==0) {
    return; 
  }
  int inc = diff < 0 ? -1 : 1 ;
  for(int i = current; i != _input; i += inc) {
     Serial.print(i);
     Serial.print(" ");
    s_roll.write(i);
    delay(50); 
  }
//  if(diff < 0) {
//    for(int i = current; i >= _input; i--) {
//      Serial.print(i);
//      Serial.print(" ");
//      s_roll.write(i);
//      delay(100);
//    }
//  } else {
//    for(int i = current; i <= _input; i++) {
//      s_roll.write(i);
//      delay(20);
//    }
//    
//  }
  current = _input;
}

void setup() {
  // put your setup code here, to run once:

  s_roll.attach(SERVO_ROLL);
  s_pitch.attach(SERVO_PITCH);
  s_roll.write(current);
  Serial.begin(9600);
}

void loop() {
  if(inputComplete) {
    inputComplete = false;
    s_roll_wrapper(input);
    Serial.print(" > ");
    Serial.println(input);
    input = 0;
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

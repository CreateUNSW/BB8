#include <Servo.h>


Servo s_roll, s_pitch;
bool inputComplete = false;
int input = 0;

#define SERVO_PITCH 9
#define SERVO_ROLL 10

int current = 90;

void s_roll_wrapper(int _input) {
  int diff = _input - current;
  if(diff==0) {
    return; 
  }
  int inc = diff < 0 ? -1 : 1 ;
  int i;
  for( i = current; i != _input; i += inc) {
    s_roll.write(i);
    delay(50); 
  }
  s_roll.write(i);
  delay(50); 
  current = _input;
}

void setup() {
  // put your setup code here, to run once:

  s_roll.attach(SERVO_ROLL);
  s_pitch.attach(SERVO_PITCH);
  s_roll.write(current);
  s_pitch.write(current);
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

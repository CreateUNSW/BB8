#include <Servo.h>

#define SERVO_PITCH 9
#define SERVO_ROLL 10

#define CH1 18
#define CH2 19
#define CH3 2
#define CH4 3
#define CH5 20
#define CH6 21

Servo s_roll, s_pitch;
bool inputComplete = false;
int input = 0;

int ch1_value = 0, ch2_value = 0, ch3_value = 0, ch4_value = 0, ch5_value = 0, ch6_value = 0;
bool ch1_ready = false, ch2_ready = false, ch3_ready = false, ch4_ready = false, ch5_ready = false, ch6_ready = false;
int ch1_value_start, ch2_value_start, ch3_value_start, ch4_value_start, ch5_value_start, ch6_value_start;


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

int current_roll = 90, current_pitch = 90;

void s_roll_wrapper(int _input) {
  int diff = _input - current_roll;
  if(diff==0) {
    return; 
  }
  int inc = diff < 0 ? -1 : 1 ;
  int i;
  for( i = current_roll; i != _input; i += inc) {
    s_roll.write(i);
    delay(50); 
  }
  s_roll.write(i);
  delay(50); 
  current_roll = _input;
}


void s_pitch_wrapper(int _input) {
  int diff = _input - current_pitch;
  if(diff==0) {
    return; 
  }
  int inc = diff < 0 ? -1 : 1 ;
  int i;
  for( i = current_pitch; i != _input; i += inc) {
    s_pitch.write(i);
    delay(50); 
  }
  s_pitch.write(i);
  delay(50); 
  current_pitch = _input;
}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

  s_roll.attach(SERVO_ROLL);
  s_pitch.attach(SERVO_PITCH);
  s_roll.write(current_roll);
  s_pitch.write(current_pitch);

  
  pinMode(CH1,INPUT);
  pinMode(CH2,INPUT);
  pinMode(CH3,INPUT);
  pinMode(CH4,INPUT);
  pinMode(CH5,INPUT);
  pinMode(CH6,INPUT);

 attachInterrupt(digitalPinToInterrupt(CH1),ch1_handler,CHANGE);
 attachInterrupt(digitalPinToInterrupt(CH2),ch2_handler,CHANGE);
// attachInterrupt(digitalPinToInterrupt(CH3),ch3_handler,CHANGE);
// attachInterrupt(digitalPinToInterrupt(CH4),ch4_handler,CHANGE);
// attachInterrupt(digitalPinToInterrupt(CH5),ch5_handler,CHANGE);
// attachInterrupt(digitalPinToInterrupt(CH6),ch6_handler,CHANGE);
}

void loop() {
  Serial.print(map(ch1_value,1000,2000,80,100));
  Serial.print(" ");
  Serial.print(map(ch2_value,1000,2000,80,100));
  Serial.println(" ");
  if(ch1_ready) {
    ch1_ready = false;
    s_roll.write(map(ch1_value,1000,2000,80,100));
  }
  if(ch2_ready) {
    ch2_ready = false;
    s_pitch.write(map(ch2_value,1000,2000,80,100));
  }
  delay(25);

}

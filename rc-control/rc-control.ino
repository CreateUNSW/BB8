#define CH1 20
#define CH2 21
#define CH3 18
#define CH4 19
#define CH5 2
#define CH6 3

// CH1 right up-down

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

void setup() {
  pinMode(CH1,INPUT);
  pinMode(CH2,INPUT);
  pinMode(CH3,INPUT);
  pinMode(CH4,INPUT);
  pinMode(CH5,INPUT);
  pinMode(CH6,INPUT);

 attachInterrupt(digitalPinToInterrupt(CH1),ch1_handler,CHANGE);
 attachInterrupt(digitalPinToInterrupt(CH2),ch2_handler,CHANGE);
 attachInterrupt(digitalPinToInterrupt(CH3),ch3_handler,CHANGE);
 attachInterrupt(digitalPinToInterrupt(CH4),ch4_handler,CHANGE);
 attachInterrupt(digitalPinToInterrupt(CH5),ch5_handler,CHANGE);
 attachInterrupt(digitalPinToInterrupt(CH6),ch6_handler,CHANGE);

  Serial.begin(9600);
}

void loop() {
  Serial.print(ch1_value);
  Serial.print(" ");
  Serial.print(ch2_value);
  Serial.print(" ");
  Serial.print(ch3_value);
  Serial.print(" ");
  Serial.print(ch4_value);
  Serial.print(" ");
  Serial.print(ch5_value);
  Serial.print(" ");
  Serial.print(ch6_value);
  Serial.println(" ");
}

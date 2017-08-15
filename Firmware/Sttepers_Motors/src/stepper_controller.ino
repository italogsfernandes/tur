#define STEPS_PER_REVOLUTION 500
#define STEPPER_RIGHT_IN1 8
#define STEPPER_RIGHT_IN2 7
#define STEPPER_RIGHT_IN3 6
#define STEPPER_RIGHT_IN4 5


#define MICROSECONDSMODE

#define STATE_SINGLE_STEP 0
#define STATE_DOUBLE_STEP 1
#define STATE_HALF_STEP 2

uint8_t actual_state = STATE_SINGLE_STEP;
#ifdef MICROSECONDSMODE
uint16_t delay_between_steps = 10000;
uint16_t delay_between_half_steps = 5000;
#endif
#ifndef MICROSECONDSMODE
uint16_t delay_between_steps = 10;
uint16_t delay_between_half_steps = 5;
#endif
char serial_command = 's';

void setup() {
  //Determina a velocidade inicial do motor
  pinMode(STEPPER_RIGHT_IN1, OUTPUT);
  pinMode(STEPPER_RIGHT_IN2, OUTPUT);
  pinMode(STEPPER_RIGHT_IN3, OUTPUT);
  pinMode(STEPPER_RIGHT_IN4, OUTPUT);
  Serial.begin(115200);
  Serial.println("Stepper Motor Controller Started!");
}
void loop()
{
  if (Serial.available()) {
    serial_command  = Serial.read();
    switch (serial_command) {
      case 's':
        actual_state = STATE_SINGLE_STEP;
        Serial.println("Single Step Selecionado");
        break;
      case 'd':
        actual_state = STATE_DOUBLE_STEP;
        Serial.println("Double Step Selecionado");
        break;
      case 'h':
        actual_state = STATE_HALF_STEP;
        Serial.println("Half Step Selecionado");
        break;
#ifdef MICROSECONDSMODE
      case '+':
        delay_between_steps += 100;
        delay_between_half_steps = delay_between_steps / 2;
        Serial.println("Delay de " + String(delay_between_steps) + "us.");
        break;
      case '-':
        delay_between_steps -= 100;
        delay_between_half_steps = delay_between_steps / 2;
        Serial.println("Delay de " + String(delay_between_steps) + "us.");
        break;
#endif
#ifndef MICROSECONDSMODE
      case '+':
        delay_between_steps += 10;
        delay_between_half_steps = delay_between_steps / 2;
        Serial.println("Delay de " + String(delay_between_steps) + "ms.");
        break;
      case '-':
        delay_between_steps -= 10;
        delay_between_half_steps = delay_between_steps / 2;
        Serial.println("Delay de " + String(delay_between_steps) + "ms.");
#endif
    }
  }
  switch (actual_state) {
    case STATE_SINGLE_STEP:
      single_step();;
      break;
    case STATE_DOUBLE_STEP:
      double_step();
      break;
    case STATE_HALF_STEP:
      half_step();
      break;
  }
}

void single_step() {
  digitalWrite(STEPPER_RIGHT_IN1, 1);
  digitalWrite(STEPPER_RIGHT_IN2, 0);
  digitalWrite(STEPPER_RIGHT_IN3, 0);
  digitalWrite(STEPPER_RIGHT_IN4, 0);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_steps);
#endif
  digitalWrite(STEPPER_RIGHT_IN1, 0);
  digitalWrite(STEPPER_RIGHT_IN2, 1);
  digitalWrite(STEPPER_RIGHT_IN3, 0);
  digitalWrite(STEPPER_RIGHT_IN4, 0);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_steps);
#endif
  digitalWrite(STEPPER_RIGHT_IN1, 0);
  digitalWrite(STEPPER_RIGHT_IN2, 0);
  digitalWrite(STEPPER_RIGHT_IN3, 1);
  digitalWrite(STEPPER_RIGHT_IN4, 0);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_steps);
#endif
  digitalWrite(STEPPER_RIGHT_IN1, 0);
  digitalWrite(STEPPER_RIGHT_IN2, 0);
  digitalWrite(STEPPER_RIGHT_IN3, 0);
  digitalWrite(STEPPER_RIGHT_IN4, 1);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_steps);
#endif
}

void double_step() {
  digitalWrite(STEPPER_RIGHT_IN1, 1);
  digitalWrite(STEPPER_RIGHT_IN2, 1);
  digitalWrite(STEPPER_RIGHT_IN3, 0);
  digitalWrite(STEPPER_RIGHT_IN4, 0);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_steps);
#endif
  digitalWrite(STEPPER_RIGHT_IN1, 0);
  digitalWrite(STEPPER_RIGHT_IN2, 1);
  digitalWrite(STEPPER_RIGHT_IN3, 1);
  digitalWrite(STEPPER_RIGHT_IN4, 0);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_steps);
#endif
  digitalWrite(STEPPER_RIGHT_IN1, 0);
  digitalWrite(STEPPER_RIGHT_IN2, 0);
  digitalWrite(STEPPER_RIGHT_IN3, 1);
  digitalWrite(STEPPER_RIGHT_IN4, 1);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_steps);
#endif
  digitalWrite(STEPPER_RIGHT_IN1, 1);
  digitalWrite(STEPPER_RIGHT_IN2, 0);
  digitalWrite(STEPPER_RIGHT_IN3, 0);
  digitalWrite(STEPPER_RIGHT_IN4, 1);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_steps);
#endif
}

void half_step() {
  digitalWrite(STEPPER_RIGHT_IN1, 1);
  digitalWrite(STEPPER_RIGHT_IN2, 1);
  digitalWrite(STEPPER_RIGHT_IN3, 0);
  digitalWrite(STEPPER_RIGHT_IN4, 0);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_half_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_half_steps);
#endif
  digitalWrite(STEPPER_RIGHT_IN1, 0);
  digitalWrite(STEPPER_RIGHT_IN2, 1);
  digitalWrite(STEPPER_RIGHT_IN3, 0);
  digitalWrite(STEPPER_RIGHT_IN4, 0);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_half_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_half_steps);
#endif

  digitalWrite(STEPPER_RIGHT_IN1, 0);
  digitalWrite(STEPPER_RIGHT_IN2, 1);
  digitalWrite(STEPPER_RIGHT_IN3, 1);
  digitalWrite(STEPPER_RIGHT_IN4, 0);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_half_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_half_steps);
#endif
  digitalWrite(STEPPER_RIGHT_IN1, 0);
  digitalWrite(STEPPER_RIGHT_IN2, 0);
  digitalWrite(STEPPER_RIGHT_IN3, 1);
  digitalWrite(STEPPER_RIGHT_IN4, 0);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_half_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_half_steps);
#endif

  digitalWrite(STEPPER_RIGHT_IN1, 0);
  digitalWrite(STEPPER_RIGHT_IN2, 0);
  digitalWrite(STEPPER_RIGHT_IN3, 1);
  digitalWrite(STEPPER_RIGHT_IN4, 1);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_half_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_half_steps);
#endif
  digitalWrite(STEPPER_RIGHT_IN1, 0);
  digitalWrite(STEPPER_RIGHT_IN2, 0);
  digitalWrite(STEPPER_RIGHT_IN3, 0);
  digitalWrite(STEPPER_RIGHT_IN4, 1);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_half_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_half_steps);
#endif

  digitalWrite(STEPPER_RIGHT_IN1, 1);
  digitalWrite(STEPPER_RIGHT_IN2, 0);
  digitalWrite(STEPPER_RIGHT_IN3, 0);
  digitalWrite(STEPPER_RIGHT_IN4, 1);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_half_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_half_steps);
#endif
  digitalWrite(STEPPER_RIGHT_IN1, 1);
  digitalWrite(STEPPER_RIGHT_IN2, 0);
  digitalWrite(STEPPER_RIGHT_IN3, 0);
  digitalWrite(STEPPER_RIGHT_IN4, 0);
#ifdef MICROSECONDSMODE
  delayMicroseconds(delay_between_half_steps);
#endif
#ifndef MICROSECONDSMODE
  delay(delay_between_half_steps);
#endif

}




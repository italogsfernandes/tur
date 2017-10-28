




#define JUST_SURVIVE

//#define ROLL_ROLL_ROLL_ATTACK
//#define RANDON_SEARCH




#define buzzer 3
#define btn_start 12

#define in4 4
#define enB 5
#define in3 6
#define in2 7
#define in1 8
#define enA 9

#define linha_direita A1
#define linha_esquerda A0

#define trig 10
#define echo 11

/////////////////////////////////////////////////
// Implementacao de um timer atraves do millis //
/////////////////////////////////////////////////

class Timer {
  private:
    unsigned long _actual_time;
    unsigned long _waited_time;
    bool _running;
    uint32_t _interval;

  public:
    bool elapsed;

    Timer(uint32_t interval = 1000) {
      _interval = interval;
      elapsed = false;
      _actual_time = millis();
      _waited_time = _actual_time + interval;
    }

    void setInterval(uint32_t interval) {
      _interval = interval;
    }

    uint32_t getInterval() {
      return _interval;
    }

    void start() {
      _running = true;
      _actual_time = millis();
      _waited_time = _actual_time + _interval;
    }

    void stop() {
      _running = false;
    }

    void wait_next() {
      elapsed = false;
    }

    void update() {
      _actual_time = millis();
      if (_running) {
        if (_actual_time >= _waited_time) {
          _waited_time = _actual_time + _interval;
          elapsed = true;
        }
      }
    }
};


typedef enum {
  FRENTE,
  TRAS,
  DIREITA,
  ESQUERDA,
  NOTHING
} direction_t;

class Motores {
  private:
    uint8_t _pin_IN[4];
    uint8_t _pin_EN[2];
    uint8_t _velocidade; // valor do sinal pwm 0-255
    direction_t _direction;
    Timer motorTimer;


    void updateDirection() {
      switch (_direction) {
        case FRENTE:
          digitalWrite(_pin_IN[0], 1); digitalWrite(_pin_IN[1], 0);
          digitalWrite(_pin_IN[2], 1); digitalWrite(_pin_IN[3], 0);
          break;
        case TRAS:
          digitalWrite(_pin_IN[0], 0); digitalWrite(_pin_IN[1], 1);
          digitalWrite(_pin_IN[2], 0); digitalWrite(_pin_IN[3], 1);
          break;
        case DIREITA:
          digitalWrite(_pin_IN[0], 0); digitalWrite(_pin_IN[1], 1);
          digitalWrite(_pin_IN[2], 1); digitalWrite(_pin_IN[3], 0);
          break;
        case ESQUERDA:
          digitalWrite(_pin_IN[0], 1); digitalWrite(_pin_IN[1], 0);
          digitalWrite(_pin_IN[2], 0); digitalWrite(_pin_IN[3], 1);
          break;
        case NOTHING:
          digitalWrite(_pin_IN[0], 1); digitalWrite(_pin_IN[1], 1);
          digitalWrite(_pin_IN[2], 1); digitalWrite(_pin_IN[3], 1);
          break;
        default:
          //TODO: Handle exceptions
          break;
      }
    }

    void updateSpeed() {
      analogWrite(_pin_EN[0], _velocidade);
      analogWrite(_pin_EN[1], _velocidade);
    }

    void stopMotors() {
      digitalWrite(_pin_EN[0], 1); digitalWrite(_pin_EN[1], 1);
      digitalWrite(_pin_IN[0], 1); digitalWrite(_pin_IN[1], 1);
      digitalWrite(_pin_IN[2], 1); digitalWrite(_pin_IN[3], 1);
    }

    void begin() {
      pinMode(_pin_EN[0], OUTPUT); pinMode(_pin_EN[1], OUTPUT);

      pinMode(_pin_IN[0], OUTPUT); pinMode(_pin_IN[1], OUTPUT);
      pinMode(_pin_IN[2], OUTPUT); pinMode(_pin_IN[3], OUTPUT);
    }

  public:
    Motores(uint8_t pinENA, uint8_t pinIN1, uint8_t pinIN2, uint8_t pinIN3,
            uint8_t pinIN4, uint8_t pinENB) {
      _pin_EN[0] = pinENA;
      _pin_IN[0] = pinIN1;
      _pin_IN[1] = pinIN2;
      _pin_IN[2] = pinIN3;
      _pin_IN[3] = pinIN4;
      _pin_EN[1] = pinENB;
      begin();
      setSpeedPercent(100);
      setDirection(FRENTE);
    }

    void updateMotor()
    {
      motorTimer.update();
      if (motorTimer.elapsed)
      {
        motorTimer.stop();
        this->stop();
      }
    }

    void runFor(uint32_t timeToRotate)
    {
      motorTimer.setInterval(timeToRotate);
      motorTimer.start();
    }



    direction_t getDirection() {
      return _direction;
    }

    void setDirection(direction_t direcao) {
      _direction = direcao;
      updateDirection();
    }

    uint8_t getSpeed() {
      return _velocidade;
    }

    void setSpeed(uint8_t newSpeed) {
      _velocidade = newSpeed;
      updateSpeed();
    }

    double getSpeedPercent() {
      return _velocidade * 100.0 / 255.0;
    }

    void setSpeedPercent(double newSpeed) {
      _velocidade = (uint8_t)(((double)newSpeed / 100.0) * 255.0);
      updateSpeed();
    }

    void run() {
      updateDirection();
      updateSpeed();
    }

    void stop() {
      stopMotors();
    }

};


class LineSensor {
  private:
    uint8_t _pin;
    uint16_t _limiar;

  public:
    bool _is_in_black;

    LineSensor(uint8_t pin) {
      _pin = pin;
      calibrate();
    }

    void update() {
      _is_in_black = analogRead(_pin) > _limiar;
    }

    void calibrate() {
      setLimiar(512);
    }

    void setLimiar(uint16_t limiar) {
      _limiar = limiar;
    }
};

class UltrassonicSensor
{
  private:
    uint8_t _pinTrigger;
    uint8_t _pinEcho;
    uint32_t _duration = 0;
    uint32_t _distance;
    uint32_t _timeOut = 3000;

  public:
    UltrassonicSensor(uint8_t pinTrigger, uint8_t pinEcho)
    {
      _pinTrigger = pinTrigger;
      _pinEcho = pinEcho;
      pinMode(_pinTrigger,OUTPUT);
      pinMode(_pinEcho,INPUT);
      _timeOut = 3000; //see ultrasonic library for more info
    }

    uint32_t takeTime()
    {
      digitalWrite(_pinTrigger, LOW);
      delayMicroseconds(2);
      digitalWrite(_pinTrigger, HIGH);
      delayMicroseconds(10);
      digitalWrite(_pinTrigger, LOW);

      _duration = pulseIn(_pinEcho,HIGH,_timeOut);
      if ( _duration == 0 )
        _duration = _timeOut;
      return _duration;
    }

    uint32_t takeDistance()
    {
      takeTime();
      _distance = _duration /29 / 2 ;
      return _distance;
    }
};


/**
  FUUCK THE KING!!

*/
void waitForButton() {
  Serial.println("Waiting");
  digitalWrite(LED_BUILTIN, HIGH);
  //Waiting for button
  while (digitalRead(btn_start));
  Serial.println("Going in 5 sec");
  digitalWrite(LED_BUILTIN, LOW);
  delay(5000);
  digitalWrite(buzzer, 1);
  delay(100);
  digitalWrite(buzzer, 0);
  Serial.println("Go");
}




// The pins were verified!
Motores carro(9, 8, 7, 4, 6, 5); // ENA,IN1,IN2,IN3,IN4,ENB in that order
UltrassonicSensor ultrasonic(10,11); // Trig ,Echo
LineSensor rightLine(A1);
LineSensor leftLine(A0);


float distance;

void setup() {
  Serial.begin(115200);
  carro.stop();
  //Setando IO
  pinMode(buzzer, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(btn_start, INPUT_PULLUP);

  waitForButton();
  carro.run();
  carro.setSpeedPercent(35);
  carro.setDirection(FRENTE);
}


#ifdef JUST_SURVIVE
/*
  void loop(){
  digitalWrite(LED_BUILTIN,0);
  carro.setSpeedPercent(0);
  delay(1000); digitalWrite(LED_BUILTIN,1);
  carro.setSpeedPercent(10);
  delay(1000); digitalWrite(LED_BUILTIN,0);
  carro.setSpeedPercent(20);
  delay(1000); digitalWrite(LED_BUILTIN,1);
  carro.setSpeedPercent(30);
  delay(1000); digitalWrite(LED_BUILTIN,0);
  carro.setSpeedPercent(40);
  delay(1000);  digitalWrite(LED_BUILTIN,1);
  carro.setSpeedPercent(50);
  delay(1000); digitalWrite(LED_BUILTIN,0);
  carro.setSpeedPercent(60);
  delay(1000);  digitalWrite(LED_BUILTIN,1);
  carro.setSpeedPercent(70);
  delay(1000); digitalWrite(LED_BUILTIN,0);
  carro.setSpeedPercent(80);
  delay(1000);  digitalWrite(LED_BUILTIN,1);
  carro.setSpeedPercent(90);
  delay(1000); digitalWrite(LED_BUILTIN,0);
  carro.setSpeedPercent(100);
  delay(1000);   digitalWrite(LED_BUILTIN,1);
  }
*/
void loop() {
  distance = ultrasonic.takeDistance();
  rightLine.update();
  leftLine.update();
  Serial.print(analogRead(A0)); Serial.print("\t");
  Serial.print(analogRead(A1)); Serial.print("\n");

  if (!rightLine._is_in_black && !leftLine._is_in_black) {
    digitalWrite(LED_BUILTIN, 1);
    carro.setSpeedPercent(100);
    carro.setDirection(TRAS);
    delay(400);
  } else if (!rightLine._is_in_black) {
    digitalWrite(LED_BUILTIN, 1);
    carro.setSpeedPercent(100);
    carro.setDirection(TRAS);
    delay(400);
    carro.setSpeedPercent(70);
    carro.setDirection(ESQUERDA);
    delay(330);
  } else if (!leftLine._is_in_black) {
    digitalWrite(LED_BUILTIN, 1);
    carro.setSpeedPercent(100);
    carro.setDirection(TRAS);
    delay(400);
    carro.setSpeedPercent(70);
    carro.setDirection(DIREITA);
    delay(330);
  } else {
    digitalWrite(LED_BUILTIN, 0);/*
    if (distance < 30) {
      //Work around....
      //Or maybe not...
      distance < 10 ? carro.setSpeedPercent(80) : carro.setSpeedPercent(60);
      carro.setDirection(FRENTE);
    } else {
      carro.setSpeedPercent(55);
      carro.setDirection(DIREITA);
    }*/
    carro.setSpeedPercent(50);
    carro.setDirection(FRENTE);
  }
}/*
  void loop(){
  rightLine.update();
  leftLine.update();

  if(rightLine._is_in_black && leftLine._is_in_black){
    digitalWrite(LED_BUILTIN,1);
    carro.setDirection(TRAS);
  } else if(rightLine._is_in_black){
    digitalWrite(LED_BUILTIN,1);
    carro.setDirection(ESQUERDA);
    delay(50);
  } else if(leftLine._is_in_black){
    digitalWrite(LED_BUILTIN,1);
    carro.setDirection(DIREITA);
    delay(50);
  } else {
    digitalWrite(LED_BUILTIN,0);
    carro.setDirection(FRENTE);
  }
  }
*/
#endif /*JUST_SURVIVE*/



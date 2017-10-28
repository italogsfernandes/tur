#include <NewPing.h>
/////////////////////////////////////////////////
// Implementacao de um timer atraves do millis //
/////////////////////////////////////////////////

bool flagBegin = true;

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

    void updateLeftSpeed() {
      analogWrite(_pin_EN[1], _velocidade);
    }

    void updateRightSpeed() {
      analogWrite(_pin_EN[0], _velocidade);
    }

    void stopMotors() {
      digitalWrite(_pin_EN[0], 0); digitalWrite(_pin_EN[1], 0);
      digitalWrite(_pin_IN[0], 0); digitalWrite(_pin_IN[1], 0);
      digitalWrite(_pin_IN[2], 0); digitalWrite(_pin_IN[3], 0);
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

    void setLeftSpeedPercent(double newSpeed) {
      _velocidade = (uint8_t)(((double)newSpeed / 100.0) * 255.0);
      updateLeftSpeed();
    }

    void setRigthSpeedPercent(double newSpeed) {
      _velocidade = (uint8_t)(((double)newSpeed / 100.0) * 255.0);
      updateRightSpeed();
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
      setLimiar(900);
    }

    void setLimiar(uint16_t limiar) {
      _limiar = limiar;
    }
};



//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

// The pins were verified!


Motores carro(5, 2, 4, 7, 8, 6); // ENA,IN1,IN2,IN3,IN4,ENB in that order
LineSensor rightLine(A0);
LineSensor leftLine(A1);

#define MAX_DISTANCE 35 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(12, 11, MAX_DISTANCE); //Trigger, ECHO, MAX_DISTANCE // NewPing setup of pins and maximum distance.
int distance;
unsigned long waitedtime_to_measure;

#define PIN_BTN_START 10
#define PIN_BUZZER  9

void setup() {
  Serial.begin(115200);
  carro.stop();
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Iniciado e aguardando...");
  waitForButton();
  Serial.println("Go!");
  carro.run();
  carro.setSpeedPercent(40);
  delay(550);
}

void waitForButton() {
  Serial.println("Waiting...");
  while (digitalRead(PIN_BTN_START));
  for (int i = 4; i > 0; i--) {
    Serial.println("Going in " + String(i + 1) + " sec");
    digitalWrite(LED_BUILTIN, LOW);//digitalWrite(PIN_BUZZER, LOW);
    delay(900);
    digitalWrite(LED_BUILTIN, HIGH);// digitalWrite(PIN_BUZZER, HIGH);
    delay(100);
  }
  Serial.println("Going in " + String(1) + " sec");
  digitalWrite(LED_BUILTIN, LOW);//digitalWrite(PIN_BUZZER, LOW);
  delay(900);
  digitalWrite(LED_BUILTIN, HIGH); digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW); digitalWrite(PIN_BUZZER, LOW);
  Serial.println("Go");
}


///////////////
//ESTRATEGIAS//
//////////////
//#define SENSOR_TESTER
//#define FRENTE_E_AVANTE
#define LINE_AVOIDER //Evitador de linha - selecione abaixo as opções:
//#define JUST_AVOID
#define LITTLE_TURN_AND_SEARCH


//To Implement:
//#define JUST_SURVIVE
//#define TE_PEGO_NO_MEIO
//#define ROLL_ROLL_ROLL_ATTACK
//#define RANDON_SEARCH


//////////////////
//Implementações//
//////////////////
////////////////////
//FRENTE_E_AVANTE //
////////////////////
#ifdef FRENTE_E_AVANTE
void loop() {
  carro.setSpeedPercent(100);
  carro.setDirection(FRENTE);
}
#endif /*FRENTE_E_AVANTE*/

//////////////////
//LINE_AVOIDER  //
//////////////////
#ifdef LINE_AVOIDER

#define VEL_RE      50    //Porcentagem da velocidade
#define VEL_VIRAR   50
#define TEMPO_RE    400   //Tempo em milisegundos para manter a velocidade
#define TEMPO_VIRAR 220

//Just Avoiding:
#define VEL_FRENTE  50 //is used in the little also


//////////////////
//LITTLE TURN   //
//////////////////
//Little Turn
#define LITTLE_TURN_VEL         50
#define LITTLE_TURN_TIME        50 // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
#define LITTLE_TURN_DIRECTION   ESQUERDA
#define LITTLE_TURN_READING_TIME  10

#define LITTLE_TURN_VEL_FRENTE_MAX  100
#define LITTLE_TURN_VEL_FRENTE_MED 70

#define LITTLE_TURN_DIST        30 //20 cm
#define LITTLE_TURN_DIST_CLOSER 7 //20 cm

void loop() {
  rightLine.update();
  leftLine.update();
  //Debug Messages
  Serial.print("Direita: ");
  rightLine._is_in_black ? Serial.print("PRETO\t") : Serial.print("BRANCO\t");
  Serial.print("Esquerda: ");
  leftLine._is_in_black ? Serial.print("PRETO\t") : Serial.print("BRANCO\t");

  //Avoiding
  if (!rightLine._is_in_black && !leftLine._is_in_black) {
    Serial.println("\n**************************************************");
    Serial.println("RE - Vel: " + String(VEL_RE) + "% - Tempo: " + String(TEMPO_RE) + " ms.");
    Serial.println("**************************************************");
    carro.setSpeedPercent(VEL_RE); carro.setDirection(TRAS); delay(TEMPO_RE);
  } else if (!rightLine._is_in_black) {
    Serial.println("\n**************************************************");
    Serial.println("RE - Vel: " + String(VEL_RE) + "% - Tempo: " + String(TEMPO_RE) + " ms.");
    Serial.println("Virar Esquerda - Vel: " + String(VEL_VIRAR) + "% - Tempo: " + String(TEMPO_VIRAR) + " ms.");
    Serial.println("**************************************************");
    carro.setSpeedPercent(VEL_RE); carro.setDirection(TRAS); delay(TEMPO_RE);
    carro.setSpeedPercent(VEL_VIRAR); carro.setDirection(ESQUERDA); delay(TEMPO_VIRAR);
  } else if (!leftLine._is_in_black) {
    Serial.println("\n**************************************************");
    Serial.println("RE - Vel: " + String(VEL_RE) + "% - Tempo: " + String(TEMPO_RE) + " ms.");
    Serial.println("Virar Direita - Vel: " + String(VEL_VIRAR) + "% - Tempo: " + String(TEMPO_VIRAR) + " ms.");
    Serial.println("**************************************************");
    carro.setSpeedPercent(VEL_RE); carro.setDirection(TRAS); delay(TEMPO_RE);
    carro.setSpeedPercent(VEL_VIRAR); carro.setDirection(DIREITA); delay(TEMPO_VIRAR);
  } else {
#ifdef JUST_AVOID
    Serial.println("Frente - Vel: " + String(VEL_FRENTE) + "%");
    carro.setSpeedPercent(VEL_FRENTE); carro.setDirection(FRENTE);
#endif /*JUST_AVOID*/
#ifdef LITTLE_TURN_AND_SEARCH
    doSearch();
#endif /*LITLE_TURN_AND_SEARCH*/
  }
}

void doSearch() {
  if (distance < LITTLE_TURN_DIST && distance != 0) { //Menor que LITTLE_TURN_DIST cm e nao eh out of range
    if (distance < LITTLE_TURN_DIST_CLOSER) {
      Serial.println("Frente - Vel: " + String(LITTLE_TURN_VEL_FRENTE_MAX) + "%");
      carro.setSpeedPercent(LITTLE_TURN_VEL_FRENTE_MAX); carro.setDirection(FRENTE);
    } else {
      Serial.println("Frente - Vel: " + String(LITTLE_TURN_VEL_FRENTE_MED) + "%");
      carro.setSpeedPercent(LITTLE_TURN_VEL_FRENTE_MED); carro.setDirection(FRENTE);
    }
    if (millis() >= waitedtime_to_measure) {
      waitedtime_to_measure = millis() + LITTLE_TURN_READING_TIME;
      distance = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
      Serial.println("\t\t\t\t\t\t\t\t\t\t\t\tPing: " + String(distance) + " cm.");
    }
  } else {
    Serial.println("\nLittle Turn - Vel: " + String(LITTLE_TURN_VEL) + "% - Tempo: " + String(LITTLE_TURN_TIME) + " ms.");
    Serial.println("Little Turn Pausa: " + String(LITTLE_TURN_READING_TIME) + " ms.");
    carro.setSpeedPercent(LITTLE_TURN_VEL); carro.setDirection(LITTLE_TURN_DIRECTION); delay(LITTLE_TURN_TIME);
    //qcarro.stop();
    distance = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.println("\t\t\t\t\t\t\t\t\t\t\t\tPing: " + String(distance) + " cm.");
    delay(LITTLE_TURN_READING_TIME);
    carro.run();
  }
}
#endif /*LINE_AVOIDER*/

#ifdef SENSOR_TESTER
void loop() {
  distance = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print(distance * 30); Serial.print("\t");
  Serial.print(analogRead(A0)); Serial.print("\t");
  Serial.print(analogRead(A1)); Serial.print("\t");
  Serial.println();
  delay(50);
}
#endif /*SENSOR_TESTER*/



//SO PRA FRENTE
//  void loop()
//  {
//
//  }

//JA FUNCIONOU
//void loop() {
//  //distance = ultrasonic.takeDistance();
//  rightLine.update();
//  leftLine.update();
//
//
//  if (!rightLine._is_in_black && !leftLine._is_in_black) {
//    digitalWrite(LED_BUILTIN, 1);
//    carro.setSpeedPercent(50);
//    carro.setDirection(TRAS);
//    delay(400);
//  } else if (!rightLine._is_in_black) {
//    digitalWrite(LED_BUILTIN, 1);
//    carro.setSpeedPercent(50);
//    carro.setDirection(TRAS);
//    delay(400);
//    carro.setSpeedPercent(50);
//    carro.setDirection(ESQUERDA);
//    delay(220);
//  } else if (!leftLine._is_in_black) {
//    digitalWrite(LED_BUILTIN, 1);
//    carro.setSpeedPercent(50);
//    carro.setDirection(TRAS);
//    delay(400);
//    carro.setSpeedPercent(50);
//    carro.setDirection(DIREITA);
//    delay(220);
//  } else {
//    digitalWrite(LED_BUILTIN, 0);
//    carro.setSpeedPercent(30);
//    carro.setDirection(FRENTE);
//  }
//}

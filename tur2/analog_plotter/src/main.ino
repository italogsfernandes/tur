#define LINE_SENSOR_THRESOULD 900

typedef enum {
    ROT_CLOCKWISE,
    ROT_ANTI_CLOCKWISE,
    ROT_NONE
} rot_direction_t;

class Motor {
private:
    uint8_t _pin_IN[2];
    uint8_t _pin_EN;
    uint8_t _velocidade; // valor do sinal pwm 0-255
    rot_direction_t _rot_direction;

    void updateRotDirection() {
        switch (_rot_direction) {
            case ROT_CLOCKWISE:
            digitalWrite(_pin_IN[0], 1); digitalWrite(_pin_IN[1], 0);
            break;
            case ROT_ANTI_CLOCKWISE:
            digitalWrite(_pin_IN[0], 0); digitalWrite(_pin_IN[1], 1);
            break;
            case ROT_NONE:
            digitalWrite(_pin_IN[0], 1); digitalWrite(_pin_IN[1], 1);
            break;
            default:
            //TODO: Handle exceptions
            break;
        }
    }

    void updateSpeed() {
        analogWrite(_pin_EN, _velocidade);
    }

    void stopMotors() {
        digitalWrite(_pin_IN[0], 1); digitalWrite(_pin_IN[1], 1);
        analogWrite(_pin_EN, 0);
    }

    void begin() {
        pinMode(_pin_EN, OUTPUT);
        pinMode(_pin_IN[0], OUTPUT); pinMode(_pin_IN[1], OUTPUT);
    }

public:
    Motor(uint8_t pinEN, uint8_t pinIN1, uint8_t pinIN2) {
            _pin_EN = pinEN;
            _pin_IN[0] = pinIN1;
            _pin_IN[1] = pinIN2;
            begin();
            setSpeedPercent(100);
            setRotDirection(ROT_CLOCKWISE);
        }

        rot_direction_t getRotDirection() {
            return _rot_direction;
        }

        void setRotDirection(rot_direction_t rot_direcao) {
            _rot_direction = rot_direcao;
            updateRotDirection();
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
            updateRotDirection();
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
        bool is_in_black;
        bool is_in_white;

        uint16_t raw_value;

        LineSensor(uint8_t pin) {
            _pin = pin;
            calibrate();
        }

        void update() {
            raw_value = analogRead(_pin);
            is_in_black = raw_value > _limiar;
            is_in_white = !is_in_black;
        }

        void calibrate() {
            setLimiar(LINE_SENSOR_THRESOULD);
        }

        void setLimiar(uint16_t limiar) {
            _limiar = limiar;
        }
    };

LineSensor leftLine(A0);
LineSensor centerLine(A2);
LineSensor rightLine(A1);

//#define DEBUGGING_SENSORS
//#define POO_LINE_SENSORS
#define PID_SENSOR

//#define TESTE_MOTOR

Motor main_motor(3,4,5); //pinEN, pinIN1, pinIN2

float resposta_p, Kp;
float resposta_i, Ki;

float resposta_pi;

void setup(){
    Kp = 40;
    Ki = 25;
    Serial.begin(115200);
}

void loop(){
    #ifdef TESTE_MOTOR
    main_motor.run();
    main_motor.run();
    main_motor.setSpeed(255);
    main_motor.setRotDirection(ROT_CLOCKWISE);

    #endif

    #ifdef POO_LINE_SENSORS
    leftLine.update();
    centerLine.update();
    rightLine.update();

    Serial.print(100 * (int) leftLine.is_in_black);
    Serial.print("\t");
    Serial.print(100 * (int) centerLine.is_in_black);
    Serial.print("\t");
    Serial.print(100 * (int) rightLine.is_in_black);
    Serial.print("\t");
    Serial.println();
    #endif

    #ifdef PID_SENSOR
    leftLine.update();
    centerLine.update();
    rightLine.update();

    uint8_t leituras_array;
    float leitura_sensor;

    leituras_array = leftLine.is_in_black << 2 |  centerLine.is_in_black << 1 | rightLine.is_in_black << 0;

    if(leituras_array == 0b100){
        leitura_sensor = 2;
    } else if(leituras_array == 0b010){
        leitura_sensor = 0;
    } else if(leituras_array == 0b001){
        leitura_sensor = -2;
    } else if(leituras_array == 0b110){
        leitura_sensor = 1;
    } else if(leituras_array == 0b011){
        leitura_sensor = -1;
    } else if(leituras_array == 0b111){
        leitura_sensor = 0;
    } else if(leituras_array == 0b000){
        leitura_sensor = 0;
    }

    float valor_erro;
    valor_erro = leitura_sensor - 0;
    resposta_p = valor_erro*Kp;
    resposta_i = resposta_i + valor_erro*Ki*0.05;
    resposta_pi = resposta_p + resposta_i;
    main_motor.setRotDirection(resposta_pi > 0 ? ROT_CLOCKWISE : ROT_ANTI_CLOCKWISE);
    main_motor.setSpeed(abs(resposta_pi));

    Serial.print(valor_erro); Serial.print("\t");
    Serial.print(2*resposta_pi/255.0);
    Serial.println();
    #endif

    #ifdef DEBUGGING_SENSORS
    Serial.print(analogRead(A0));
    Serial.print("\t");
    Serial.print(analogRead(A1));
    Serial.print("\t");
    Serial.print(analogRead(A2));
    Serial.print("\t");
    Serial.println();
    #endif
    delay(50);
}

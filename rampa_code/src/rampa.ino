/* UNIVERSIDADE FEDERAL DE UBERLANDIA
BIOLAB - Biomedical Engineering Lab

Autor: Ítalo G S Fernandes
contact: italogsfernandes@gmail.com
URLs: www.biolab.eletrica.ufu.br
https://github.com/BIOLAB-UFU-BRAZIL
Este códido faz parte do projeto da competição do cobec.
O que faz:
Realiza leitura de 1 sensor inercial e envia para pc
Realiza leitura de um emg, filtra e envia para pc

TODO:
Adicionar Timer de aquisicao confiavel
Obter offsets do sensor inercial
Verificar "NOTE" espalhados no codigo.
Verificar "TODO" espalhados no codigo.

Pacotes:
Inercial(Quaternion): (11 bytes)
['$'] ['Q'] [WH] [WL] [XH] [XL] [YH] [YL] [ZH] [ZL] ['\n']
EMG(Valores do ADC passados após Media Móvel): (5 bytes)
['$'] ['E'] [EMGH] [EMGL] ['\n']

Esquema de montagem:
Arduino - Dispositivo
13      - LED
A0      - EMG-sinal (0 a 3.3V)
GND     - EMG-GND
A4      - SDA do GY-521
A5      - SCL do GY-521
5V      - VCC do GY-521
GND     - GND do GY-521

Para visualizar de forma visivel ao ser humano
Altere o comentario em: #define DEBUG_MODE
*/
#define DEBUG_MODE

//Se estiver no modo debug printa as msg debug, se nao estiver nao printa
#define DEBUG_PRINT__(x) Serial.print(x)
//#define DEBUG_PRINT__(x)


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LED_PIN 13

//NOTE: Antes de usar vc deve alterar a frequenciana biblioteca mpu6050
//CASO ISSO NAO SEJA FEITO CORRE PERIGO DA FIFO ESTOURAR
#define MPUsampFreq 100
#define mpu_interval 10 //Each 10ms
#define print_interval 100 //Each 100ms

#define QPMM 10 // Qnt de pontos media movel - Sera feita a media de 10 pontos
#define PSDMP 42 //Packet Size DMP - tam do pacote interno da mpu-6050

#define UART_BAUDRATE 115200

//TODO: trocar esses millis por timer
unsigned long currentMillis = 0;
unsigned long previousMPUMillis = 0;
unsigned long previousPrintMillis = 0;

//Variaveis Inercial
MPU6050 mpu(0x68);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int numbPackets;

void setup() {
    //GPIO:
    pinMode(LED_PIN, OUTPUT);

    //Serial:
    Serial.begin(UART_BAUDRATE);

    //Sensor Inercial
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(200000); //NOTE: Ajustar de acordo com arduino utilizado
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif
    iniciar_sensor_inercial();
}

void loop() {
    //Menu
    currentMillis = millis();
    if (currentMillis - previousMPUMillis >= mpu_interval) {
        previousMPUMillis = currentMillis;
        ler_sensor_inercial(); //Realiza leitura e envia pacote(ou mostra) dados
    }

    currentMillis = millis();
    if (currentMillis - previousPrintMillis >= print_interval) {
        previousPrintMillis = currentMillis;
        mostrar_sensor_inercial(); //Realiza leitura e envia pacote(ou mostra) dados
    }


    if(abs(euler[1] * 180/M_PI) < 10){
        digitalWrite(LED_PIN, 0); //E estabiliza
    } else if(euler[1] * 180/M_PI < -15){
        digitalWrite(LED_PIN, 1); //E sobe
    } else if(euler[1] * 180/M_PI > 15){
        digitalWrite(LED_PIN, 1); //E desce
    }
}


////////////////////
//Sensor Inercial //
////////////////////

void iniciar_sensor_inercial() {
    if (mpu.testConnection()) {
        mpu.initialize(); //Initializes the IMU
        uint8_t ret = mpu.dmpInitialize(); //Initializes the DMP
        delay(50);
        if (ret == 0) {
            mpu.setDMPEnabled(true);
            mpu.setXAccelOffset(-1099);
            mpu.setYAccelOffset(-14);
            mpu.setZAccelOffset(454);
            mpu.setXGyroOffset(83);
            mpu.setYGyroOffset(-48);
            mpu.setZGyroOffset(21);
            DEBUG_PRINT__("Sensor Inercial configurado com sucesso.\n");
        } else {
            //TODO: adicionar uma forma melhor de aviso. outro led?
            DEBUG_PRINT__("Erro na inicializacao do sensor Inercial!\n");
        }
    } else {
        DEBUG_PRINT__("Erro na conexao do sensor Inercial.\n");
    }
}

void ler_sensor_inercial() {
    numbPackets = floor(mpu.getFIFOCount() / PSDMP);
    //DEBUG_PRINT__(numbPackets); DEBUG_PRINT__(" - ");
    if (numbPackets >= 24) {
        mpu.resetFIFO();
        DEBUG_PRINT__("FIFO sensor 1 overflow!\n"); //TODO: mostrar isso de alguma forma. outro led?
    } else {
        while (numbPackets > 0) {
            mpu.getFIFOBytes(fifoBuffer, PSDMP);
            numbPackets--;
        }
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);

        mpu.dmpGetEuler(euler, &q);


        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    }
}

void mostrar_sensor_inercial(){
    Serial.print("euler\t");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print("\t");
    Serial.print(-euler[2] * 180/M_PI);
    Serial.print("\t");

    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
}

void enviar_pacote_inercial() {
    #ifndef DEBUG_MODE
    //Assembling packet and sending
    serial_buffer_out[0] = UART_START;
    serial_buffer_out[1] = UART_PQUAT;
    serial_buffer_out[2] = fifoBuffer[0]; //qw_msb
    serial_buffer_out[3] = fifoBuffer[1]; //qw_lsb
    serial_buffer_out[4] = fifoBuffer[4]; //qx_msb
    serial_buffer_out[5] = fifoBuffer[5]; //qx_lsb
    serial_buffer_out[6] = fifoBuffer[8]; //qy_msb
    serial_buffer_out[7] = fifoBuffer[9]; //qy_lsb
    serial_buffer_out[8] = fifoBuffer[12]; //qz_msb
    serial_buffer_out[9] = fifoBuffer[13]; //qz_lsb
    serial_buffer_out[10] = UART_END;
    Serial.write(serial_buffer_out, 11);
    #endif /*DEBUG_MODE*/
    #ifdef DEBUG_MODE
    float q[4], a[3], g[3];
    //Quaternion
    q[0] = (float) ((fifoBuffer[0] << 8) | fifoBuffer[1]) / 16384.0f;
    q[1] = (float) ((fifoBuffer[4] << 8) | fifoBuffer[5]) / 16384.0f;
    q[2] = (float) ((fifoBuffer[8] << 8) | fifoBuffer[9]) / 16384.0f;
    q[3] = (float) ((fifoBuffer[12] << 8) | fifoBuffer[13]) / 16384.0f;
    //Aceleracao
    a[0] = (float) ((fifoBuffer[28] << 8) | fifoBuffer[29]) / 8192.0f;
    a[1] = (float) ((fifoBuffer[32] << 8) | fifoBuffer[33]) / 8192.0f;
    a[2] = (float) ((fifoBuffer[36] << 8) | fifoBuffer[37]) / 8192.0f;
    //Giroscopio
    g[0] = (float) ((fifoBuffer[16] << 8) | fifoBuffer[17]) / 131.0f;
    g[1] = (float) ((fifoBuffer[20] << 8) | fifoBuffer[21]) / 131.0f;
    g[2] = (float) ((fifoBuffer[24] << 8) | fifoBuffer[25]) / 131.0f;
    DEBUG_PRINT__("Quat-Accel-Gyro:\t");
    //Quaternions
    DEBUG_PRINT__(q[0]);
    DEBUG_PRINT__("\t");
    DEBUG_PRINT__(q[1]);
    DEBUG_PRINT__("\t");
    DEBUG_PRINT__(q[2]);
    DEBUG_PRINT__("\t");
    DEBUG_PRINT__(q[3]);
    DEBUG_PRINT__("\t-\t");
    //accel in G
    DEBUG_PRINT__(a[0]);
    DEBUG_PRINT__("\t");
    DEBUG_PRINT__(a[1]);
    DEBUG_PRINT__("\t");
    DEBUG_PRINT__(a[2]);
    DEBUG_PRINT__("\t-\t");
    //g[1]ro in degrees/s
    DEBUG_PRINT__(g[0]);
    DEBUG_PRINT__("\t");
    DEBUG_PRINT__(g[1]);
    DEBUG_PRINT__("\t");
    DEBUG_PRINT__(g[2]);
    DEBUG_PRINT__("\n");
    #endif /*DEBUG_MODE*/
}

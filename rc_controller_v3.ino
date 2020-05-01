/*
 * RC카 !
 * 
 * 모드 1 : 자이로 센서를 통한 전후좌우 등속 제어
 * 모드 2 : 자이로 센서를 통한 전후좌우 가변속 제어 (플렉스 센서 로 엑셀)
 * 모드 3 : 자이로 센서를 통한 전후좌우 가변속 제어 (소리 센서 로 엑셀)
 * 모드 4 : 자이로 센서를 통한 전후 제어, (플레스 센서 값 > constant 일 때, 180도 회전 구현
 * 1, 3 -> LED LOW        2, 4 -> LED HIGH
 * 
 * - 컨트롤러 부분 핀 (D 2개, A 4개)
 * 자이로센서 A4, A5
 * 사운드센서 A0
 * 플렉스센서 A?
 * 블루투스 D7, D8
 * 
 * 
 * - RC카 핀 (D 6개, A2개)
 * L298n 디지털 4개, 아날로그 2개
 * 블루투스 D7, D8
 * 
 */


#include <SoftwareSerial.h>
#include <SimpleTimer.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define BT_RX 8//hc06's tx -> mcu's d8
#define BT_TX 7//hc06's rx -> mcu's d7
#define FLEXPIN A3
#define SOUNDPIN A0

SoftwareSerial bluetooth(BT_RX, BT_TX);
unsigned mode = 1;
int flexVal = 0;
int soundVal = 0;

int current_x = 0;
int current_y = 0;
int current_yaw = 0;
int last_yaw = 0;
SimpleTimer timer;

MPU6050 mpu;

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
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };





// ================================================================
// ===                         Functions                        ===
// ================================================================
void dmpDataReady();
void print_yawpitchroll();
void sound_calculate();
void flex_calculate();
void signal_transmit();
void mode_changer();





// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);
    bluetooth.begin(9600);
    
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(SOUNDPIN, INPUT);
    pinMode(FLEXPIN, INPUT);
    pinMode(13, OUTPUT);

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    while (Serial.available() && Serial.read()); // empty buffer

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
    }

    timer.setInterval(100, print_yawpitchroll);
    timer.setInterval(100, signal_transmit);
    timer.setInterval(100, sound_calculate);
    timer.setInterval(100, flex_calculate);
    timer.setInterval(1500, mode_changer);
}





// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    timer.run();
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize)

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
        }
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        //Serial.print(ypr[0] * 180/M_PI);
        //yaw
        
        current_x = ( ypr[2] * 180/M_PI );
        current_y = ( ypr[1] * 180/M_PI );
    }
}





// ================================================================
// ===                         Functions                        ===
// ================================================================

void dmpDataReady() {
    mpuInterrupt = true;
}


void print_yawpitchroll(){
  Serial.print(mode);
  Serial.print("\t");
  Serial.print(current_x);
  Serial.print("\t");
  Serial.println(current_y);
}

void sound_calculate(){
  soundVal = analogRead(SOUNDPIN);
}

void flex_calculate(){
  flexVal = analogRead(FLEXPIN);
}

void mode_changer(){
  if(current_x > 70){
    switch(mode){
      case 1:{
        mode++;
        digitalWrite(13, HIGH);
        break;
      }
      case 2:{
        mode++;
        digitalWrite(13, LOW);
        break;
      }
      case 3:{
        mode++;
        digitalWrite(13, HIGH);
        break;
      }
      case 4:{
        mode = 1;
        digitalWrite(13, LOW);
        break;
      }
    }
    //bluetooth.write('m');
    bluetooth.print('m');
  }
}

void signal_transmit(){
  switch (mode) {
    //==============================
    case 1:{//자이로 등속제어
      if(current_y < -30){//우회전
        bluetooth.write('d');
      }
      else if(current_y > 30){//좌회전
        bluetooth.write('a');
      }
      else if(current_x < -20){//전진
        bluetooth.write('w');
      }
      else if(20 < current_x && current_x < 70){//후진
        bluetooth.write('s');
      }
      else{//정지
        bluetooth.write('e');
      }
      break;
    }
    
    //==============================
    case 2:{//자이로 플렉스 가변속제어
      if(current_y < -30){//우회전
        bluetooth.write('d');
      }
      else if(current_y > 30){//좌회전
        bluetooth.write('a');
      }
      else if(current_x < -20){//전진
        bluetooth.write('w');
      }
      else if(20 < current_x && current_x < 70){//후진
        bluetooth.write('s');
      }
      else{//정지
        bluetooth.write('e');
      }

      if(0<=flexVal && flexVal<200){
        bluetooth.write('1');
      }
      else if(200<flexVal && flexVal<400){
        bluetooth.write('2');
      }
      else if(400<flexVal && flexVal<600){
        bluetooth.write('3');
      }
      else if(600<flexVal && flexVal < 800){
        bluetooth.write('4');
      }
      else{
        bluetooth.write('5');
      }
      break;
    }
    

    //==============================
    case 3:{//자이로 소리 가변속제어
      if(current_y < -30){//우회전
        bluetooth.write('d');
      }
      else if(current_y > 30){//좌회전
        bluetooth.write('a');
      }
      else if(current_x < -20){//전진
        bluetooth.write('w');
      }
      else if(20 < current_x && current_x < 70){//후진
        bluetooth.write('s');
      }
      else{//정지
        bluetooth.write('e');
      }

      if(0<=flexVal && flexVal<200){//1단
        bluetooth.write('1');
      }
      else if(200<flexVal && flexVal<400){//2단
        bluetooth.write('2');
      }
      else if(400<flexVal && flexVal<600){//3단
        bluetooth.write('3');
      }
      else if(600<flexVal && flexVal < 800){//4단
        bluetooth.write('4');
      }
      else{//최고속도
        bluetooth.write('5');
      }
      break;
    }
    

    //==============================
    case 4:{//자이로 180도 등속 제어
      if(current_y < -30){//CW 180도
        bluetooth.write('r');
      }
      else if(current_y > 30){//CCW 180도
        bluetooth.write('l');
      }
      else if(current_x < -20){//전진
        bluetooth.write('w');
      }
      else if(20 < current_x && current_x < 70){//후진
        bluetooth.write('s');
      }
      else{//정지
        bluetooth.write('e');
      }
      break;
    }
    
  }
}

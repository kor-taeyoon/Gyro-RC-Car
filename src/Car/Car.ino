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
 * 플렉스센서 A3
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
#define BT_RX 8 //hc06's tx -> uno's d8
#define BT_TX 7 //hc06's rx -> uno's d7
#define MOTOR_A1 5
#define MOTOR_A2 6
#define MOTOR_B1 9
#define MOTOR_B2 10

SoftwareSerial bluetooth(BT_RX, BT_TX);
char mode = 'x';//mode 1은 온전한 가속계 제어 모드
char command = 0;
int current_PWM;
SimpleTimer timer;




// ================================================================
// ===                         Functions                        ===
// ================================================================
void go();
void back();
void left();
void right();
void STOP();
void process_data();





// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  timer.setInterval(50, process_data);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
}





// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  // put your main code here, to run repeatedly:
  timer.run();
}





// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void go(){
  analogWrite(MOTOR_A1, current_PWM);
  analogWrite(MOTOR_A2, 0); 
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, current_PWM); 
}
void back(){
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, current_PWM);
  analogWrite(MOTOR_B1, current_PWM);
  analogWrite(MOTOR_B2, 0);
}
void left(){
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, current_PWM);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, current_PWM);
}
void right(){
  analogWrite(MOTOR_A1, current_PWM);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, current_PWM);
  analogWrite(MOTOR_B2, 0);
}


void STOP(){
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}


void process_data() {
  if (bluetooth.available()) {
    command = bluetooth.read();
    Serial.println(command);
    if(command == '6'){
      mode = 'x';
    }
    if(command == '7'){
      mode = 'y';
    }
    if(command == '8'){
      mode = 'z';
    }
    if(command == '9'){
      mode = 't';
    }
    switch (mode){
      //==========
      case 'x':{//자이로 등속
        current_PWM = 200;
        switch (command){
          case 'w':{//전진
            go();
            break;
          }
          case 's':{//후진
            back();
            break;
          }
          case 'a':{//좌회전
            left();
            break;
          }
          case 'd':{//우회전
            right();
            break;
          }
          case 'e':{//정지
            STOP();
            break;
          }
        }
        break;
      }

      //==========
      case 'y':{//플렉스 엑셀
        switch (command){
          case '1':{//1단
            current_PWM = 80;
            break;
          }
          case '2':{//2단
            current_PWM = 120;
            break;
          }
          case '3':{//3단
            current_PWM = 160;
            break;
          }
          case '4':{//4단
            current_PWM = 200;
            break;
          }
          case '5':{//최고속도
            current_PWM = 255;
            break;
          }
          case 'w':{//전진
            go();
            break;
          }
          case 's':{//후진
            back();
            break;
          }
          case 'a':{//좌회전
            left();
            break;
          }
          case 'd':{//우회전
            right();
            break;
          }
          case 'e':{//정지
            STOP();
            break;
          }
        }
        break;
      }

      //==========
      case 'z':{//소리센서 트리거
        switch (command){
          case '1':{//동작
            current_PWM = 255;
            break;
          }
          case '5':{//미동작
            current_PWM = 0;
            break;
          }
          case 'w':{//전진
            go();
            break;
          }
          case 's':{//후진
            back();
            break;
          }
          case 'a':{//좌회전
            left();
            break;
          }
          case 'd':{//우회전
            right();
            break;
          }
          case 'e':{//정지
            STOP();
            break;
          }
        }
        break;
      }
      
      //==========
      case 't':{//좌우 조작 반전
        current_PWM = 255;
        switch (command){
          case 'w':{//전진
            go();
            break;
          }
          case 's':{//후진
            back();
            break;
          }
          case 'l':{//좌회전    인데 지금은 우회전
            right();
            break;
          }
          case 'r':{//우회전    인데 지금은 좌회전
            left();
            break;
          }
          case 'e':{//정지
            STOP();
            break;
          }
        }
        break;
      }
    }
  }
  command = 0;
}

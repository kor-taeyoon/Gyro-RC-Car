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
#define BT_RX 8 //hc06's tx -> uno's d8
#define BT_TX 7 //hc06's rx -> uno's d7
#define MOTOR_A1 5
#define MOTOR_A2 6
#define MOTOR_APWM A0
#define MOTOR_B1 9
#define MOTOR_B2 10
#define MOTOR_BPWM A1

SoftwareSerial bluetooth(BT_RX, BT_TX);
unsigned mode = 1;//mode 1은 온전한 가속계 제어 모드
char command = 0;
SimpleTimer timer;





// ================================================================
// ===                         Functions                        ===
// ================================================================
void velo(int APWM, int BPWM);
void go();
void back();
void left();
void right();
void CW();
void CCW();
void STOP();
void process_data();





// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  timer.setInterval(50, process_data);
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
void velo(int APWM, int BPWM){
  digitalWrite(MOTOR_APWM, APWM);
  digitalWrite(MOTOR_BPWM, BPWM);
}

void go(){
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}
void back(){
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}
void left(){
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}
void right(){
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}

void CW(){
  right();
  velo(255,255);
  delay(1000);
}
void CCW(){
  left();
  velo(255,255);
  delay(1000);
}

void STOP(){
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_APWM, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, LOW);
  digitalWrite(MOTOR_BPWM, LOW);
}


void process_data() {
  if (bluetooth.available()) {
    command = bluetooth.read();
    if(command == 'm'){
      mode++;
      if(mode == 5){
        mode = 1;
      }
    }
    switch (mode){
      case 1:{//
        switch (command){
          //==========
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
      case 2:{//플렉스 엑셀
        switch (command){
          //==========
          case '1':{//1단
            velo(50, 50);
            break;
          }
          case '2':{//2단
            velo(100, 100);
            break;
          }
          case '3':{//3단
            velo(150, 150);            
            break;
          }
          case '4':{//4단
            velo(200, 200);
            break;
          }
          case '5':{//최고속도
            velo(255, 255);
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
      case 3:{//소리센서로 엑셀
        switch (command){
          //==========
          case '1':{//1단
            velo(50, 50);
            break;
          }
          case '2':{//2단
            velo(100, 100);
            break;
          }
          case '3':{//3단
            velo(150, 150);            
            break;
          }
          case '4':{//4단
            velo(200, 200);
            break;
          }
          case '5':{//최고속도
            velo(255, 255);
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
      case 4:{//좌우 180도 CW, CCW
        switch (command){
          //==========
          case 'w':{//전진
            go();
            break;
          }
          case 's':{//후진
            back();
            break;
          }
          case 'a':{//좌회전
            CCW();
            break;
          }
          case 'd':{//우회전
            CW();
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

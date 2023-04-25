#include <Wire.h>
#include <AFMotor.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

String buff;

void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial.println("Let's go!");
}

void goForward(float v){//GO FORWARD
  Serial.println("goForward");
  motor1.setSpeed(v);
  motor1.run(FORWARD);
  motor2.setSpeed(v);
  motor2.run(FORWARD);
  motor3.setSpeed(v);
  motor3.run(FORWARD);
  motor4.setSpeed(v);
  motor4.run(FORWARD);
  //delay(100);
}
void goBack(float v){//GO BACK
  Serial.print("Backward");
  motor1.setSpeed(v);
  motor1.run(BACKWARD);
  motor2.setSpeed(v);
  motor2.run(BACKWARD);
  motor3.setSpeed(v);
  motor3.run(BACKWARD);
  motor4.setSpeed(v);
  motor4.run(BACKWARD);
  //delay(100);
}
void forwardRight(float v){//GO forward diagonal right
  motor1.setSpeed(v);
  motor1.run(FORWARD);
  motor2.setSpeed(v);
  motor2.run(RELEASE);
  motor3.setSpeed(v);
  motor3.run(RELEASE);
  motor4.setSpeed(v);
  motor4.run(FORWARD);
  //delay(100);
}
void forwardLeft(float v){// GO forward diagonal left
  motor1.setSpeed(v);
  motor1.run(RELEASE);
  motor2.setSpeed(v);
  motor2.run(FORWARD);
  motor3.setSpeed(v);
  motor3.run(FORWARD);
  motor4.setSpeed(v);
  motor4.run(RELEASE);
  //delay(100);
}  
void backwardLeft(float v){// GO backward diagonal left
  motor1.setSpeed(v);
  motor1.run(BACKWARD);
  motor2.setSpeed(v);
  motor2.run(RELEASE);
  motor3.setSpeed(v);
  motor3.run(RELEASE);
  motor4.setSpeed(v);
  motor4.run(BACKWARD);
  //delay(100);
}
void backwardRight(float v){// GO backward diagonal right
  motor1.setSpeed(v);
  motor1.run(RELEASE);
  motor2.setSpeed(v);
  motor2.run(BACKWARD);
  motor3.setSpeed(v);
  motor3.run(BACKWARD);
  motor4.setSpeed(v);
  motor4.run(RELEASE);
  //delay(100);
}  
void goRight(float v){// GO right
  motor1.setSpeed(v);
  motor1.run(FORWARD);
  motor2.setSpeed(v);
  motor2.run(BACKWARD);
  motor3.setSpeed(v);
  motor3.run(BACKWARD);
  motor4.setSpeed(v);
  motor4.run(FORWARD);
  //delay(100);
}
void goLeft(float v){// GO left
  motor1.setSpeed(v);
  motor1.run(BACKWARD);
  motor2.setSpeed(v);
  motor2.run(FORWARD);
  motor3.setSpeed(v);
  motor3.run(FORWARD);
  motor4.setSpeed(v);
  motor4.run(BACKWARD);
  //delay(100);
} 
void goTwirl(float v){// quay
  Serial.println("quayPhai");
  motor1.setSpeed(v);
  motor1.run(FORWARD);
  motor2.setSpeed(v);
  motor2.run(BACKWARD);
  motor3.setSpeed(v);
  motor3.run(FORWARD);
  motor4.setSpeed(v);
  motor4.run(BACKWARD);
  //delay(100);
}
void goStop(float v){
  motor1.setSpeed(v);
  motor1.run(RELEASE);
  motor2.setSpeed(v);
  motor2.run(RELEASE);
  motor3.setSpeed(v);
  motor3.run(RELEASE);
  motor4.setSpeed(v);
  motor4.run(RELEASE);
  //delay(100);
}

void loop() {
  if (Serial.available()) //Nếu có tín hiệu từ Pi
  {
    buff = Serial.readStringUntil('\r'); //Đọc vào đến khi gặp \r (xuống dòng)
    
    if (buff=="forward"){
      goForward(150);
    }else if(buff == "backward"){
      goBack(150);
    }else if(buff == "right"){
      goRight(150);
    }else if(buff == "left"){
      goLeft(150);
    }else if(buff == "twirl"){
      goTwirl(150);
    }else if(buff == "stop"){
      goStop(150);
    }
  }
}

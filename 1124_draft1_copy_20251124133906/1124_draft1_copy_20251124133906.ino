
#include "Timer.h"
const int AIN1  = 10;
const int AIN2  = 9;
const int PWMA  = 11;
const int BIN1  = 7;
const int BIN2  = 6;
const int PWMB  = 5;
const int STDBY = 8;
const int L_sens = A1;
const int M_sens = A2;
const int R_sens = A3;

const bool L_invert = true;
const bool R_invert = false;

// motor speeds
int speedL = 0;
int speedR = 0;

// time stamps
const int beg_rumba = 127000; //12500
const int end_rumba = 18650;
const int beg_corner = 21300;

float prevError = 0;
unsigned long lastTime = 0;

Timer timer;
Timer t_turn;
float errorBias = 0;
int directionBias = 0;   // +1 = push right, -1 = push left


void setup()  {
  Serial.begin(9600);
  pinMode(L_sens, INPUT);
  pinMode(M_sens, INPUT);
  pinMode(R_sens, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH);

  timer.start(); //start timer
}

void loop(){

  int L_sensVal;
  int M_sensVal;
  int R_sensVal;

  while (timer.read() < beg_rumba){ // from beginning to the curve, activate normal drive.
    L_sensVal = map(analogRead(L_sens), 0, 1000, 0, 100);
    M_sensVal = map(analogRead(M_sens), 0, 1000, 0, 100);
    R_sensVal = map(analogRead(R_sens), 0, 1000, 0, 100);
    int error = getSmartError(L_sensVal, M_sensVal, R_sensVal);
    //Serial.println(error);
    int turn = PD_turn(-error, 7.0, .5);   
    if (isLineLost(L_sensVal, M_sensVal, R_sensVal)) diffDrive(50, turn);
    else diffDrive(230, turn);

    // if (L_sensVal < (R_sensVal-7)) {
    //   diffDrive(150, -error * 3);  // forward_speed, turn_amount

    // }

    // if (R_sensVal < (L_sensVal-7)) { //7
    //    diffDrive(150, -error * 3);  // forward_speed, turn_amount

    // }

    // if (L_sensVal < (R_sensVal+7) && L_sensVal > (R_sensVal-7)) {
    //   speedL = 150; //230,232
    //   speedR = 150;
    //   diffDrive(150, 0);
    // }

    
  }


  while (timer.read() > beg_rumba && timer.read() < end_rumba){ // RUMBA while in the curve, activate slow drive, between 27 to 40 secs

  L_sensVal = map(analogRead(L_sens), 0, 1000, 0, 100);
  M_sensVal = map(analogRead(M_sens), 0, 1000, 0, 100);
  R_sensVal = map(analogRead(R_sens), 0, 1000, 0, 100);

  int error = L_sensVal - R_sensVal;

    if (abs(error) < 7) {
      diffDrive(80, 0);
    } else if (error > 7) {
      diffDrive(60, error * 2);  
    } else {
      diffDrive(60, error * 2);
    }

    // if (L_sensVal < (R_sensVal-7) && L_sensVal > (R_sensVal-10)) { //small adjustment, left is off track
    //   speedL = 130; //(140,50)(120,70)
    //   speedR = 65;
    // }
    // else if (L_sensVal < (R_sensVal-10)) { // big adjustment, left is off track
    //   speedL = 225; //(225,-115)
    //   speedR = -115;
    // }

    // if (R_sensVal < (L_sensVal-7)&& R_sensVal > (L_sensVal-10)) { // small adjustment, right is off
    //   speedR = 130;
    //   speedL = 65;
    // }
    // else if (R_sensVal < (L_sensVal-10)) { // big adjustment, right is off
    //   speedR = 225;
    //   speedL = -115;
    // }

    // if (L_sensVal < (R_sensVal+7) && L_sensVal > (R_sensVal-7)) { // in the range
    //   speedL = 70;
    //   speedR = 82;
    // }

    // drive(speedL, speedR);
  }

  while (timer.read() > end_rumba && timer.read() < beg_corner){ //Past the curve, activate normal drive. gaps
    L_sensVal = map(analogRead(L_sens), 0, 1000, 0, 100);
    M_sensVal = map(analogRead(M_sens), 0, 1000, 0, 100);
    R_sensVal = map(analogRead(R_sens), 0, 1000, 0, 100);

    if (L_sensVal < (R_sensVal-7)) {
      speedL = 155;
      speedR = 55;
    }

    if (R_sensVal < (L_sensVal-7)) {
      speedR = 155;
      speedL = 55;
    }

    if (L_sensVal < (R_sensVal+7) && L_sensVal > (R_sensVal-7)) {
      speedL = 130; //120,  130
      speedR = 142; //122
    }

    drive(speedL, speedR);
  }


  while (timer.read() > beg_corner){ // everything after corners, corners included
    L_sensVal = map(analogRead(L_sens), 0, 1000, 0, 100);
    M_sensVal = map(analogRead(M_sens), 0, 1000, 0, 100);
    R_sensVal = map(analogRead(R_sens), 0, 1000, 0, 100);

    if (L_sensVal < (R_sensVal-8)) { //small adjustment, left is off track
      speedL = 140;
      speedR = 50;
    }


    if (R_sensVal < (L_sensVal-7)) { // small adjustment, right is off
      speedR = 180;
      speedL = 50;
    }


    if (M_sensVal > 55) {
      speedL = 90; //70
      speedR = 110; //90
    }

    if (L_sensVal < (R_sensVal+5) && L_sensVal > (R_sensVal-8)) { // in the range
      speedL = 145; //70,70, 110  130
      speedR = 145;
    }

    drive(speedL, speedR);

    if (M_sensVal < 40 && L_sensVal < 40 && R_sensVal < 40){ //Barbline is off track
    uint32_t star_time = timer.read();
    while((timer.read()-star_time) < 250){ //turn left for 350 ms, check for path along the way.
        L_sensVal = map(analogRead(L_sens), 0, 1000, 0, 100);
        M_sensVal = map(analogRead(M_sens), 0, 1000, 0, 100);
        R_sensVal = map(analogRead(R_sens), 0, 1000, 0, 100);
        drive(-150,150);
        if (M_sensVal > 40) {
          break;
        }
    }
    while (M_sensVal < 40){
      M_sensVal = map(analogRead(M_sens), 0, 1000, 0, 100);
      drive(200, -200); //80,-80 120 150

    }
    }
  }
}

//----------------------------------------------
void motorWrite(int motorSpeed, int xIN1, int xIN2, int PWMx)
{

  if (motorSpeed > 0)          // it's forward
  {  digitalWrite(xIN1, LOW);
     digitalWrite(xIN2, HIGH);
  }
  else                         // it's reverse
  {  digitalWrite(xIN1, HIGH);
     digitalWrite(xIN2, LOW);
  }

    motorSpeed = abs(motorSpeed);
    motorSpeed = constrain(motorSpeed, 0, 255);   // Just in case...
    analogWrite(PWMx, motorSpeed);
}

void drive(int spdL, int spdR){
  if (L_invert==true){
  motorWrite(-spdL, AIN1, AIN2, PWMA);}
  else{ motorWrite(spdL, AIN1, AIN2, PWMA);}

  if (R_invert==true){
  motorWrite(-spdR, BIN1, BIN2, PWMB);}
  else{ motorWrite(spdR, BIN1, BIN2, PWMB);}
  }

void diffDrive(int forward, int turn) {
  int L = forward + turn;
  int R = forward - turn;

  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  drive(L, R);
}

int PD_turn(int error, float Kp, float Kd) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt == 0) dt = 0.001;  // don't divide by zero like a muppet

    float dError = (error - prevError) / dt;

    float turn = Kp * error + Kd * dError;

    prevError = error;
    lastTime = now;
    return constrain(turn, -255, 255);
}

int getSmartError(int L, int M, int R) {
    static int directionBias = 0;
   
    int error = L - R;

    // track direction
    if (error > 20) directionBias = +1;
    else if (error < 20) directionBias = -1;
    else directionBias = 0;

    // detect total loss
    bool lost = isLineLost(L,M,R);

    // if blind, shove error hard in previous direction
    if (lost) {
        error = directionBias * 15;  // tune this value!

    }

    return error;
}
bool isLineLost(int L, int M, int R) {
  return (L < 75 && M < 75 && R < 75);  // tune threshold if needed
}


// void drive_original()
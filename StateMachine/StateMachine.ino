
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

int gapCounter = 0;

// time stamps
const int beg_rumba = 127000; //12500
const int end_rumba = 18650;
const int beg_corner = 21300;

float prevError = 0;
unsigned long lastTime = 0;
float kp, ki, kd;


Timer timer;
Timer t_turn;
float errorBias = 0;
int directionBias = 0;   // +1 = push right, -1 = push left

enum State {
  START_STRAIGHT,
  FIRST_CURVE,
  FIRST_TURN,
  SECOND_TURN,
  RUMBA,
  ONE_EIGHTY,
  SHARP_SQUIGGLES,
  MISSING_LINE,
  LEFT_90,
  THREE_SIXTY,
  RHOMBUS,
  CIRCLE,
  DONE
};

State state = START_STRAIGHT;
unsigned long stateStart = 0;

// PID STUFF
float integral = 0;
unsigned long lastPID = 0;



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

void loop() {
  unsigned long t = timer.read();     // your existing timer
  unsigned long now = millis();

  // pre-read sensors (don’t read 20 times like a psychopath)
  int L = map(analogRead(L_sens), 0, 1000, 0, 100);
  int M = map(analogRead(M_sens), 0, 1000, 0, 100);
  int R = map(analogRead(R_sens), 0, 1000, 0, 100);

  int error = L - R;   // your original error model
  int turn  = PID_turn(error, kp, ki, kd);  // tune these later

  //---------------------------------------------
  //        STATE MACHINE (PURE TIME)
  //---------------------------------------------
  switch (state) {

    // -------------------------------------------
    case START_STRAIGHT:
      kp = 2.0; ki = 0.0; kd = 1.0; 
      diffDrive(255, turn);
      if (t > 1000) advanceState(FIRST_CURVE);
      break;

    // -------------------------------------------
    case FIRST_CURVE:
      kp = 2.0; ki = 0.0; kd = 1.0; 
      diffDrive(255, turn);

      if (t > 2000) advanceState(FIRST_TURN);
      break;

    // -------------------------------------------
    case FIRST_TURN:
      kp = 10.0; ki = 0.0; kd = 1.0;  
      diffDrive(200, turn);

      if (-error > 50) advanceState(SECOND_TURN);
      if (t > 3000) advanceState(SECOND_TURN);
      break;

    // -------------------------------------------
    case SECOND_TURN:
      kp = 15.0; ki = 0.00005; kd = 1.0;  
      diffDrive(140, turn * 1.3);
      if (t > 5000) advanceState(RUMBA);
      break;

    // -------------------------------------------
    case RUMBA:
      kp = 1.0; ki = 0.00005; kd = 1.0;
      diffDrive(120, turn * 0.8);
      if (-error > 50) advanceState(ONE_EIGHTY);
      if (t > 5500) advanceState(ONE_EIGHTY);
      break;

    // -------------------------------------------
    case ONE_EIGHTY:
      kp = 10.0; ki = 0.00005; kd = 1.0;
      diffDrive(0, turn * 2);   // spin
      if (t > 6000) advanceState(SHARP_SQUIGGLES);
      break;

    // -------------------------------------------
    case SHARP_SQUIGGLES:
      kp = 15.0; ki = 0.0; kd = 1.0;
      diffDrive(200, turn * 1.4);
      // SENSOR TRANSITION:
      // Instead of time, we check if the line has disappeared.
      if (isLineLost(L, M, R)) {
        gapCounter++; 
      } else {
        gapCounter = 0; // Reset if we see a speck of black
      }

      // If we have seen white for enough consecutive loops, switch state
      if (gapCounter > 5) {
        gapCounter = 0; // Reset for next time
        advanceState(MISSING_LINE);
      }
      if (t > 8600) advanceState(MISSING_LINE); 
      break;

    // -------------------------------------------
    case MISSING_LINE:
      kp = 2.0; ki = 0.0; kd = 1.0; 
      
      if (isLineLost(L, M, R)) diffDrive(255, 0);
      
      else diffDrive(255, turn)
      
      if (!isLineLost(L, M, R)) {
        gapCounter++; 
      } 
      else {
        gapCounter = 0; // Reset if we see a speck of black
      }

      // If we have seen black for enough consecutive loops, switch state
      if (gapCounter > 15) {
        gapCounter = 0; // Reset for next time
        advanceState(LEFT_90);
      }
      if (t > 9000) advanceState(MISSING_LINE); 
      break;


      
    // -------------------------------------------
    case LEFT_90:
      kp = 3.0; ki = 0.0; kd = 1.0;
      if (isLineLost(L, M, R)) diffDrive(0, -180);
      else diffDrive(255, turn);
      if (t > 10000) advanceState(THREE_SIXTY);
      break;

    // -------------------------------------------
    case THREE_SIXTY:
      diffDrive(0, 255);
      if (t > 11000) advanceState(RHOMBUS);
      break;

    // -------------------------------------------
    case RHOMBUS:
      kp = 5.0; ki = 0.0; kd = 1.0;
      diffDrive(255, turn);
      if (t > 14000) advanceState(CIRCLE);
      break;

    // -------------------------------------------
    case CIRCLE:
      diffDrive(255, 80);   // orbit-like
      if (t > 16000) advanceState(DONE);
      break;

    // -------------------------------------------
    case DONE:
      kp = 5.0; ki = 0.0; kd = 1.0;
      diffDrive(255, turn);
      break;
  }
}


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


int PID_turn(int error, float Kp, float Ki, float Kd) {
  unsigned long now = millis();
  float dt = (now - lastPID) / 1000.0;
  if (dt == 0) dt = 0.001;

  integral += error * dt;
  float dError = (error - prevError) / dt;

  float turn = Kp * error + Ki * integral + Kd * dError;

  prevError = error;
  lastPID = now;

  return constrain(turn, -200, 200);
}

int getSmartError(int L, int M, int R) {
    static int directionBias = 0;
   
    int error = L - R;

    // track direction
    if (error > 20) directionBias = +1;
    else if (error < 20) directionBias = -1;
    else directionBias = 0;

    // detect total loss
    // bool lost = isLineLost(L,M,R);

    // // if blind, shove error hard in previous direction
    // if (lost) {
    //     error = directionBias * 15;  // tune this value!

    // }

    return error;
}
bool isLineLost(int L, int M, int R) {
  return (L < 75 && M < 75 && R < 75);  // tune threshold if needed
}

void advanceState(State next) {
    state = next;
    stateStart = timer.read();   // reset state timestamp

}




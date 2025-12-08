
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
const int RED  = 2;
const int YELLOW  = 3;
const int GREEN = 4;
const int LOST = 13;
const int ALERT = 12;

const bool L_invert = true;
const bool R_invert = false;

// motor speeds
int speedL = 0;
int speedR = 0;

int gapCounter = 0;


float prevError = 0;
unsigned long lastTime = 0;
float kp, ki, kd;

int error = 0;
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

//State state = RUMBA;
//State state = ONE_EIGHTY;

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

  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(LOST, OUTPUT);
  pinMode(ALERT, OUTPUT);


  timer.start(); //start timer
}

void loop() {
  unsigned long t = timer.read();     // your existing timer
  unsigned long now = millis();

  // pre-read sensors 
  int L = map(analogRead(L_sens), 0, 1000, 0, 100);
  int M = map(analogRead(M_sens), 0, 1000, 0, 100);
  int R = map(analogRead(R_sens), 0, 1000, 0, 100);


  // Serial.print("L:");
  // Serial.print(L);
  // Serial.print("  M:");
  // Serial.print(M);
  // Serial.print("  R:");
  // Serial.print(R);
  // Serial.print(" Error: ");
  // Serial.println(R - L);


  if (state == RUMBA) error = R-(L*1.25);//M + R - 255 + 7 * L;
  else if (state == FIRST_CURVE) error = R-(L*1.05);
    else if (state==FIRST_TURN) error = ((R+10)-L);
    else if ( state == CIRCLE) error = ((R*1.2)-L);
    else error = (R - L);   // your original error model
  int turn  = PID_turn(error, kp, ki, kd);  // tune these later

  if (isLineLost(L, M, R)) digitalWrite(LOST, HIGH);
     else digitalWrite(LOST, LOW);

  //---------------------------------------------
  //        STATE MACHINE 
  //---------------------------------------------
  switch (state) {

    // -------------------------------------------
    case START_STRAIGHT:
      digitalWrite(GREEN, HIGH);
      kp = 6.0; ki = 0.0; kd = 5.0; 
      diffDrive(255, turn * .3);
      if (t > 900) advanceState(FIRST_CURVE);
      break;

    // -------------------------------------------
    case FIRST_CURVE:
      digitalWrite(GREEN, LOW);
      digitalWrite(YELLOW, HIGH);
      kp = 30.0; ki = 0.0000; kd = 15.0; 
      if (abs(turn) > 30) 
      {diffDrive(255, turn*1.22);
       digitalWrite(ALERT, HIGH);
      }
      else  if (abs(turn) > 22) 
      {diffDrive(255, turn*1.1);
       digitalWrite(ALERT, HIGH);
      }
      else {diffDrive(255, turn*.2);
        digitalWrite(ALERT, LOW);}


      if (t - stateStart > 2500) advanceState(FIRST_TURN);
      break;

    // -------------------------------------------
    case FIRST_TURN:
      digitalWrite(GREEN, HIGH);
      digitalWrite(YELLOW, HIGH);
      kp = 20.0; ki = 0.0005; kd = 15.0;  
      
      if ((t-stateStart > 1450) && (t - stateStart < 2200)){

        digitalWrite(ALERT, HIGH);

        diffDrive(255, (turn*1.1) + 90);}

        
      
      else 
      {diffDrive(255, turn * .8);
      digitalWrite(ALERT, LOW);}
      // if (-error > 40) {
      //   digitalWrite(ALERT, HIGH);
      //   advanceState(SECOND_TURN);

      // }
      if (t - stateStart > 3550) advanceState(SECOND_TURN);
      break;

    // -------------------------------------------
    case SECOND_TURN:
      digitalWrite(GREEN, LOW);
      digitalWrite(YELLOW, LOW);
      digitalWrite(RED, HIGH);
      kp = 20.0; ki = 0.0001; kd = 15;  
      //diffDrive(250, turn * 2);
      if (t - stateStart > 550) diffDrive(250, turn * 1);
      else diffDrive(250, (turn * 1.1)+ 150);

      if (t - stateStart > 1250) advanceState(RUMBA);
      break;

    // -------------------------------------------
    case RUMBA:
      digitalWrite(ALERT, LOW);
      digitalWrite(GREEN, HIGH);
      digitalWrite(YELLOW, LOW);
      digitalWrite(RED, HIGH);
      //kp = 3.0; ki = 0.0002; kd = 1.5;
      kp = 4; ki = 0.000; kd = 10;

      //if (isLineLost(L, M, R)) drive (240, 255);
      //else diffDrive(255, turn);
      diffDrive(255, turn);
      if (-error > 60) {
        digitalWrite(ALERT, HIGH);
        advanceState(ONE_EIGHTY);}
      
      if (t - stateStart > 3090) advanceState(ONE_EIGHTY);
      break;

    // -------------------------------------------
    case ONE_EIGHTY:
      digitalWrite(GREEN, LOW);
      digitalWrite(YELLOW, HIGH);
      digitalWrite(RED, HIGH);
      digitalWrite(ALERT, LOW);
      kp = 15.0; ki = 0.00005; kd = 4.0;
      if (t - stateStart < 640) diffDrive(180, 200);   // spin
      if (t - stateStart > 650) diffDrive(210, turn);
      if (t - stateStart > 1200) advanceState(SHARP_SQUIGGLES);
      break;

    // -------------------------------------------
    case SHARP_SQUIGGLES:
      
      digitalWrite(GREEN, HIGH);
      digitalWrite(YELLOW, HIGH);
      digitalWrite(RED, HIGH);
      kp = 15.0; ki = 0.001; kd = 2.0;
      if (t - stateStart < 900 && isLineLost(L,M,R)) {
      diffDrive(255, (turn * .4) - 50);
      digitalWrite(ALERT, HIGH);
      }
      else if (t - stateStart < 900){
        diffDrive(250, turn * 1.4 + 100);
      }
      else if (t - stateStart <= 1100 && t - stateStart >= 900) {
      diffDrive(250, turn * 1.4 + 100);
      digitalWrite(ALERT, LOW); }

      else if (t - stateStart < 1800 && t - stateStart > 1100) {
      diffDrive(250, turn * 1.4);
      digitalWrite(ALERT, LOW); }


      else 
      {diffDrive (255, turn *.7);
      digitalWrite(ALERT, HIGH);}

      //SENSOR TRANSITION:
      //Instead of time, we check if the line has disappeared.
      if (isLineLost(L, M, R)) {
        gapCounter++; 
      } else {
        gapCounter = 0; // Reset if we see a speck of black
      }

      // If we have seen white for enough consecutive loops, switch state
      if ((gapCounter > 400) && (t - stateStart) > 2000) {
        digitalWrite(ALERT, HIGH);
        gapCounter = 0; // Reset for next time
        advanceState(MISSING_LINE);
      }
      //if (t - stateStart > 13000) advanceState(MISSING_LINE); 
      break;

    // -------------------------------------------
    case MISSING_LINE:
      digitalWrite(ALERT, LOW);
      digitalWrite(GREEN, HIGH);
      digitalWrite(YELLOW, LOW);
      digitalWrite(RED, LOW);
      kp = 10.0; ki = 0.00005; kd = 4.0; 
      
      if (isLineLost(L, M, R)) diffDrive(150, -10);
      
      else diffDrive(200, turn);
      
      if (!isLineLost(L, M, R)) {
        gapCounter++; 
      } 
      else {
        gapCounter = 0; // Reset if we see a speck of black
      }

      // If we have seen black for enough consecutive loops, switch state
       if (gapCounter > 400) {
         digitalWrite(ALERT, HIGH);
         gapCounter = 0; // Reset for next time
         advanceState(LEFT_90);
       }
      if (t - stateStart > 300) advanceState(LEFT_90); 
      break;


      
    // -------------------------------------------
    case LEFT_90:
      digitalWrite(GREEN, LOW);
      digitalWrite(YELLOW, HIGH);
      digitalWrite(RED, LOW);
      kp = 15.0; ki = 0.0; kd = 3.0;
      if (t - stateStart < 580 && t - stateStart > 300) diffDrive(0, -180);
      //kp = 30.0; ki = 0.0005; kd = 1.0;
      //if (t - stateStart < 280) diffDrive(100, turn);
      else diffDrive(255, turn);
      if (t - stateStart > 1480) advanceState(THREE_SIXTY);
      break;

    // -------------------------------------------
    case THREE_SIXTY:
      digitalWrite(ALERT, LOW);
      digitalWrite(GREEN, HIGH);
      digitalWrite(YELLOW, HIGH);
      digitalWrite(RED, LOW);
      diffDrive(255, 190);
      if (t - stateStart > 1500) advanceState(RHOMBUS);
      break;

    // -------------------------------------------
    case RHOMBUS:
      digitalWrite(GREEN, LOW);
      digitalWrite(YELLOW, LOW);
      digitalWrite(RED, HIGH);
      kp = 30.0; ki = 0.000; kd = 20.0;
      if ((t- stateStart < 500) || ((t - stateStart > 1600) && (t - stateStart < 2150)) || ((t - stateStart > 1000) && (t - stateStart < 1200))) {
        diffDrive(255, turn*1.2);
        digitalWrite(ALERT, HIGH);}

      

      else 
      {diffDrive(255, turn * 0.5);
      digitalWrite(ALERT, LOW);}
      
      if (isLineLost(L, M, R)) {
         gapCounter++; 
       } else {
         gapCounter = 0; // Reset if we see a speck of black
       }

      // // If we have seen white for enough consecutive loops, switch state
      if ((gapCounter > 10) && (t - stateStart > 2300)) {
        digitalWrite(ALERT, HIGH);
        gapCounter = 0; // Reset for next time
        advanceState(CIRCLE);}
      // }
      if (t - stateStart > 2810) advanceState(CIRCLE);
      //2510
      break;

    // -------------------------------------------
    case CIRCLE:
      kp = 20.0; ki = 0.0010; kd = 10.0; 
      digitalWrite(GREEN, HIGH);
      digitalWrite(YELLOW, LOW);
      digitalWrite(RED, HIGH);
      if (t - stateStart < 200) diffDrive(0, -180);
      else if (t - stateStart < 1800) {
        
      if (isLineLost(L, M, R)){
        diffDrive(255, 150);}
        //   gapCounter ++;
        //   if (gapCounter > 100){
        //   if (t - stateStart > 1650)diffDrive(255, 120);
        //   else diffDrive(0, -180);  
        //   if (t - stateStart > 1750) advanceState(DONE);
        //   }

        // } 
        else{
        diffDrive(255, (turn * 1.1));
        //gapCounter = 0; 
        }

      }
      else diffDrive(0, -180);   
      if (t - stateStart > 1980) advanceState(DONE);
      break;

    // -------------------------------------------
    case DONE:
      digitalWrite(GREEN, LOW);
      digitalWrite(YELLOW, HIGH);
      digitalWrite(RED, HIGH);
      kp = 5.0; ki = 0.0; kd = 1.0;
      if (!isLineLost(L, M, R)) diffDrive(255, turn);
      else diffDrive(255, 0);
      break;
  }
  delayMicroseconds(100);
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

  if (turn > 40) turn = turn * 1.2;

  prevError = error;
  lastPID = now;

  return constrain(turn, -200, 200);
}

int getSmartError(int L, int M, int R) {
    static int directionBias = 0;
   
    int error = (L - R) *1.2;

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
  return (L < 60 && M < 60 && R < 60);  // tune threshold if needed
}

void advanceState(State next) {
    integral = 0;
    state = next;
    stateStart = timer.read();   // reset state timestamp

}




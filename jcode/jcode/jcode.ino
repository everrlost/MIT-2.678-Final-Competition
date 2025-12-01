const int PWMA=9; // Pololu drive A
const int AIN2=8;
const int AIN1 =7;
const int STDBY=6;
const int BIN1 =5; // Pololu drive B
const int BIN2 =4;
const int PWMB =3;

const int CALLED = 10; // calibration LED
const int CALBUT = 2; // calibration button

#define RSENSOR A0
#define MSENSOR A1
#define LSENSOR A2

#define CALIBNO 0
#define CALIBBLACK 1
#define CALIBHALF 2
#define CALIBWHITE 3
#define CALIBDONE 4

volatile int calibMode = CALIBDONE;
int calibSamples = 0;
unsigned long lBlack = 985;
unsigned long mBlack = 982;
unsigned long rBlack = 981; 
unsigned long lWhite = 553;
unsigned long mWhite = 435;
unsigned long rWhite = 457;
int leftSense;
int midSense;
int rightSense;

#define NSTATES 4

#define SDONE -1
#define SLEFT 0
#define SRIGHT 1
#define SCROSSING 2

typedef int state;

void rightGuid();
void leftGuid();
void crossingGuid();

int nullChk();
int crossingEntry();
int crossingExit();


void (*guidanceFxs[NSTATES])() = {leftGuid, rightGuid, crossingGuid};
int (*entryChkFxs[NSTATES])() = {nullChk, nullChk, crossingEntry};
int (*exitChkFxs[NSTATES])() = {nullChk, nullChk, crossingExit};

const state trackDefn[] = {SRIGHT, SCROSSING, SLEFT, SCROSSING, SDONE};

state *curState = &trackDefn[0];
state *nextState = &trackDefn[1];
uint16_t startTime;

float error[2] = {0,0};
float integral = 0;
long time;   
int offset = 0; 
int mode = 3;
void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH);
  Serial.begin(9600);
  pinMode(CALBUT, INPUT_PULLUP);
  pinMode(CALLED, OUTPUT);
  // attachInterrupt(digitalPinToInterrupt(2), calibrationPress, RISING);
  drive(0,0);
  Serial.println("Welcome to hell");
  startTime = millis(); 
  drive(-5,255);
  delay(500);
}
int targetSpeed = 200;
int death = 0;
int lineRecover = 0;
float guidP = 0.45;
float guidD = 2; 
float guidI = 0.0005;
int thinLine;
int lockedout = 0;
int gap = 0;
long secondgap = 0;
int overrideR = 0;
int bias = 0;
int factor = 2;
long start1;
long start2 = 99999999;
void loop() {
  leftSense = adjustSensor(analogRead(LSENSOR), lWhite, lBlack);
  midSense = adjustSensor(analogRead(MSENSOR), mWhite, mBlack);
  rightSense = adjustSensor(analogRead(RSENSOR), rWhite, rBlack);
  if (lineRecover > 100) {
    drive(-255,255);
  } else if (lineRecover) {
    drive(128,255);
    if (lineRecover == 1) {
      drive(0,0);
      integral = 0;
      error[0] = 0;
      error[1] = 0;
      guidP = 0.7;
      lockedout = 1;
    }
  } else 
  if (mode == 1){
    leftGuid();
  }
  else if (mode == 2){
    straightGuid();
  }
  else if (mode == 3) {
    rightGuid();
  } else if (mode == 4){
    drive(-3,255);
    gap--;
    if (gap <= 0) {
      
      guidP = 0.85;guidD=0.45;mode = 5;}
    start1 = millis(); 
  }
  else if (mode == 5){
    rightGuid();
    /*if (millis()>start1+6500) {
      guidP = 2.5;
      overrideR = 1;
    } else if (millis()>start1+2000) {
      guidP = 0.5;
      overrideR = 0;
      integral = 0;
    }
    else */if (millis()>start1+1700) {
      mode = 7;
      overrideR = 2400;
    } else if (millis()>start1+1100) {
      guidP = 0.3;
    } else  if (millis()>start2+5000) {
      guidP = 0.7;
      //if (leftSense > 100 && midSense > 100 && rightSense > 100) mode = 6;
    } else if (millis()>start2+2900){ //infinite loop
      guidP = 0.45;
      guidD = 0.9; 
    }
    else if(millis() > start2+2100){
      //guidP = 3.5;
      drive(-70,255);

    }
    //if(millis()> start1 + 5000){
    //  mode = 6;
    //}

  } else if (mode == 6) {

    motorWrite(0, AIN1, AIN2, PWMA);
    motorWrite(0, BIN1, BIN2, PWMB); 
  } else if (mode == 7) {
    
    drive(-255,255);
    overrideR--;
    if (overrideR <= 0) {
      mode = 5;
      start1 = 999999999;
      start2 = millis()-200;
      guidP = 0.5;
    }

  }
  if (rightSense > (leftSense) && !lockedout && lineRecover == 0) {
    death++;
  } else {
    death -= 10;
    if (death < 0 && death > -1000) death = 0;
  }
  if (mode < 4) {
    if (lockedout && error[1] < 8) {
      gap++;
    } else if (lockedout && gap > 0) {
      gap -= 2;
    }
    if (gap > 350) {
      if (secondgap == 0) {
      secondgap = millis();
      gap = 0;
      } else if (millis() - secondgap > 1500) {
        mode = 4;
        gap = 1300;
        secondgap = 99999;
      }
    }
    if (gap > 100 && millis() - secondgap > 1500 && secondgap != 0) {
      guidP = 0.17;
      integral = 0;
    }
  }
  if (millis() > 7300 && millis() < 7400) {
    guidP = 0.18;
    factor = 7;
  }

  if (millis() > 9200 && millis () < 9300) {
    factor = 2;
    guidP = 0.55;
  }
  if (death > 40 && (millis() > 9200 || millis() < 2000)) {
    //if (revive == 0) {
    //  revive = millis();
    //} else if (millis() - revive > 500) {
      lineRecover = 1300;
    //}
  } else if (lineRecover > 0) {
    lineRecover--;
  }
  digitalWrite(CALLED, lineRecover ? HIGH : overrideR);
  delayMicroseconds(100);

}
void motorWrite(int spd, int pin_IN1 , int pin_IN2 , int pin_PWM) {
    if (spd < 0) {
        digitalWrite(pin_IN1, HIGH); // go one way
        digitalWrite(pin_IN2, LOW);
    } else {
        digitalWrite(pin_IN1, LOW); // go the other way
        digitalWrite(pin_IN2, HIGH);
    }
    analogWrite(pin_PWM, abs(spd));
}
void drive(int speed, int amp) {
    if (speed > 0){
      motorWrite(amp, AIN1, AIN2, PWMA);
      motorWrite(amp-speed, BIN1, BIN2, PWMB);   
    }
    else{
      motorWrite(amp+speed, AIN1, AIN2, PWMA);
      motorWrite(amp, BIN1, BIN2, PWMB);   
    }

}
void calibrationPress() {
  if (calibMode == CALIBHALF) {
    calibMode = CALIBWHITE;
  } else {
    calibMode = CALIBBLACK;
  }
  calibSamples = 0;
}
int adjustSensor(int measured, int whiteCal, int blackCal) {
  int lowCutoff = (3*whiteCal + blackCal) / 4;
  int hiCutoff = (3*blackCal + whiteCal) / 4;
  if (measured < lowCutoff) return 0;
  if (measured > hiCutoff) return 255;
  return map(measured, lowCutoff, hiCutoff, 0, 255);
}

void straightGuid(){
  float p = 2;
  float d = 4; 
  error[1] = leftSense-rightSense;
  float proportion = error[1];
  float derivative = error[1] - error[0];
  offset = constrain(proportion*p + derivative*d,-255,255);
  integral += offset; 
  // Serial.println(integral);
  error[0] = error[1];
  //drive(offset,255);
}

void leftGuid() {
  float p = 0.3;
  float d = 5;
  float i = 0.1 ;
  error[1] = midSense + leftSense - 255 + 3*rightSense;
  float proportion = error[1];
  float derivative = error[1] - error[0];
  offset = constrain(proportion*p + derivative*d + integral*i,-255,255);
  integral += offset; 
  // Serial.println(integral);
  error[0] = error[1];
  //drive(offset,255);
  // Serial.print(leftSense);
  // Serial.print(",");
  // Serial.println(rightSense);

}
void rightGuid() {
  float d = 1;
  float i = 0.0008;

  if (!lockedout) {
    error[1] = midSense + rightSense - 255 + factor*leftSense;
  }/* else if (mode >= 5){
    error[1] = midSense + leftSense - 255 + 4*rightSense;
  } else {
    error[1] = leftSense + rightSense - 255 + 4*midSense;
  }*/
   else {
    error[1] = midSense + leftSense - 255 + 4*rightSense;
   }
  float proportion = error[1];
  float derivative = error[1] - error[0];
  offset = constrain(proportion*guidP + derivative*guidD + integral*guidI + bias,-255,255);
  integral += error[1]; 
  integral = constrain(integral,-60000,60000);
  // Serial.println(integral);
  error[0] = error[1];
  drive(-offset,255);

}

void crossingGuid() {
  //
}
int nullChk() {
  return 0;
}
int crossingEntry() {
  return 0;
}
int crossingExit() {
  return 0;
}

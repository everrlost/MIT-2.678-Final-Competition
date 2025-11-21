// Kate Carpenter and Ian Frankel
// 2.678 Fall 2025

// code from sample code
// Pololu #713 motor driver pin assignments
const int PWMA = 11; // Pololu drive A / left
const int AIN2 = 10;
const int AIN1 = 9;
const int STDBY = 8;
const int BIN1 = 7;  // Pololu drive B / right
const int BIN2 = 6;
const int PWMB = 5;

void setup()
{
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH);
}

void loop()
{
  drive(100, 100); // drive forwards
}

void motorWrite(int spd, int pin_IN1, int pin_IN2, int pin_PWM)
{
  if (spd < 0)
  {
    digitalWrite(pin_IN1, HIGH); // go one way
    digitalWrite(pin_IN2, LOW);
  }
  else
  {
    digitalWrite(pin_IN1, LOW);  // go the other way
    digitalWrite(pin_IN2, HIGH);
  }

  analogWrite(pin_PWM, abs(spd)); // PWM duty cycle sets speed (0–255)
}

// my drive() function
// notes from assignment: 
// 1) be type void, named drive(), and use int speedL (1st) and int speedR (2nd) as parameters
// 2) pass the params to spd of motorWrite(spd, ...)
// 3) set collabs at top
void drive(int speedL, int speedR)
{
  motorWrite(speedL, AIN1, AIN2, PWMA);
  motorWrite(speedR, BIN1, BIN2, PWMB);
}


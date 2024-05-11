#include <Wire.h>
#include <VL6180X.h>
#include "Adafruit_VL53L0X.h"
 
//////////////////////////////////////////////////////////////////////////////////////////////////////////
 
// Sensor Array Setup
 
#define GPIO1 A6
#define GPIO2 A7
#define GPIO3 A0
#define GPIO4 A1
#define GPIO5 A2
 
#define tofAddress1 20
#define tofAddress2 22
#define tofAddress3 24
#define tofAddress4 26
#define tofAddress5 28
 
Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
VL6180X tof2;
VL6180X tof3;
Adafruit_VL53L0X tof4 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof5 = Adafruit_VL53L0X();
 
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
// Motor Driver Setup
 
#define STBY_PIN 11
#define PWMA_PIN 10
#define PWMB_PIN 12
#define BIN1_PIN 8
#define BIN2_PIN 9
#define AIN1_PIN 4
#define AIN2_PIN 5
 
int motorSpeedA = 31;
int motorSpeedB = 30;
int DriftContr = 0; 
bool MotorMove = false;
bool JustTurnedRight = false;
bool MovStraight = false;
bool Turned = false;
bool BackedUp = false;
unsigned long startTime;
unsigned long duration;


//Encoders/////////////////////////////////////
int encoderPin1 = 2; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with intreput pin of arduino.
int encoderPin3 = 6; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin4 = 7; //Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncodedA = 0; // Here updated value of encoder store.
volatile int lastEncodedB = 0; // Here updated value of encoder store.
volatile long encoderValueA = 0; // Raw en
volatile long encoderValueB = 0; // Raw en
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position record/////////////////////////////////////////////////////////////////////////////////////// 
int XPosition = 0;
int YPosition = 0;
int XtoY = 0;
int oldEncoderVal = 0;







//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device Setups
 
void setup()
{
  Serial.begin(115200);
  Wire.begin();
 //encoders////////////////////////////////////////////
  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(encoderPin3, INPUT_PULLUP); 
  pinMode(encoderPin4, INPUT_PULLUP);
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin3, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin4, HIGH); //turn pullup resistor on
  attachInterrupt(2, updateEncoderA, CHANGE); 
  attachInterrupt(3, updateEncoderA, CHANGE);
  attachInterrupt(6, updateEncoderB, CHANGE); 
  attachInterrupt(7, updateEncoderB, CHANGE);
///////////////////////////////////////////////////////
  pinMode(GPIO1, OUTPUT);
  pinMode(GPIO2, OUTPUT);
  pinMode(GPIO3, OUTPUT);
  pinMode(GPIO4, OUTPUT);
  pinMode(GPIO5, OUTPUT);
  delay(50);
 
  digitalWrite(GPIO1, LOW);
  digitalWrite(GPIO2, LOW);
  digitalWrite(GPIO3, LOW);
  digitalWrite(GPIO4, LOW);
  digitalWrite(GPIO5, LOW);
  delay(50);
 
  digitalWrite(GPIO1, HIGH);
  delay(50);
  tof1.begin(tofAddress1);
  tof1.startRangeContinuous();
  delay(100);
 
  digitalWrite(GPIO2, HIGH);
  delay(50);
  tof2.init();
  tof2.configureDefault();
  tof2.setAddress(tofAddress2);
  tof2.setTimeout(0);
  tof2.stopContinuous();
  delay(50);
  tof2.writeReg(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET, 4);
  tof2.startRangeContinuous(100);
  delay(100);
 
  digitalWrite(GPIO3, HIGH);
  delay(50);
  tof3.init();
  tof3.configureDefault();
  tof3.setAddress(tofAddress3);
  tof3.setTimeout(0);
  tof3.stopContinuous();
  delay(50);
  tof3.writeReg(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET, 2);
  tof3.startRangeContinuous(100);
  delay(100);
 
  digitalWrite(GPIO4, HIGH);
  delay(50);
  tof4.begin(tofAddress4);
  tof4.startRangeContinuous();
  delay(100);
 
  digitalWrite(GPIO5, HIGH);
  delay(50);
  tof5.begin(tofAddress5);
  tof5.startRangeContinuous();
  delay(100);
 
  pinMode(STBY_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
 
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
// Main Code Loop
 
void loop()
{
  Serial.println(encoderValueA);
  Serial.println(encoderValueB);
  //Go straight when front path open and sides are blocked.////////////////////////////////////////////////////////////////////////////
  if(tof1.readRange()>85 && tof2.readRangeContinuousMillimeters()<100 && tof3.readRangeContinuousMillimeters()<100) 
  {
    if(MotorMove==false && Turned == false)
    {
      digitalWrite(AIN1_PIN, HIGH);
      digitalWrite(AIN2_PIN, LOW);
      digitalWrite(BIN1_PIN, HIGH);
      digitalWrite(BIN2_PIN, LOW);
      forward();
    }
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //turn right if front and left side are blocked./////////////////////////////////////////////////////////////////////////////////////
  else if (tof1.readRange()<85 && tof2.readRangeContinuousMillimeters()>100 && tof3.readRangeContinuousMillimeters()<100)
  {
    if(MotorMove==false && Turned == false)
    {
      digitalWrite(AIN1_PIN, LOW);
      digitalWrite(AIN2_PIN, HIGH);
      digitalWrite(BIN1_PIN, HIGH);
      digitalWrite(BIN2_PIN, LOW);
      Turned = true;
      turnRight();
    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //turn left if front and right are blocked./////////////////////////////////////////////////////////////////////////////////////////
  else if (tof1.readRange()<85 && tof2.readRangeContinuousMillimeters()<100 && tof3.readRangeContinuousMillimeters()>100)
  {
    if(MotorMove==false && Turned == false)
    {
      digitalWrite(AIN1_PIN, HIGH);
      digitalWrite(AIN2_PIN, LOW);
      digitalWrite(BIN1_PIN, LOW);
      digitalWrite(BIN2_PIN, HIGH);
      Turned = true;
      turnLeft();
    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //go straight if you just turned right and turn otherwise.//////////////////////////////////////////////////////////////////////////
  else if (tof1.readRange()>85 && tof2.readRangeContinuousMillimeters()>100)
  {
    if(MotorMove==false && JustTurnedRight == true && Turned == false)
    {
      digitalWrite(AIN1_PIN, HIGH);
      digitalWrite(AIN2_PIN, LOW);
      digitalWrite(BIN1_PIN, HIGH);
      digitalWrite(BIN2_PIN, LOW);
      forward();
    }
    else if(MotorMove==false && JustTurnedRight == false && Turned == false)
    {
      digitalWrite(AIN1_PIN, LOW);
      digitalWrite(AIN2_PIN, HIGH);
      digitalWrite(BIN1_PIN, HIGH);
      digitalWrite(BIN2_PIN, LOW);
      if(tof3.readRangeContinuousMillimeters()<100)
      {
        Turned = true;
      }
      else
      {
        Turned = false;
      }
      turnRight();
    }
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //Turn around////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if (tof1.readRange()<85 && tof2.readRangeContinuousMillimeters()<100 && tof3.readRangeContinuousMillimeters()<100)
  {
    if(MotorMove==false && Turned == false)
    {
      if(tof2.readRangeContinuousMillimeters()<tof3.readRangeContinuousMillimeters())
      {
      digitalWrite(AIN1_PIN, HIGH);
      digitalWrite(AIN2_PIN, LOW);
      digitalWrite(BIN1_PIN, LOW);
      digitalWrite(BIN2_PIN, HIGH);
      }
      else if(tof2.readRangeContinuousMillimeters()>tof3.readRangeContinuousMillimeters())
      {
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, HIGH);
        digitalWrite(BIN1_PIN, HIGH);
        digitalWrite(BIN2_PIN, LOW);
      }
      turnAround();
    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //Go forward when only front and left are open//////////////////////////////////////////////////////////////////////////////////////
  else if (tof1.readRange()>85 && tof2.readRangeContinuousMillimeters()<100 && tof3.readRangeContinuousMillimeters()>100)
  {
    if(MotorMove==false && Turned == false)
    {
      digitalWrite(AIN1_PIN, HIGH);
      digitalWrite(AIN2_PIN, LOW);
      digitalWrite(BIN1_PIN, HIGH);
      digitalWrite(BIN2_PIN, LOW);
      forward();
    }
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //Turn right when front is blocked but sides are open////////////////////////////////////////////////////////////////////////////////
  else if (tof1.readRange()<85 && tof2.readRangeContinuousMillimeters()>100 && tof3.readRangeContinuousMillimeters()>100)
  {
    if(MotorMove==false && Turned == false)
    {
      digitalWrite(AIN1_PIN, LOW);
      digitalWrite(AIN2_PIN, HIGH);
      digitalWrite(BIN1_PIN, HIGH);
      digitalWrite(BIN2_PIN, LOW);
      Turned = false;
      turnRight();
    }
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //Reverse after turning around///////////////////////////////////////////////////////////////////////////////////////////////////////
  if(Turned == true && MotorMove == false)
  {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    reverse();
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //change speed of wheels to stay away from walls/////////////////////////////////////////////////////////////////////////////////////
  if(MotorMove && MovStraight == true)
  {
    if(tof2.readRangeContinuousMillimeters()<20)
    {
      motorSpeedA = 33;
      motorSpeedB = 30;
      analogWrite(PWMA_PIN, motorSpeedA);
      analogWrite(PWMB_PIN, motorSpeedB);
      DriftContr++;
    }
    else if(tof3.readRangeContinuousMillimeters()<25 || (DriftContr > 0 && tof2.readRangeContinuousMillimeters()>20))
    {
      motorSpeedA = 31;
      motorSpeedB = 32;
      analogWrite(PWMA_PIN, motorSpeedA);
      analogWrite(PWMB_PIN, motorSpeedB);
      DriftContr = 0;
    }
    else
    {
      motorSpeedA = 31;
      motorSpeedB = 30;
      analogWrite(PWMA_PIN, motorSpeedA);
      analogWrite(PWMB_PIN, motorSpeedB);
      DriftContr++;
    }
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //Brake when front sensor detects wall less than 6cm away and the robot is moving////////////////////////////////////////////////////
  if(MotorMove && tof1.readRange()<60 && MovStraight == true)
  {
    brake();

  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //Brake when time is hit/////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(MotorMove && millis()-startTime>=duration)
  {
    brake();
  }
}
 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
//Motor Movement Functions
void forward(){
  analogWrite(PWMA_PIN, motorSpeedA);
  analogWrite(PWMB_PIN, motorSpeedB);
  MotorMove = true;
  JustTurnedRight = false;
  MovStraight = true;
  Turned = false;
  startTime = millis();
  oldEncoderVal = encoderValueA;
  if(BackedUp == true)
  {
    duration = 2000;
  }
  else
  {
    duration = 1600;
  }
}
 
void brake(){
  MotorMove = false;
  //MovStraight = false;
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
  delay(1000);
  if(XtoY%2 == 0)
    {
      if(XtoY%4 == 0)
      {
        YPosition += (encoderValueA-oldEncoderVal);
      }
      else
      {
        YPosition += -(encoderValueA-oldEncoderVal);
      }
    }
    else
    {
      if(XtoY%4 == 1)
      {
        XPosition += (encoderValueA-oldEncoderVal);
      }
      else
      {
        XPosition += -(encoderValueA-oldEncoderVal);
      }
    }
}
 
void turnRight(){
      MotorMove = true;
      JustTurnedRight = true;
      MovStraight = false;
      XtoY = XtoY+1;
      analogWrite(PWMA_PIN, motorSpeedA);
      analogWrite(PWMB_PIN, motorSpeedB);
      startTime = millis();
      duration = 630;
}
 
void turnLeft(){
  MotorMove = true;
  JustTurnedRight = false;
  MovStraight = false;
  Turned = true;
  XtoY = XtoY+3;
  analogWrite(PWMA_PIN, motorSpeedA);
  analogWrite(PWMB_PIN, motorSpeedB);
  startTime = millis();
  duration = 630;
}
 
void reverse(){
  MotorMove = true;
  MovStraight = false;
  Turned = false;
  BackedUp = true;
  analogWrite(PWMA_PIN, 40);
  analogWrite(PWMB_PIN, 40);
  startTime = millis();
  DriftContr=0;
  duration = 700;
}
void turnAround()
{
  MotorMove = true;
  JustTurnedRight = false;
  MovStraight = false;
  Turned = true;
  XtoY = XtoY+2;
  analogWrite(PWMA_PIN, motorSpeedA);
  analogWrite(PWMB_PIN, motorSpeedB);
  startTime = millis();
  duration = 1300;
}

//Encoders//////////////////////////////////////////////////////////////////////////////////////
void updateEncoderA(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encodedA = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedA << 2) | encodedA; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueA --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueA ++;

  lastEncodedA = encodedA; //store this value for next time
}
void updateEncoderB(){
  int MSB2 = digitalRead(encoderPin3); //MSB = most significant bit
  int LSB2 = digitalRead(encoderPin4); //LSB = least significant bit

  int encodedB = (MSB2 << 1) |LSB2; //converting the 2 pin value to single number
  int sum  = (lastEncodedB << 2) | encodedB; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueB --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueB ++;

  lastEncodedB = encodedB; //store this value for next time
}
/////////////////////////////////////////////////////////////////////////////////////////////
 
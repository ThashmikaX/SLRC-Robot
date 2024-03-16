#include <QTRSensors.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <motor.h>
#include <ir_module.h>
#include <color.h>

#include <Ultrasonic.h>

#define Buz 22

#define QTR0 0
#define QTR1 1
#define QTR2 2
#define QTR3 3
#define QTR4 4
#define QTR5 5
#define QTR6 6
#define QTR7 7

#define threshold 700

QTRSensors qtr;

const uint8_t SensorCount = 16;
uint16_t sensorValues[SensorCount];

uint16_t sensorThresholds[SensorCount];

float initialPos = 0;
unsigned int line_position = 0;


float Kp = 0.2; 
float Ki = 0; 
float Kd = 2; 
int P;
int I;
int D;

int lastError = 0;

const uint8_t rightMaxSpeed = 220;
const uint8_t leftMaxSpeed = 220;
const uint8_t rightBaseSpeed = 160;
const uint8_t leftBaseSpeed = 130;



void setup()
{
  irSetup();
  forwardDistanceSetup();
 
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12,A13,A14,A15,A0, A1, A2, A3, A4, A5,A6,A7}, SensorCount);
  qtr.setEmitterPin(2);
  //qtr.setEmitterPin(13);

  delay(500);
  pinMode(Buz, OUTPUT);
  digitalWrite(Buz, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  for (uint16_t i = 0; i < 200; i++)
  {
    turnLeft(200);
    qtr.calibrate();
  }
  stopMotor();
  digitalWrite(Buz, LOW); // turn off Arduino's LED to indicate we are through with calibration
  delay(4000);
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {

    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

   //Take threshold values
    Serial.println("Thresholds");

  digitalWrite(Buz, HIGH);
  delay(400);
  digitalWrite(Buz,LOW);
  delay(400);
  digitalWrite(Buz, HIGH);
  delay(400);
  digitalWrite(Buz,LOW);
  delay(400);
  digitalWrite(Buz,HIGH);
  delay(400);
  digitalWrite(Buz, LOW);
  delay(400);
  digitalWrite(Buz,HIGH);
  delay(400);
  digitalWrite(Buz, LOW);
  delay(400);
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {

    sensorThresholds[i] = 700;
    Serial.print(sensorThresholds[i]);
    Serial.print(' ');
  }

  Serial.println();
  delay(1000);

  for (uint8_t i = 0; i < 10; i++){
     initialPos=qtr.readLineWhite(sensorValues);
      delay(100);
  }

  //buz 4 times
  digitalWrite(Buz, HIGH);
  delay(4000);
  digitalWrite(Buz, LOW);
  delay(1000);
 

  //servo
  //setupServo(); 
}

void follow_line() // follow the line
{
 
  int lastError = 0;

  while (1)
  {

    line_position = qtr.readLineWhite(sensorValues);

    int error = line_position - initialPos;
    int error1 = error - lastError;
    int error2 = (2.0 / 3.0) * error2 + error;
    int motorSpeed = Kp * error + Kd * error1 + Ki * error2;
    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;
    if (rightMotorSpeed > rightMaxSpeed)
      rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > leftMaxSpeed)
      leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    
    driveMotor(leftMotorSpeed,rightMotorSpeed);

    lastError = error;

    qtr.readLineWhite(sensorValues);
    if (sensorValues[4] < threshold || sensorValues[11] < threshold)
    {
      driveMotor(leftBaseSpeed,rightBaseSpeed);
      return;
    }
    if (sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold && sensorValues[8] < threshold && sensorValues[9] < threshold&& sensorValues[10] < threshold&& sensorValues[11] < threshold&& sensorValues[12] < threshold)
    {

     driveMotor(leftBaseSpeed,rightBaseSpeed);
      return;
    }
  }
}



void loop()
{
  
    follow_line();
}



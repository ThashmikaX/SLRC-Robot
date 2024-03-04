#include <QTRSensors.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <QTRSensors.h>
#include <QTRSensors.h>

#include <motor.h>
#include <ir_module.h>
#include <color.h>

#define Buz 22

#define QTR0 0
#define QTR1 1
#define QTR2 2
#define QTR3 3
#define QTR4 4
#define QTR5 5
#define QTR6 6
#define QTR7 7



QTRSensors qtr;

const uint8_t SensorCount = 16;
uint16_t sensorValues[SensorCount];

uint16_t sensorThresholds[SensorCount];



float initialPos = 0;


float Kp = 0.03; 
float Ki = 0; 
float Kd = 7; 
int P;
int I;
int D;


int lastError = 0;

const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;


void setup()
{
  irSetup();
  SetColorSensor();
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12,A13,A14,A15,A0, A1, A2, A3, A4, A5,A6,A7}, SensorCount);
  qtr.setEmitterPin(2);
  qtr.setEmitterPin(9);


  delay(500);
  pinMode(Buz, OUTPUT);
  digitalWrite(Buz, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(Buz, LOW); // turn off Arduino's LED to indicate we are through with calibration
  delay(1000);
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
}


void PID_control() {

  uint16_t position = qtr.readLineWhite(sensorValues); 

  // uint16_t position = qtr.readLineWhite(sensorValues); 
  int error = initialPos - position; 

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;
  
  int motorspeeda = basespeeda - motorspeed;
  int motorspeedb = basespeedb + motorspeed;

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  

  // Serial.print(motorspeeda);
  // Serial.print('\t');
  // Serial.print(motorspeedb);
  // Serial.print('\t');
  // Serial.println(position);

  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  //driveMotor(motorspeeda, motorspeedb);
}

uint16_t line_position = 0;

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
    int rightMotorSpeed = basespeeda - motorSpeed;
    int leftMotorSpeed = basespeedb + motorSpeed;
    if (rightMotorSpeed > maxspeeda)
      rightMotorSpeed = maxspeeda; 
    if (leftMotorSpeed > maxspeedb)
      leftMotorSpeed = maxspeedb; 
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    
    driveMotor(rightMotorSpeed, leftMotorSpeed);

    lastError = error;

    qtr.readLineWhite(sensorValues);
    if (sensorValues[0] > sensorThresholds[0] || sensorValues[10] > sensorThresholds[10])
    {
      driveMotor(150, 150);
      return;
    }
    if (sensorValues[0] < sensorThresholds[0] && sensorValues[1] < sensorThresholds[1] && sensorValues[2] < sensorThresholds[2] && sensorValues[3] < sensorThresholds[3] && sensorValues[4] < sensorThresholds[4] && sensorValues[5] < sensorThresholds[5] && sensorValues[6] < sensorThresholds[6]&& sensorValues[7] < sensorThresholds[7]&& sensorValues[8] < sensorThresholds[8]&& sensorValues[9] < sensorThresholds[9]&& sensorValues[10] < sensorThresholds[10])
    {

      driveMotor(150, 150);
      return;
    }
  }
}

void turn(char dir)
{
  switch (dir)
  {
  case 'R':
    
    driveMotor(0, 200);
    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[15] < sensorThresholds[0])
    {
          line_position = qtr.readLineWhite(sensorValues);

    }

    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[12] < sensorThresholds[2] || sensorValues[13] < sensorThresholds[3]) // wait for outer most sensor to find the line
    {
      line_position = qtr.readLineWhite(sensorValues);
    }

    driveMotor(0, 200);
    break;

  // Turn right 90deg
  case 'L':
    driveMotor(200, 0);
    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[0] < sensorThresholds[0]) // wait for outer most sensor to find the line
    {
      line_position = qtr.readLineWhite(sensorValues);
    }
    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[2] < sensorThresholds[2] || sensorValues[3] < sensorThresholds[3]) // wait for outer most sensor to find the line
    {
          line_position = qtr.readLineWhite(sensorValues);

    }

    driveMotor(200, 0);
    
    break;

  // Turn right 180deg to go back
  case 'B':
    

   
    break;

  case 'S':

    break;
  }
}

void loop()
{
  

  Serial.println(GetColors());
  
  

  while (1)
  {
  

     if (GetColors()==2)
     {
      
      stopMotor();

     }else{
      follow_line();
     qtr.readLineWhite(sensorValues);
      if (sensorValues[15] < 700){
      stopMotor();
      driveBackMotor(200,200);
      delay(200);
      //delay(2000);
      digitalWrite(Buz, HIGH);
      turn('R');
      break;
    }else{
      digitalWrite(Buz, LOW);
    }
     }
    
      
  }

  
  

  
}



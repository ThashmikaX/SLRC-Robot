#include <QTRSensors.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <QTRSensors.h>
#include <QTRSensors.h>

#include <motor.h>

#define Buz 15


QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


float initialPos = 0;


float Kp = 0.07; 
float Ki = 0.001; 
float Kd = 4; 
int P;
int I;
int D;


int lastError = 0;

const uint8_t maxspeeda = 210;
const uint8_t maxspeedb = 210;
const uint8_t basespeeda = 130;
const uint8_t basespeedb = 130;


void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A8, A9, A10, A11, A12,A13,A14}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(Buz, OUTPUT);
  digitalWrite(Buz, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 100; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(Buz, LOW); // turn off Arduino's LED to indicate we are through with calibration

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
  Serial.println();
  Serial.println();
  delay(1000);

  for (uint8_t i = 0; i < 10; i++){
     initialPos=qtr.readLineBlack(sensorValues);
      delay(100);
  }
}


void PID_control() {
  uint16_t position = qtr.readLineWhite(sensorValues); 
  int error = initialPos - position; //3500 is the ideal position (the centre)

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position);

  Serial.print(motorspeeda);
  Serial.print('\t');
  Serial.print(motorspeedb);
  Serial.print('\t');
  Serial.println(position);

  
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
  driveMotor(motorspeeda, motorspeedb);
}


void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
//   float err = qtr.readLineBlack(sensorValues);

//   // print the sensor values as numbers from 0 to 1000, where 0 means maximum
//   // reflectance and 1000 means minimum reflectance, followed by the line
//   // position
//   // for (uint8_t i = 0; i < SensorCount; i++)
//   // {
//   //   Serial.print(sensorValues[i]);
//   //   Serial.print('\t');
//   // }
//   //Serial.println(position);

// float diff=(initialPos-err)*kP + (err-prevError)*kD;
//   float leftSpeed = 120-diff;
//   float rightSpeed = 120+diff;

  

//   prevError = err;
//   driveMotor(leftSpeed, rightSpeed);

  // PID_control();

  turnLeft(200);

  delay(4000);
   stopMotor();
  delay(4000);
  turnRight(200);
  delay(4000);
  stopMotor();
  delay(4000);
}



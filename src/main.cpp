#include <QTRSensors.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <motor.h>
#include <color.h>

#define Buz 14


QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

float kP = 0.05;
float kD = 0;

float initialPos = 0;

float prevError = 0;

void setup()
{
  //setup color sensor
  SetColorSensor();

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
  //Serial.println();

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
     initialPos=qtr.readLineWhite(sensorValues);
      delay(100);
  }
}

void loop()
{

  GetColors(); //get color sensor values
  
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  float err = qtr.readLineWhite(sensorValues);

  
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  //Serial.println(position);

float diff=(initialPos-err)*kP;
  float leftSpeed = 120-diff;
  float rightSpeed = 120+diff;

   Serial.println(diff);

  if(leftSpeed<10){
    leftSpeed=10;
  }

  if(rightSpeed<10){
    rightSpeed=10;
  }
  

  prevError = err;
  driveMotor(leftSpeed, rightSpeed);
  
  
}

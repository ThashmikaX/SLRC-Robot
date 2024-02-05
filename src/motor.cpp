#include <Arduino.h>
#include <AFMotor.h>
#include <Wire.h>
#include <SPI.h>
#include <motor.h>

AF_DCMotor left(3);
AF_DCMotor right(4);

void driveMotor(uint8_t leftSpeed, uint8_t rightSpeed)
{
  left.setSpeed(leftSpeed);
  right.setSpeed(rightSpeed);
  left.run(BACKWARD);
  right.run(BACKWARD);

  Serial.print(leftSpeed);
  Serial.print('\t');
  Serial.println(rightSpeed);
}



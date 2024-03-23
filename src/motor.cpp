#include <Arduino.h>
#include <AFMotor.h>
#include <Wire.h>
#include <SPI.h>
#include <motor.h>

AF_DCMotor left(3);
AF_DCMotor right(4);

void driveMotor(uint8_t leftSpeed, uint8_t rightSpeed)
{
  left.setSpeed(rightSpeed);
  right.setSpeed(leftSpeed);
  left.run(BACKWARD);
  right.run(BACKWARD);

  // Serial.print(leftSpeed);
  // Serial.print('\t');
  // Serial.println(rightSpeed);
}

void turnRight(uint8_t speed)
{
  left.setSpeed(speed);
  right.setSpeed(speed);
  left.run(BACKWARD);
  right.run(FORWARD);
}

void turnRightSmooth(uint8_t speed)
{
  left.setSpeed(speed);
  right.setSpeed(0);
  left.run(BACKWARD);
  right.run(FORWARD);
}

void turnLeft(uint8_t speed)
{
  left.setSpeed(speed);
  right.setSpeed(speed);
  left.run(FORWARD);
  right.run(BACKWARD);
}

void turnLeftSmooth(uint8_t speed)
{
  left.setSpeed(0);
  right.setSpeed(speed);
  left.run(FORWARD);
  right.run(BACKWARD);
}

void stopMotor()
{
  left.run(RELEASE);
  right.run(RELEASE);
}

void stopMotorHard()
{
  stopMotor();
  delay(100);
  driveBackMotor(250, 250);
  delay(10);
  stopMotor();

}

void driveBackMotor(uint8_t leftSpeed, uint8_t rightSpeed)
{
  left.setSpeed(rightSpeed);
  right.setSpeed(leftSpeed);
  left.run(FORWARD);
  right.run(FORWARD);

  // Serial.print(leftSpeed);
  // Serial.print('\t');
  // Serial.println(rightSpeed);
}



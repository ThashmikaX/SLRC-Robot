#include <Arduino.h>
#include <AFMotor.h>
#include <Wire.h>
#include <SPI.h>
#include <motor.h>
#include <servo.h>

Servo grip1servo1;
Servo grip1servo2;


AF_DCMotor left(3);
AF_DCMotor right(4);


void grip1(uint8_t val)
{
  grip1servo1.write(val);
  grip1servo2.write(val);
  delay(20);
}

void setupServo()
{
  grip1servo2.attach(9);
  
  delay(100);
  grip1servo1.attach(10);
}



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



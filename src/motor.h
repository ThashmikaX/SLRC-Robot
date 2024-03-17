#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

void driveMotor(uint8_t leftSpeed, uint8_t rightSpeed);
void turnLeft(uint8_t speed);
void turnRightSmooth(uint8_t speed);
void turnLeftSmooth(uint8_t speed);
void turnRight(uint8_t speed);
void stopMotor();
void driveBackMotor(uint8_t leftSpeed, uint8_t rightSpeed);

#endif
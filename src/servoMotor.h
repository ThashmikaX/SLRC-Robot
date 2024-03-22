#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Servo.h>
#include <Arduino.h>

void setupServo();
void gripOpen();
void gripClose();
void servoDetach();

#endif // SERVOMOTOR_H
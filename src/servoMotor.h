#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Servo.h>
#include <Arduino.h>

void setupServo();
void gripOpen();
void gripClose();
void liftUp();
void liftDown();
void armUp();
void armDown();

#endif // SERVOMOTOR_H
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

const int trigPin = 25;
const int echoPin = 27;

const int trigPin2 = 43;
const int echoPin2 = 42;
// defines variables

void forwardDistanceSetup();
int getForwardDistance();
void backwardDistanceSetup();
int getBackwardDistance();

#endif // ULTRASONIC_H


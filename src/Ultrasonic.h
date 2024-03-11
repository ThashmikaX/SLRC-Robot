#ifndef ULTRASONIC_H
#define ULTRASONIC_H

const int trigPin = 9;
const int echoPin = 10;
// defines variables

void forwardDistanceSetup();
int getForwardDistance();

#endif // ULTRASONIC_H


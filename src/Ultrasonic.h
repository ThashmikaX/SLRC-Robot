#ifndef ULTRASONIC_H
#define ULTRASONIC_H

const int trigPin = 25;
const int echoPin = 27;
// defines variables

void forwardDistanceSetup();
int getForwardDistance();

#endif // ULTRASONIC_H


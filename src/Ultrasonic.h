#ifndef ULTRASONIC_H
#define ULTRASONIC_H

const int trigPin = 25;
const int echoPin = 27;

const int trigPin2 = 43;
const int echoPin2 = 42;
// defines variables

const int trigPin3 = 47;
const int echoPin3 = 45;

//47 -trig
//45 -echo

void forwardDistanceSetup();
int getForwardDistance();
void sideDistanceSetup();
int getSideDistance();
void upDistanceSetup();
int getUpDistance();

#endif // ULTRASONIC_H


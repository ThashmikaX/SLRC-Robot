#ifndef OBJECTALGO_H
#define OBJECTALGO_H

#include <Arduino.h>
#include <Ultrasonic.h>

const int trigPin = 9;
const int echoPin = 10;
const int ledPin = 13;

// Define array
const int arraySize = 10; // Adjust size according to your requirement
int distanceArray[arraySize];

#endif
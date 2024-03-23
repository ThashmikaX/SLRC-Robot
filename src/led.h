#ifndef LED_H
#define LED_H

#include <Arduino.h>

const int Blue = 50;
const int Green = 23;

void setupLEDs();
void onBlue();
void onGreen();
void offBlue();
void offGreen();

#endif // LED_H

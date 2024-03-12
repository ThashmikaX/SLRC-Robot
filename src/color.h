#ifndef COLOR_H
#define COLOR_H

#include <Arduino.h>



uint8_t GetColorsFloor();
uint8_t GetColorsForward();
void SetColorSensor();

#define s1_0 24      //Module pins  wiring
#define s1_1 26
#define s1_2 28
#define s1_3 30
#define out1 32

#define s2_0 34      //Module pins  wiring
#define s2_1 35
#define s2_2 36
#define s2_3 37
#define out2 38


#endif // PINS_H


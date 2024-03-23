#ifndef BUZZ_H
#define BUZZ_H

#include <Arduino.h>

#define Buz 22

void buzzSetup();
void buzz(uint8_t duration);
void twoBuzz();

#endif // BUZZ_H
#include <buzz.h>

void buzzSetup() {
  pinMode(Buz, OUTPUT);
}

void buzz(uint8_t duration) {
    digitalWrite(Buz, HIGH);
    delay(duration);
    digitalWrite(Buz, LOW);
}

void twoBuzz() {
    buzz(100);
    delay(100);
    buzz(100);

}
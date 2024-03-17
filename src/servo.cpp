#include <Servo.h>
#include <Arduino.h>

#define LIFT_UP 0
#define LIFT_DOWN 100
#define GRIP_OPEN 0

Servo rightServo;
Servo leftServo;

int pos = 0; //0 - close, 1 - open

void gripOpen()
{
  if (pos != 1)
  {
    for(int i=0; i<=70; i++)
    {
      leftServo.write(90+i);
      rightServo.write(90-i);
      delay(10);
    }
  }
  rightServo.write(20);
  leftServo.write(160);
  delay(10);
  pos = 1;
}

void gripClose()
{
  if (pos != 0)
  {
    for(int i=0; i<=70; i++)
    {
      leftServo.write(160-i);
      rightServo.write(20+i);
      delay(10);
    }
  }
  rightServo.write(90);
  leftServo.write(90);
  delay(10);
  pos = 0;
}

void setupServo()
{
  leftServo.attach(9);
  delay(100);
  rightServo.attach(10);
}

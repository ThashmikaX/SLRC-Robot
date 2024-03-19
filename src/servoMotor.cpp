#include <servoMotor.h>

#define LIFT_UP 0
#define LIFT_DOWN 100
#define GRIP_OPEN 0

Servo rightServo;
Servo leftServo;
Servo liftServo;

uint8_t gripPos = 0; //0 - close, 1 - open
uint8_t liftPos = 0; //0 - down, 1 - up


void setupServo()
{
  leftServo.attach(9);
  delay(100);
  rightServo.attach(10);
  delay(100);
  liftServo.attach(13);
}

void gripOpen()
{
  if (gripPos != 1)
  {
    for(int i=0; i<=60; i++)
    {
      leftServo.write(110-i);
      rightServo.write(70+i);
      delay(10);
    }
  }
  leftServo.write(50);
  rightServo.write(130);
  delay(10);
  gripPos = 1;
}

void gripClose()
{
  if (gripPos != 0)
  {
    for(int i=0; i<=60; i++)
    {
      leftServo.write(50+i);
      rightServo.write(130-i);
      delay(10);
    }
  }
  leftServo.write(110);
  rightServo.write(70);
  
  delay(10);
  gripPos = 0;
}

void liftUp()
{
  if (liftPos != 1)
  {
    for(int i=0; i<=30; i++)
    {
      liftServo.write(80-i);
      delay(5);
    }
  }
  liftServo.write(50);
  liftPos = 1;
}

void liftDown()
{
  if (liftPos != 0)
  {
    for(int i=0; i<=30; i++)
    {
      liftServo.write(50+i);
      delay(50);
    }
  }
  liftServo.write(80);
  delay(10);
  liftPos = 0;
}

void armDown()
{
  for(int i=0; i<=80; i++)
  {
    liftServo.write(i);
    delay(50);
  }
}

void armUp()
{
  for(int i=0; i<=80; i++)
  {
    liftServo.write(80-i);
    delay(50);
  }
}



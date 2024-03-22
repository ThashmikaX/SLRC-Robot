#include <QTRSensors.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <motor.h>
#include <ir_module.h>
#include <color.h>

#include <Ultrasonic.h>
#include <led.h>

#define Buz 22

#define QTR0 0
#define QTR1 1
#define QTR2 2
#define QTR3 3
#define QTR4 4
#define QTR5 5
#define QTR6 6
#define QTR7 7

#define BUTTON_PIN 2
#define CALIBRATE_BUTTON_PIN 18

#define threshold 700

#define CYLINDER 0
#define SQUARE 1


volatile byte ledState = LOW;

QTRSensors qtr;

const uint8_t SensorCount = 16;
uint16_t sensorValues[SensorCount];

uint16_t sensorThresholds[SensorCount];

float initialPos = 0;
unsigned int line_position = 0;


float Kp = 0.2; 
float Ki = 0; 
float Kd = 2; 
int P;
int I;
int D;

int lastError = 0;

const uint8_t rightMaxSpeed = 200;
const uint8_t leftMaxSpeed = 200;
const uint8_t rightBaseSpeed = 150;
const uint8_t leftBaseSpeed = 150;

uint8_t forwardColor = 10;
uint8_t floorColor = 0;
uint8_t shape = CYLINDER; //0 - cylinder, 1 - cube

bool startPos=false;
bool isc = true;


void onStartClick(){
  startPos=true;
  
}

void onCalibrateClick(){
  isc=true;

}

void setup()
{
  // irSetup();
   forwardDistanceSetup();
   SetColorSensor();
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7,A8, A9, A10, A11, A12,A13,A14,A15}, SensorCount);

  //qtr.setEmitterPin(2);
 

  delay(500);
  pinMode(Buz, OUTPUT);
  digitalWrite(Buz, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  turnLeft(200);
  for (uint16_t i = 0; i < 200; i++)
  {
    //turnLeft(200);
    qtr.calibrate();
  }
  stopMotor();
  digitalWrite(Buz, LOW); // turn off Arduino's LED to indicate we are through with calibration
  delay(4000);
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {

    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

   //Take threshold values
    Serial.println("Thresholds");

  digitalWrite(Buz, HIGH);
  delay(400);
  digitalWrite(Buz,LOW);
  delay(400);
  digitalWrite(Buz, HIGH);
  delay(400);
  digitalWrite(Buz,LOW);
  delay(400);
  digitalWrite(Buz,HIGH);
  delay(400);
  digitalWrite(Buz, LOW);
  delay(400);
  digitalWrite(Buz,HIGH);
  delay(400);
  digitalWrite(Buz, LOW);
  delay(400);
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {

    sensorThresholds[i] = 700;
    Serial.print(sensorThresholds[i]);
    Serial.print(' ');
  }

  Serial.println();
  delay(1000);

  for (uint8_t i = 0; i < 10; i++){
     initialPos=qtr.readLineWhite(sensorValues);
      delay(100);
  }

  //buz 4 times
  digitalWrite(Buz, HIGH);
  delay(4000);
  digitalWrite(Buz, LOW);
  delay(1000);
 
  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onStartClick, RISING);
  attachInterrupt(digitalPinToInterrupt(CALIBRATE_BUTTON_PIN), onCalibrateClick, RISING);
  delay(500);

  //servo
  //setupServo(); 
}


void turn(char dir)
{
  switch (dir)
  {
  case 'L':
    turnLeftSmooth(180);
    line_position = qtr.readLineWhite(sensorValues);

    //intially it was 0. can generate turn erros
    while (sensorValues[3] > threshold)
    {
      line_position = qtr.readLineWhite(sensorValues);
    }

    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[6] > threshold || sensorValues[7] > threshold) // wait for outer most sensor to find the line
    {
      line_position = qtr.readLineWhite(sensorValues);
    }

    //warn: change to ready motor for forward motion

    stopMotor();
    break;

  // Turn right 90deg
  case 'R':
    
    turnRightSmooth(200);
    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[12] > threshold) // wait for outer most sensor to find the line
    {
      line_position = qtr.readLineWhite(sensorValues);
    }
    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[9] > threshold || sensorValues[10] > threshold) // wait for outer most sensor to find the line
    {
      line_position = qtr.readLineWhite(sensorValues);
    }

    stopMotor();
    break;

  // // Turn right 180deg to go back
  case 'B':

     turnRight(180);
    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[12] > threshold) // wait for outer most sensor to find the line
    {
      line_position = qtr.readLineWhite(sensorValues);
    }
    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[9] > threshold || sensorValues[10] > threshold) // wait for outer most sensor to find the line
    {
      line_position = qtr.readLineWhite(sensorValues);
    }
    stopMotor();
    turnLeft(200);
    delay(100);
    stopMotor();
    break;

  case 'D':
    
    turnRight(180);
    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[12] > threshold) // wait for outer most sensor to find the line
    {
      line_position = qtr.readLineWhite(sensorValues);
    }
    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[9] > threshold || sensorValues[10] > threshold) // wait for outer most sensor to find the line
    {
      line_position = qtr.readLineWhite(sensorValues);
    }

    stopMotor();
    break;
    

  case 'W':
    turnLeft(180);
    line_position = qtr.readLineWhite(sensorValues);

    //intially it was 0. can generate turn erros
    while (sensorValues[3] > threshold)
    {
      line_position = qtr.readLineWhite(sensorValues);
    }

    line_position = qtr.readLineWhite(sensorValues);

    while (sensorValues[6] > threshold || sensorValues[7] > threshold) // wait for outer most sensor to find the line
    {
      line_position = qtr.readLineWhite(sensorValues);
    }

    //warn: change to ready motor for forward motion

    stopMotor();
    break;

  case 'S':

    break;
   }
}


void QTR_TEST() {

  uint16_t position = qtr.readLineWhite(sensorValues); 

  // uint16_t position = qtr.readLineWhite(sensorValues); 
  int error = initialPos - position; 


 


  

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
   Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" Error: ");
  Serial.println(error);

 
}


void follow_line() // follow the line
{
 
  int lastError = 0;
  while (1)
  {

    line_position = qtr.readLineWhite(sensorValues);

    int error = line_position - initialPos;
    int error1 = error - lastError;
    int error2 = (2.0 / 3.0) * error2 + error;
    int motorSpeed = Kp * error + Kd * error1 + Ki * error2;
    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;
    if (rightMotorSpeed > rightMaxSpeed)
      rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > leftMaxSpeed)
      leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    
    driveMotor(leftMotorSpeed,rightMotorSpeed);

    lastError = error;

    qtr.readLineWhite(sensorValues);
    if (sensorValues[0] < threshold || sensorValues[15] < threshold)
    {
      driveMotor(leftBaseSpeed,rightBaseSpeed);
      return;
    }
    if (sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold && sensorValues[8] < threshold && sensorValues[9] < threshold&& sensorValues[10] < threshold&& sensorValues[11] < threshold&& sensorValues[12] < threshold)
    {

     driveMotor(leftBaseSpeed,rightBaseSpeed);
      return;
    }

    

   
  }
}



void follow_line_back() // follow the line
{
 
  int lastError = 0;
  while (1)
  {

    line_position = qtr.readLineWhite(sensorValues);
    
    //reverse sensor values array
    

    int error = line_position - initialPos;
    int error1 = error - lastError;
    int error2 = (2.0 / 3.0) * error2 + error;
    int motorSpeed = Kp * error + Kd * error1 + Ki * error2;
    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;
    if (rightMotorSpeed > rightMaxSpeed)
      rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > leftMaxSpeed)
      leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    
    driveBackMotor(leftMotorSpeed,rightMotorSpeed);

    lastError = error;

    qtr.readLineWhite(sensorValues);
    if (sensorValues[0] < threshold || sensorValues[15] < threshold)
    {
      driveBackMotor(leftBaseSpeed,rightBaseSpeed);
      return;
    }
    if (sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold && sensorValues[8] < threshold && sensorValues[9] < threshold&& sensorValues[10] < threshold&& sensorValues[11] < threshold&& sensorValues[12] < threshold)
    {

     driveBackMotor(leftBaseSpeed,rightBaseSpeed);
      return;
    }

    

   
  }
}


void follow_line_with_ds() // follow the line
{
 
  int lastError = 0;
  while (1)
  {

    line_position = qtr.readLineWhite(sensorValues);

    int error = line_position - initialPos;
    int error1 = error - lastError;
    int error2 = (2.0 / 3.0) * error2 + error;
    int motorSpeed = Kp * error + Kd * error1 + Ki * error2;
    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;
    if (rightMotorSpeed > rightMaxSpeed)
      rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > leftMaxSpeed)
      leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    
    driveMotor(leftMotorSpeed,rightMotorSpeed);

    lastError = error;

    qtr.readLineWhite(sensorValues);
    uint8_t fd =getForwardDistance();
    if(fd<10 && fd>5){
      return;
    }

    

   
  }
}


void follow_line_with_floor_color() // follow the line
{
 
  int lastError = 0;
  while (1)
  {

    line_position = qtr.readLineWhite(sensorValues);

    int error = line_position - initialPos;
    int error1 = error - lastError;
    int error2 = (2.0 / 3.0) * error2 + error;
    int motorSpeed = Kp * error + Kd * error1 + Ki * error2;
    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;
    if (rightMotorSpeed > rightMaxSpeed)
      rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > leftMaxSpeed)
      leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    
    driveMotor(leftMotorSpeed,rightMotorSpeed);

    lastError = error;

    qtr.readLineWhite(sensorValues);
    
      floorColor = GetColorsFloor();
    if(floorColor==0 || floorColor==1 || floorColor==2 || floorColor==3){
      return;
    }

    

   
  }
}






bool s1=false;
bool s2=false;
bool s3=false;
bool s4 =false;
bool s5 =false;
bool s6 =false;
bool s7 =false;
bool s8 =false;
bool s9 =false;
bool s10 =false;
bool s11 =false;
bool s12 =false;
bool s13 =false;
bool s14 =false;
bool s15 =false;
bool s16 =false;

void rightWay(){
  
      //stopMotorHard();
      delay(1000);
      s9=true;
      while (s9)
      {
        follow_line();
        if(sensorValues[4]<threshold){
              stopMotorHard();
              delay(1000);
              turn('L');
              s9=false;
              s10=true;
            }
      }

      delay(1000);

      while (s10)
      {
        follow_line();
        if(sensorValues[3]<threshold){
              stopMotorHard();
              delay(1000);
              turn('L');
              s10=false;
              s11=true;
            }
      }

      delay(1000);

      //run in circle
      while (s11)
      {
        follow_line();
        if(sensorValues[12]<threshold){
              stopMotorHard();

              if (isc)
              {
                //turn on green
                onGreen();
              }else{
                //turn on blue
                onBlue();
              }
              

              delay(1000);
              turn('R');
              s11=false;
              s12=true;
            }
      }
      

      delay(25000);
}

void leftWay(){
  
      //stopMotorHard();
      delay(1000);
      s9=true;
      while (s9)
      {
        follow_line();
        if(sensorValues[15]<threshold){
              stopMotorHard();
              delay(1000);
              turn('R');
              s9=false;
              s10=true;
            }
      }

      delay(1000);

      while (s10)
      {
        follow_line();
        if(sensorValues[0]<threshold){
              stopMotorHard();
              delay(1000);
              turn('L');
              s10=false;
              s11=true;
            }
      }

      delay(1000);

      while (s11)
      {
        follow_line();
        if(sensorValues[0]<threshold){
              stopMotorHard();
              delay(1000);
              turn('L');
              s11=false;
              s12=true;
            }
      }
      
      //start going away from circle
      delay(1000);
      while (s12)
      {
        follow_line();
        if(sensorValues[0]<threshold){
              stopMotorHard();
              delay(1000);
              turn('L');
              s12=false;
              s13=true;
            }
      }

      delay(1000);

      //stop at color wheel
      while(s13){
        follow_line();
        if(sensorValues[15]<threshold){
          stopMotorHard();
          floorColor=GetColorsFloor();  
          delay(1000);
          s13=false;
          s14=true;
        }
      }

      delay(1000);

       //drive untill middle sensor finds the line
      driveMotor(leftBaseSpeed,rightBaseSpeed);
      delay(200);
      line_position=qtr.readLineWhite(sensorValues);
      while (sensorValues[0]<threshold)
      {
        line_position=qtr.readLineWhite(sensorValues);
      }
      delay(10);
      stopMotorHard();
      delay(1000);

      turn('W');
      delay(1000);

      //go away from color wheen untill junction
      while (s14)
      {
        follow_line();
        if(sensorValues[0]<threshold){
              stopMotorHard();
              delay(1000);
              
              s14=false;
              s15=true;
            }
      }
      
      

      delay(25000);
}

void startRobot(){
  
      

      while (startPos)
      {
        follow_line();
        line_position=qtr.readLineWhite(sensorValues);
        if(sensorValues[0]>threshold){
          startPos=false;
          s1=true;
        }
      }
      
      //start
      while (s1)
      {
          follow_line();
          line_position=qtr.readLineWhite(sensorValues);
          if(sensorValues[4]<threshold){
            delay(200);
            s1=false;
            s2=true;
          }
      }

      //first L junction
      while (s2)
      {
        follow_line_with_ds();
        if(getForwardDistance()<10 && getForwardDistance()>5){
            stopMotorHard();
            delay(200);
            s2=false;
            s3=true;
          }
      }
      
      
      delay(1000);

      //turn back at color wall
      turn('B');
      delay(1000);

      driveBackMotor(leftBaseSpeed,rightBaseSpeed);
      delay(200);
      stopMotor();

      //detect color
      while (s3)
      {
        uint8_t fc=0;
        for (uint8_t i = 0; i < 10; i++)
        {
          fc+=GetColorsForward();
          delay(200);
        }
        
        forwardColor=fc/10;
        s3=false;
        s4=true;
      }


      //turn right at first L
      while (s4)
      {
       follow_line();
        if(sensorValues[12]<threshold){
              stopMotorHard();
              delay(1000);
              turn('R');
              s4=false;
              s5=true;
            }
      }

      delay(1000);

      //turn right at second L
      while (s5)
      {
       follow_line();
        if(sensorValues[12]<threshold){
              stopMotorHard();
              delay(1000);
              turn('R');
              s5=false;
              s6=true;
            }
      }

      delay(1000);

      //turn left at first +
      while (s6)
      {
       follow_line();
        if(sensorValues[0]<threshold){
              stopMotorHard();
              delay(1000);
              turn('L');
              s6=false;
              s7=true;
            }
      }

      delay(1000);


      //stop at color wheel
      while (s7)
      {
      follow_line();

      line_position=qtr.readLineWhite(sensorValues);
      if(sensorValues[4]<threshold){
        stopMotorHard();
        floorColor=GetColorsFloor();  
        delay(1000);
        s7=false;
        s8=true;
      }
      
            
      }

      delay(1000);

      

      //drive untill middle sensor finds the line
      driveMotor(leftBaseSpeed,rightBaseSpeed);
      delay(300);
      line_position=qtr.readLineWhite(sensorValues);
      while (sensorValues[0]<threshold)
      {
        line_position=qtr.readLineWhite(sensorValues);
      }
      delay(10);
      stopMotorHard();
      delay(1000);

      //turn the direction of color
      if(forwardColor==floorColor){
        turn('D');
        rightWay();
      }else{
        turn('W');
        leftWay();
      }


}

void loop()
{
  
  //startRobot();

 
  while (1)
  {
    follow_line_back();
  }
  
  

  if (startPos)
  {
    delay(2000);
    startRobot();
    
  }
  
  

        
}


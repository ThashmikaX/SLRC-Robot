/*
  IR Proximity Sensor interface code
  Turns on an LED on when obstacle is detected, else off.
  blog.circuits4you.com 2016
 */

#include <ir_module.h>

int sensorVal = 0; // variable to store the value coming from the sensor, set to 0 initially
int sensorPin = 0; //select the analog input pin 1 on the Arduino (the output pin from the IR sensor goes to pin A1)


void irSetup() {                
  Serial.begin(9600);           //  setup serial
}

void irIndicate() {
  Serial.print("IR Sensor Readings: "); // print data to the serial port
  sensorVal = analogRead(sensorPin); // read an analog value from sensor and store it
  Serial.println(sensorVal); // print a sensor value to the serial por
}
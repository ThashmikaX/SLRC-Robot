#include <Arduino.h>
#include <Wire.h>
#include <Ultrasonic.h>

long u_duration;
int u_distance;

void forwardDistanceSetup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

int getForwardDistance() {
// Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  u_duration = pulseIn(echoPin, HIGH,5000);
  // Calculating the u_distance
  u_distance = u_duration * 0.034 / 2;
  // Prints the u_distance on the Serial Monitor
//   Serial.print("u_distance: ");
//   Serial.println(u_distance);
  return u_distance;
}
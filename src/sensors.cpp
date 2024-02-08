// #include <sensors.h>

// #include <Arduino.h>
// #include <Wire.h>
// #include <SPI.h>

// #include <QTRSensors.h>

// QTRSensors qtr;

// const uint8_t SensorCount = 8;
// uint16_t sensorValues[SensorCount];
// //float initialPos = 0;

// float startCalibrate(){

//  // configure the sensors
//   qtr.setTypeAnalog();
//   qtr.setSensorPins((const uint8_t[]){A7, A8, A9, A10, A11, A12,A13,A3}, SensorCount);
//   qtr.setEmitterPin(2);

//   delay(500);
//   //pinMode(Buz, OUTPUT);
//   //digitalWrite(Buz, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

//   // analogRead() takes about 0.1 ms on an AVR.
//   // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
//   // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
//   // Call calibrate() 400 times to make calibration take about 10 seconds.
//   for (uint16_t i = 0; i < 100; i++)
//   {
//     qtr.calibrate();
//   }
//   //digitalWrite(Buz, LOW); // turn off Arduino's LED to indicate we are through with calibration

//   // print the calibration minimum values measured when emitters were on
//   Serial.begin(9600);
//   for (uint8_t i = 0; i < SensorCount; i++)
//   {
//     Serial.print(qtr.calibrationOn.minimum[i]);
//     Serial.print(' ');
//   }
//   Serial.println();

//   // print the calibration maximum values measured when emitters were on
//   for (uint8_t i = 0; i < SensorCount; i++)
//   {
//     Serial.print(qtr.calibrationOn.maximum[i]);
//     Serial.print(' ');
//   }
//   Serial.println();
//   Serial.println();
//   delay(1000);

//   for (uint8_t i = 0; i < 10; i++){
//      initialPos=qtr.readLineBlack(sensorValues);
//       delay(100);
//   }

// }

// uint16_t readBlackLineErr(){
//   float err = qtr.readLineBlack(sensorValues);

// }
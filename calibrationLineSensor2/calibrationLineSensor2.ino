#include <QTRSensors.h>
QTRSensors qtr;

#define a1 10 //left motors
#define a2 11

#define b1 6 //right motors 
#define b2 5

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int gripperClosed = 0;

/*
 motion functions
 */
 void goForward(){
  analogWrite(a1, 0);
  analogWrite(a2, 200);
  analogWrite(b1, 220);
  analogWrite(b2, 0);
}

void turnLeft(){
  analogWrite(a1, 220);
  analogWrite(a2, 0);
  analogWrite(b1, 200);
  analogWrite(b2, 0);
}

void arrest(){
  analogWrite(a1, 0);
  analogWrite(a2, 0);
  analogWrite(b1, 0);
  analogWrite(b2, 0);
}

void setup() {
  pinMode (a1, OUTPUT);
  pinMode (a2, OUTPUT);
  pinMode (b1, OUTPUT);
  pinMode (b2, OUTPUT);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
 
  goForward();

  for(uint16_t i = 0; i < 30; i++){
    qtr.calibrate();
  }

 //print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for(uint8_t i = 0; i < SensorCount; i++){
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  //print the calibration maximum values measured when emitters were on
  for(uint8_t i = 0; i < SensorCount; i++){
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

void loop() {
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  uint16_t position = qtr.readLineBlack(sensorValues);

  /* print the sensor values as numbers from 0 to 1000, where 0 means maximum
   reflectance and 1000 means minimum reflectance */
  
  //The the data most on the right is the left sensor, and vice versa
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

if(sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] > 600 && sensorValues[7] > 600 && gripperClosed == 0){
  arrest();
  //close gripper
  //gripperClose();
  gripperClosed = 1;
 
  /*maybe i should use millis, idk*/ delay(200);
  turnLeft();
  delay(300);
  goForward();
  //delay(200);
}
} 

#include <QTRSensors.h>

QTRSensors qtr;

#define a1 10 //left motors
#define a2 11

#define b1 6 //right motors 
#define b2 5

int lineSensor[8];
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{
  pinMode (a1, OUTPUT);
  pinMode (a2, OUTPUT);
  pinMode (b1, OUTPUT);
  pinMode (b2, OUTPUT);
  
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(2);

  Serial.begin(9600);
}

void loop() {
qtr.read(sensorValues);

  // print the sensor values as numbers from 0 to 1023, where 0 means maximum
  // reflectance and 1023 means minimum reflectance
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  /*if( sensorValues[3] > 600 && sensorValues[4] > 600 ){ 
        moveForward();
  }
  else if(sensorValues[0] > 600 || sensorValues[1] > 600 || sensorValues[2] > 600){ // line is on the right
      turnRight();
  }
  else if(sensorValues[5] > 600 || sensorValues[6] > 600 || sensorValues[7] > 600){ // line is on the left
      turnLeft();
  }
  else if(sensorValues[0] < 600 && sensorValues[1] < 600 && sensorValues[2] < 600 && sensorValues[3] < 600 && sensorValues[4] < 600 && sensorValues[5] < 600 && sensorValues[6] < 600 && sensorValues[7] < 600 || sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] > 600 && sensorValues[7] > 600){
      arrest();
  }
  else if (sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] > 600 && sensorValues[7] > 600){
      arrest();
  }*/

  if(sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] > 600 && sensorValues[7] > 600){
      arrest();
  }
  else if(sensorValues[0] < 600 && sensorValues[1] < 600 && sensorValues[2] < 600 && sensorValues[3] < 600 && sensorValues[4] < 600 && sensorValues[5] < 600 && sensorValues[6] < 600 && sensorValues[7] < 600){
      arrest();
  }
  else if( sensorValues[3] > 600 && sensorValues[4] > 600 ){ 
      moveForward();
  }
  else if(sensorValues[0] > 600 || sensorValues[1] > 600 || sensorValues[2] > 600){ // line is on the right
      turnRight();
  }
  else if(sensorValues[5] > 600 || sensorValues[6] > 600 || sensorValues[7] > 600){ // line is on the left
      turnLeft();
  }

}

void moveForward(){
  analogWrite(a1, 0);
  analogWrite(a2, 200);
  analogWrite(b1, 200);
  analogWrite(b2, 0);
}

void turnRight(){
  analogWrite(a1, 0);
  analogWrite(a2, 200);
  analogWrite(b1, 0);
  analogWrite(b2, 200);
}

void turnLeft(){
  analogWrite(a1, 200);
  analogWrite(a2, 0);
  analogWrite(b1, 200);
  analogWrite(b2, 0);
}

void moveBackwords(){
  analogWrite(a1, 0);
  analogWrite(a2, 200);
  analogWrite(b1, 200);
  analogWrite(b2, 0);
}

void arrest(){
  analogWrite(a1, 0);
  analogWrite(a2, 0);
  analogWrite(b1, 0);
  analogWrite(b2, 0);
}

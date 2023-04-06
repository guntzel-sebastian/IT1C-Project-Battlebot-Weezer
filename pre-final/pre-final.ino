

/****************************
*          [ io ]           *
****************************/
////=====[Digital Pins]=====////

  #include <Arduino.h>
  // 0 RX -> HT
  #define bluetoothRX A1
  // 1 TX -> HR
  #define bluetoothTX A0
  // 2 -> R1
  const byte interruptPin1 = 2; // R1, Left rotation sensor
  // 3 -> R2
  const byte interruptPin2 = 3; // R2, Right rotation sensor, pwm pin
  // 4 -> N1
  #define pixelPin 4 // N1 Digital IO pin connected to the NeoPixels
  // 5 -> b2
  #define b2 5 // right motor, pwm
  // 6 -> b1w
  #define b1 6 // right motor, pwm
  // 7 -> trig forward
  const int trigPinForward = 7;
  // 8 -> "Eyes" Servo
  const int echoPinForward = 8;
  // 9 -> Trig left
  const int trigPinLeft = 9; // ultrasonic trigger pin
  // 10 -> a1
  #define a1 10 // left motor, pwm
  // 11 -> a2
  #define a2 11 // left motor, pwm
  // 12 -> echo left
  const int echoPinLeft = 12;
  // 13 -> GR
  #define gripperPin 13

////=====[Analog Pins]=====////

  // 0 <- D1
  // 1 <- D2
  // 2 <- D3
  // 3 <- D4
  // 4 <- D5
  // 5 <- D6
  // 6 <- D7
  // 7 <- D8

/****************************
*****************************
**      [ Libraries ]      **
*****************************
****************************/

////====={Bluetooth}=====////

#include <SoftwareSerial.h>

////=====[Infrared Line Sensor]=====////
#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

////=====[NeoPixels]=====////
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define pixelCount 4 //Number of neopixels 

Adafruit_NeoPixel strip(pixelCount, pixelPin, NEO_RGB
+ NEO_KHZ800);

/****************************
*****************************
**    [ Definitions ]      **
*****************************
****************************/

////=====[Interupts]=====////
int counter1 = 0;
int counter2 = 0;
volatile byte state = LOW;
int setPulse1 = 0;
int setPulse2 = 0;
volatile int previousCount = 0;

////=====[Gripper]=====////

#define gripper_open_pulse 1600
#define gripper_close_pulse  971 // 
#define servoPulseRepeat 10 // number of pulse send to servo

int gripperClosed = 0;
int gripperOpen = 0;

////=====[Ultrasonic Distance Sensor]=====////

int distanceForward = 0;
int distanceLeft = 0; 
int duration = 0;

////=====[Wheel Motors]=====////

boolean turningLeft = false;
boolean forward = false;

////=====[Line Sensor]====/////

int threshold = 750;

////=====[Bluetooth]=====////

const int BUFFER_SIZE = 50;
char buf[BUFFER_SIZE] = { 0 };
boolean checkForMessage = true;

////=====[Serial]=====////

SoftwareSerial mySerial(bluetoothRX, bluetoothTX);


/****************************
**                         **
**        [ SETUP ]        **
**                         **
****************************/

void setup() {

  mySerial.begin(38400);
  
  // infrared line sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2}, SensorCount);

  openGripper();

 //print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for(uint8_t i = 0; i < SensorCount; i++){
//    Serial.print(qtr.calibrationOn.minimum[i]);
//    Serial.print(' ');
  }
  Serial.println();

  //print the calibration maximum values measured when emitters were on
  for(uint8_t i = 0; i < SensorCount; i++){
//    Serial.print(qtr.calibrationOn.maximum[i]);
//    Serial.print(' ');
  }
//  Serial.println();
//  Serial.println();

  //interrupts
  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), counterLeft, CHANGE); //counterLeft
  attachInterrupt(digitalPinToInterrupt(interruptPin2), counterRight, CHANGE); //counterRight

  //motors
  pinMode (a1, OUTPUT);
  pinMode (a2, OUTPUT);
  pinMode (b1, OUTPUT);
  pinMode (b2, OUTPUT);

  //ultrasonic distance sensor
  // The trigger and echo pins for the ultrasonic sensor as output and input, respectively
  pinMode(trigPinForward, OUTPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinForward, INPUT);
  pinMode(echoPinLeft, INPUT);

  //gripper
  pinMode(gripperPin, OUTPUT);

  //neopixels
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'

  while(checkForMessage){
    if(mySerial.available()){ 
      mySerial.readBytesUntil('/n', buf, BUFFER_SIZE);
      if (atoi(buf) == 1337){
        Serial.write("Message Recieved");
        checkForMessage = false;
      }   
    }
  }

  // Calibration
  for (uint16_t i = 0; i < 250; i++)
  {
      qtr.calibrate();

      if (i % 80 == 0)
      {
        moveForward(2, 2);
      }
      else
      {
        halt();
      }
   }
}

void loop() {

  //interval for millis
  static unsigned long interval = 0;


  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  uint16_t position = qtr.readLineBlack(sensorValues);

  /* print the sensor values as numbers from 0 to 1000, where 0 means maximum
   reflectance and 1000 means minimum reflectance */
  
  //The the data most on the right is the left sensor, and vice versa
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
//  Serial.print(position);
  
  if(sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] > threshold && sensorValues[5] > threshold && gripperClosed == 0){
  halt();
  //close gripper
  //gripperClose
  closeGripper();
  gripperClosed = 1;
  moveForward(10,10);
  turnLeft(22, 22);
  halt();
  moveForward(30,30);
  }
  else if( sensorValues[2] > threshold || sensorValues[3] > threshold ){ 
      moveForward(25, 25);
  }
  else if(sensorValues[0] > threshold && sensorValues[1] > threshold){ // line is on the right
      turnRight(1, 1);
  } 
  else if(sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] > threshold && sensorValues[5] > threshold && gripperClosed == 1){
    halt();
    openGripper();
    moveBackward(40,40);
  }
  else if(sensorValues[4] > threshold && sensorValues[5] > threshold){ // line is on the left
      turnLeft(1, 1);
  } 
  else if(sensorValues[0] < threshold && sensorValues[1] < threshold && sensorValues[2] < threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold){
    lookForward();
    lookLeft();
    if(distanceLeft > 24) {
      moveForward(15,15);
      turnLeft(22,22);
      moveForward(20, 20);
    } 
    else if (distanceForward > 10) {
      moveForward(10, 10);
    } 
    else {
      turnRight(15,15);
      moveBackward(5,5);
    }

    if(sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] > threshold && sensorValues[5] > threshold && gripperClosed == 1){
      halt();
      //close gripper
      //gripperClose
      openGripper();
      gripperClosed = 0;
      moveBackward(5, 5);
      halt();
    }
  }

}

/****************************
*****************************
**     [ Functions ]       **
*****************************
****************************/

////=====[NeoPixels]=====////

uint32_t white = strip.Color(255, 255, 255);
uint32_t red = strip.Color(255, 0, 0);
uint32_t amber = strip.Color(255, 75, 0);
uint32_t green = strip.Color(0, 255, 0);

////=====[Interrupts]=====////

void counterLeft(){
  noInterrupts();
  counter1++;
  interrupts();
}

void counterRight(){
  noInterrupts();
  counter2++;
  interrupts();

}

/***********************************
////=====[Distance Reading]=====////
***********************************/

int lookForward() {
  digitalWrite(trigPinForward, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinForward, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinForward, LOW);
  int duration = pulseIn(echoPinForward, HIGH);
  distanceForward = duration / 58.2; // convert duration to distance in cm
  Serial.print("The distance Forward is: ");
  Serial.println(distanceForward);
}

int lookLeft() {
  digitalWrite(trigPinLeft, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft, LOW);
  int duration = pulseIn(echoPinLeft, HIGH);
  distanceLeft = duration / 58.2;
  Serial.print("The distance to the Left is: ");
  Serial.println(distanceLeft);
}

////=====[Servos]=====////
void servo(int pin, int length) {
  for (int i=0; i < servoPulseRepeat; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(length);//in microseconds
    digitalWrite(pin, LOW);
    delay(20); 
  }
}

void openGripper() {
  servo(gripperPin, gripper_open_pulse);
}

void closeGripper(){
  servo(gripperPin, gripper_close_pulse);
}

////======[Movement]=====////

////non-interrupts

void goForward(){
    analogWrite(a1, 0); //lower speed to drive straight
    analogWrite(a2, 200);
    analogWrite(b1, 215);
    analogWrite(b2, 0);
     
    strip.fill(white);
    strip.show();  
}

void halt() {
  //Set motor to stop the car
  analogWrite(a1, LOW);
  analogWrite(a2, LOW);
  analogWrite(b1, LOW);
  analogWrite(b2, LOW);
  strip.fill(red);
  strip.show(); 
}

////interrupts

void moveForward(int pulse1,int pulse2){
  setPulse1 = counter1 + pulse1;
  setPulse2 = counter2 + pulse2;
  while (setPulse1 >= counter1 && setPulse2 >= counter2){
    analogWrite(a1, 0); //left motors, lower speed to drive straight
    analogWrite(a2, 200);
    analogWrite(b1, 217); //right motors
    analogWrite(b2, 0);
    strip.fill(white);
    strip.show();
    Serial.println();
  }
    halt();
}

void turnLeft(int pulse1, int pulse2){
  setPulse1 = counter1 + pulse1;
  setPulse2 = counter2 + pulse2;
  while (setPulse1 >= counter1 && setPulse2 >= counter2){
    analogWrite(a1, 200); //left
    analogWrite(a2, 0); //left b
    analogWrite(b1, 217); //right b
    analogWrite(b2, 0); //right
    strip.fill(amber);
    strip.show();
    Serial.println();
  }
    halt();
}

  void turnRight(int pulse1, int pulse2){
    setPulse1 = counter1 + pulse1;
    setPulse2 = counter2 + pulse2;
    while (setPulse1 >= counter1 && setPulse2 >= counter2){ // L && R
      analogWrite(a1, 0);
      analogWrite(a2, 200);
      analogWrite(b1, 0); //right motors
      analogWrite(b2, 217);
      strip.fill(amber);
      strip.show(); 
      Serial.println();
    }
      halt();
  }

 void moveBackward(int pulse1, int pulse2){
    setPulse1 = counter1 + pulse1;
    setPulse2 = counter2 + pulse2;
    while (setPulse1 >= counter1 && setPulse2 >= counter2){
      analogWrite(a1, 200); //lower speed to drive straight
      analogWrite(a2, 0);
      analogWrite(b1, 0); //right motors
      analogWrite(b2, 217);
      strip.fill(white);
      strip.show(); 
      Serial.println();
    }
      halt();
  }

void moveBackRight(int pulse1, int pulse2){
  setPulse1 = counter1 + pulse1;
  setPulse2 = counter2 + pulse2;
  while (setPulse1 >= counter1 && setPulse2 >= counter2){ // L && R
    analogWrite(a1, 170);
    analogWrite(a2, 0);
    analogWrite(b1, 0); //right motors
    analogWrite(b2, 255);
    strip.fill(white);
    strip.show();
     Serial.println();
  }
    halt();
}



////======[Logic]=====////

  void isStuck() {   
    static unsigned long interval = 0;
          
    if(millis() - interval >= 1000){
      if(previousCount != counter2 || previousCount == 0){
        previousCount = counter1;
      }
      else if(previousCount == counter1){
        moveBackRight(30, 30);
        return;
      }
     interval = millis();
    }
  }

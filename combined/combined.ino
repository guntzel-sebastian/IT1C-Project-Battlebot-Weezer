/****************************
*          [ io ]           *
****************************/
////=====[Digital Pins]=====////

  #include <Arduino.h>
  // 0 -> HR5
  // 1 -> HT
  // 2 -> R1
  const byte interruptPin1 = 0; // R1, Left rotation sensor
  // 3 -> R2
  const byte interruptPin2 = 3; // R2, Right rotation sensor, pwm pin
  // 4 -> N1
  #define pixelPin 2 // N1 Digital IO pin connected to the NeoPixels
  // 5 -> b2
  #define b2 5 // right motor, pwm
  // 6 -> b1w
  #define b1 6 // right motor, pwm
  // 7 -> [unassigned]
  // 8 -> "Eyes" Servo
  #define servoPin 8
  // 9 -> Trig
  const int trigPinForward = 9; // ultrasonic trigger pin
  const int trigPinLeft = 7;
  // 10 -> a1
  #define a1 10 // left motor, pwm
  // 11 -> a2
  #define a2 11 // left motor, pwm
  // 12 -> echo
  const int echoPinForward = 12; // ultrasonic echo pin 
  const int echoPinLeft = 4;
  // 13 -> GR
  #define gripperPin 13

  int gripperClosed = 0;
  int gripperOpen = 0;
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
*       [ Libraries ]       *
****************************/
////=====[Infrared Line Sensor]=====////
#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
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
*     [ Definitions ]       *
****************************/
////=====[Interupts]=====////
int counter1 = 0;
int counter2 = 0;
volatile byte state = LOW;
 int setPulse1 = 0;
  int setPulse2 = 0;
  bool movingForward;

////=====[Servo]=====////

#define servoMinPulse 500
#define servoCenterPulse 1400 // 
#define servoMaxPulse 2400 //in microseconds


////=====[Gripper]=====////

#define gripper_open_pulse 1600
#define gripper_close_pulse  971 // 
#define servoPulseRepeat 10 // number of pulse send to servo

////=====[Wheel Motors]=====////

const int wheelSpeed = 190; //Setting speed to maximum
unsigned long lastScanTime = 0;
int distanceForward = 50;
int distanceLeft; 
int distanceRight = 0;
int duration = 0;
int threshold = 800;
boolean turningLeft = false;
boolean forward = false;


////=====[Ultrasonic Distance Sensor]=====////

void setup()
{
  // infrared line sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);

// Calibration
if(gripperClosed==0) {
  moveForward(40,40);
}
  openGripper();
  for(uint16_t i = 0; i < 27; i++){
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

  //servo
  pinMode(servoPin, OUTPUT);
  pinMode(gripperPin, OUTPUT);
  digitalWrite(servoPin, LOW);

  //neopixels
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'
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

//  Serial.print(position);

// if the black line is spotted.
if(sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] > threshold && sensorValues[5] > threshold && sensorValues[6] > threshold && sensorValues[7] > threshold && gripperClosed == 0){
  halt();
  //close gripper
  //gripperClose
  closeGripper();
  gripperClosed = 1;
  turnLeft(7, 7);
  delay(1000);
  moveForward(30,30);
  delay(500);

} else if( sensorValues[3] > threshold && sensorValues[4] > threshold ){ 
      moveForward(20, 20);
  }
  else if(sensorValues[1] > threshold || sensorValues[2] > threshold){ // line is on the right
      turnRight(1, 1);
  } else if(sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] > threshold && sensorValues[5] > threshold && sensorValues[6] > threshold && sensorValues[7] > threshold && gripperClosed == 1){
    halt();
    openGripper();
    goBackward(40,40);
  }
  else if(sensorValues[5] > threshold || sensorValues[6] > threshold){ // line is on the left
      turnLeft(1, 1);
  } else if(sensorValues[0] < threshold && sensorValues[1] < threshold && sensorValues[2] < threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && gripperClosed == 1){
    
    lookForward();
    lookLeft();


      if(distanceLeft > 24) {
        moveForward(7,7);
        turnLeft(16,16);
        delay(1000);
        goForward();
        delay(500);
      } else if (distanceForward > 7) {
        moveForward(10, 10);
      } else {
        moveForward(3,3);
        turnRight(25,25);
        delay(50);
      }

//Serial.print("Counter 1: ");
//Serial.println(counter1);
//Serial.print("Counter 2: ");  
//Serial.println(counter2);

   

//      Serial.println(counter1);

//      if(sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] > threshold && sensorValues[5] > threshold && sensorValues[6] > threshold && sensorValues[7] > threshold && gripperClosed == 1){
//        halt();
//        //close gripper
//        //gripperClose
//        openGripper();
//        gripperClosed = 0;
//        goBackward(5, 5);
//        halt();
//  }
}
}
//}




////=====[NeoPixels]=====////

uint32_t white = strip.Color(255, 255, 255);
uint32_t red = strip.Color(255, 0, 0);
uint32_t amber = strip.Color(255, 75, 0);

/****************************
*      [ Functions ]        *
****************************/

////=====[Interrupts Counters]=====////

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

void counterReset(){
  counter1 = 0;
  counter2 = 0;
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

 int lookForward() {
  servo(servoPin, servoCenterPulse);
  delayMicroseconds(2);
  digitalWrite(trigPinForward, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinForward, LOW);
  int duration = pulseIn(echoPinForward, HIGH);
  distanceForward = duration / 58.2; // convert duration to distance in cm
  Serial.print("The distance Forward is: ");
  Serial.println(distanceForward);
}


////=====[Movement Interrupts]=====////

void moveForward(int pulse1, int pulse2){
    setPulse1 = counter1 - pulse1;
    setPulse2 = counter2 - pulse2;
    if (setPulse1 <= pulse1 && setPulse2 <= pulse2){
      analogWrite(a1, 0); //left motors, lower speed to drive straight
      analogWrite(a2, 200);
      analogWrite(b1, 215); //right motors
      analogWrite(b2, 0);
      strip.fill(white);
      strip.show();  
    }
    else{
      stopped();
      turningLeft = false;
      forward = false;
    }
  }
  

void goForward(){
    analogWrite(a1, 0); //lower speed to drive straight
    analogWrite(a2, 200);
    analogWrite(b1, 215);
    analogWrite(b2, 0);
     
    strip.fill(white);
    strip.show();  
}

void turnRight(int pulse1, int pulse2){
    setPulse1 = counter1 - pulse1;
    setPulse2 = counter2 - pulse2;
    if (setPulse1 <= pulse1 && setPulse2 <= pulse2){ // L && R
      movingForward = false;
      analogWrite(a1, 0);
      analogWrite(a2, 200);
      analogWrite(b1, 0); //right motors
      analogWrite(b2, 215);
      strip.fill(amber);
      strip.show(); 
      delay(250);
    }
    else{
      stopped();
    }
  }

void turnLeft(int pulse1, int pulse2){
    setPulse1 = counter1 - pulse1;
    setPulse2 = counter2 - pulse2;
    if (setPulse1 <= pulse1 && setPulse2 <= pulse2){
      movingForward = false;
      analogWrite(a1, 0); //left
      analogWrite(a2, 0); //left
      analogWrite(b1, 215); //right
      analogWrite(b2, 0); //right
      strip.fill(amber);
      strip.show();
    }
    else{
      stopped();
//      forward = true;
    }
  }

void goBackward(int pulse1, int pulse2){
  if (counter1 <= pulse1 && counter2 <= pulse2){
    analogWrite(a1, 200); //lower speed to drive straight
    analogWrite(a2, 0);
    analogWrite(b1, 0);
    analogWrite(b2, 215);
     
    strip.fill(white);
    strip.show();  
  }
  else{
    halt();
    counterReset();
  }
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

void openGripper() {
  servo(gripperPin, gripper_open_pulse);
}

void closeGripper(){
  servo(gripperPin, gripper_close_pulse);
}

 void stopped() {
    analogWrite(a1, 0); //lower speed to drive straight
    analogWrite(a2, 0);
    analogWrite(b1, 0);
    analogWrite(b2, 0); 
    counter1 = 0;
    counter2 = 0;
    strip.fill(red);
    strip.show(); 
  }


////=====[Distance Checking]=====////

//int lookRight() {
//  servo(servoPin, servoMinPulse);
//  delayMicroseconds(2);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  duration = pulseIn(echoPin, HIGH);
//  distanceRight = duration / 58.2;
//  Serial.print("The distance to the Right is: ");
//  Serial.println(distanceRight);
//}

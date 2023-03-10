/****************************
*          [ io ]           *
****************************/
////=====[Digital Pins]=====////
  
  // 0 -> HR
  // 1 -> HT
  // 2 -> R1
  const byte interruptPin1 = 2; // R1, Left rotation sensor
  // 3 -> R2
  const byte interruptPin2 = 3; // R2, Right rotation sensor, pwm pin
  // 4 -> N1
  #define pixelPin 4 // N1 Digital IO pin connected to the NeoPixels
  // 5 -> b2
  #define b2 5 // right motor, pwm
  // 6 -> b1
  #define b1 6 // right motor, pwm
  // 7 -> [unassigned]
  // 8 -> "Eyes" Servo
  #define servoPin 8
  // 9 -> Trig
  const int trigPin = 9; // ultrasonic trigger pin
  // 10 -> a1
  #define a1 10 // left motor, pwm
  // 11 -> a2
  #define a2 11 // left motor, pwm
  // 12 -> echo
  const int echoPin = 12; // ultrasonic echo pin 
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

////=====[Servo]=====////

#define servoMinPulse 500
#define servoCenterPulse 1400 // 
#define servoMaxPulse 2400 //in microseconds


////=====[Gripper]=====////

#define gripper_open_pulse 1600
#define gripper_close_pulse  971 // 
#define servoPulseRepeat 10 // number of pulse send to servo

////=====[Wheel Motors]=====////

const int speed = 255; //Setting speed to maximum
unsigned long lastScanTime = 0;
int distanceForward, distanceLeft, distanceRight, duration;

////=====[Ultrasonic Distance Sensor]=====////

void setup()
{
  // infrared line sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);

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
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //servo
  pinMode(servoPin, OUTPUT);
  pinMode(gripperPin, OUTPUT);
  digitalWrite(servoPin, LOW);

  //neopixels
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'

  Serial.begin(9600);
}


void loop()
{
//line sensor  
//  qtr.read(sensorValues);
//
//  for (uint8_t i = 0; i < SensorCount; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print('\t');
//  }

  justLookForward();

  static unsigned long start_time = millis();
  static unsigned long current_time = millis();
  static unsigned long lastCheckTime = 0;

  if (millis() - lastScanTime >= 2000) { // if it's been 2 seconds since the last scan
    halt(); // stop the car
    lastScanTime = millis(); // record the time of the scan
    scanDistances(); // scan for distances and update global variables
  }

//  if path right, turn right
//  if path forward, go forward
//  else go left.
  
   if (distanceLeft > 30) { // if there is no obstacle on the right
   turnLeft();
   } 
      
   if (distanceForward > 30) { // if there is no obstacle in front
   moveForward();
   } 
   else { // if there is no obstacle on the left
   turnRight();
   } 

  Serial.println(counter1);
}

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

////=====[NeoPixels]=====////

uint32_t white = strip.Color(255, 255, 255);
uint32_t red = strip.Color(255, 0, 0);
uint32_t amber = strip.Color(255, 75, 0);

////=====[Servos]=====////
void servo(int pin, int length) {
      for (int i=0; i < servoPulseRepeat; i++) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(length);//in microseconds
        digitalWrite(pin, LOW);
        delay(20); 
}
}

////=====[Distance Checking]=====////

void scanDistances() {
  lookForward();
  lookLeft(); // scan left
//  lookRight(); // scan right
  justLookForward(); // return to forward-facing position
}

int lookRight() {
  servo(servoPin, servoMinPulse);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceRight = duration / 58.2;
  delay(300);
  Serial.print("The distance to the Right is: ");
  Serial.println(distanceRight);
}

int lookLeft() {
  servo(servoPin, servoMaxPulse);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceLeft = duration / 58.2;
  delay(300);
  Serial.print("The distance to the Left is: ");
  Serial.println(distanceLeft);
}

 int justLookForward() {
  servo(servoPin, servoCenterPulse);
}

 int lookForward() {
  servo(servoPin, servoCenterPulse);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  distanceForward = duration / 58.2; // convert duration to distance in cm
  delay(300);
  Serial.print("The distance Forward is: ");
  Serial.println(distanceForward);
}

////=====[Movement Millis]=====////

void goForward() {
  // Set motor to move the car forward
  analogWrite(a1, LOW);
  digitalWrite(a2, speed);
  analogWrite(b1, speed);
  digitalWrite(b2, LOW);
  strip.fill(white);
  strip.show();
}

void goLeft () {
// Set motor to move the car left
  analogWrite(a1, speed);
  digitalWrite(a2, LOW);
  analogWrite(b1, speed);
  digitalWrite(b2, LOW);
  strip.fill(amber);
  strip.show(); 
}

void goBackward() {
  // Set motor to move the car backwards
  analogWrite(a1, speed);
  digitalWrite(a2, LOW);
  analogWrite(b1, LOW);
  digitalWrite(b2, speed);
  strip.fill(white);
  strip.show();
}
   
void goRight () {
  //Set motor to move the car right
  analogWrite(a1, LOW);
  digitalWrite(a2, speed);
  analogWrite(b1, LOW);
  digitalWrite(b2, speed);
  strip.fill(amber);
  strip.show(); 
}

void halt() {
  //Set motor to stop the car
  analogWrite(a1, LOW);
  digitalWrite(a2, LOW);
  analogWrite(b1, LOW);
  digitalWrite(b2, LOW);
  strip.fill(red);
  strip.show(); 
}

////=====[Movement Interrupts]=====////

void stop() {
  analogWrite(a1, 0); //lower speed to drive straight
  analogWrite(a2, 0);
  analogWrite(b1, 0);
  analogWrite(b2, 0); 
  strip.fill(red);
  strip.show(); 
}

void moveForward(){
  if (counter1 <= 45 && counter2 <= 45){
    analogWrite(a1, 0); //lower speed to drive straight
    analogWrite(a2, 245);
    analogWrite(b1, 255);
    analogWrite(b2, 0);
     
    strip.fill(white);
    strip.show();  
  }
  else{
    stop();
    counterReset();
  }
}

void turnRight() {
  if (counter1 <= 14 && counter2 <= 14){
  analogWrite(a1, 0);
  analogWrite(a2, 255);
  analogWrite(b1, 0);
  analogWrite(b2, 255); 
  strip.fill(amber);
  strip.show(); 
  }
  else{
    stop();
    counterReset();
  }
}

void turnLeft() {
  if (counter1 <= 14 && counter2 <= 14){
  analogWrite(a1, 255); //lower speed to drive straight
  analogWrite(a2, 0);
  analogWrite(b1, 255);
  analogWrite(b2, 0);
  strip.fill(amber);
  strip.show();
  }
  else{
    stop();
    counterReset();
  }
}

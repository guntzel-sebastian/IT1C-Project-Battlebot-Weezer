#include <Servo.h>

Servo gripper;


const int a1 = 12; // Pin for motor 1 (Right), direction 1 (clockwise)
const int a2 = 11; // Pin for motor 1 (Right), direction 2 (anti-clockwise)
const int b1 = 7; // Pin for motor 2 (left), direction 1 (clockwise)
const int b2 = 5; // Pin for motor 2 (left), direction 2 (anti-clockwise)
const int numSensors = 8;   // number of sensors
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};   // array to store the pin numbers of each sensor
int sensorValues[numSensors];   // array to store the readings from each sensor
const int speed = 255; //Setting speed to maximum
int pos = 0; // a variable to store the gripper position

// The pins for the ultrasonic sensor
const int trigPin = 9; // ultrasonic trigger pin
const int echoPin = 10; // ultrasonic echo pin 
const int distance = 20; //distance from bot to wall
const int defaultValue = 500; // value of the line following sensors


void setup() {
  // The motor pins as input pins
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);   // set each sensor pin as an input
  }
  
  // The trigger and echo pins for the ultrasonic sensor as output and input, respectively
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

//   gripper pin
   gripper.attach(13);
}


void loop() {

  static unsigned long start_time = millis();
  static unsigned long current_time = millis();
   current_time = millis();
//
//   while (true) {
//    // Check the line following sensors and determine the direction to turn
//    int line_following_value = checkLineFollowingSensors();
//    if (line_following_value > defaultValue) {
//      // Turn right
//      goRight();
//    } else if (line_following_value < -defaultValue) {
//      // Turn left
//      goLeft();
//    }

   if (current_time - start_time <= 10) {
  closeGripper();
   }
  
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;


    if (distance < 20) {
    goLeft();
    delay(500);
    }

  goForward();
 if (current_time - start_time >= 300000) {
    halt();
    delay(1000);
    openGripper();
    delay(10000);
    return;
 }
}


int checkLineFollowingSensors() {
  
    int totalValue = 0;   // variable to store the total value of all sensors

  // read the value of each sensor and add it to the total value
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    totalValue += sensorValues[i];
  }


  // calculate the average value by dividing the total value by the number of sensors
  int averageValue = totalValue / numSensors;

  // return the average value
  return averageValue;
}


void goForward() {
  // Set motor to move the car forward
  analogWrite(a1, LOW);
  digitalWrite(a2, speed);
  analogWrite(b1, speed);
  digitalWrite(b2, LOW);

}

void goLeft () {
// Set motor to move the car left
  analogWrite(a1, speed);
  digitalWrite(a2, LOW);
  analogWrite(b1, speed);
  digitalWrite(b2, LOW);
}

void goBackward() {
  // Set motor to move the car backwards
  analogWrite(a1, speed);
  digitalWrite(a2, LOW);
  analogWrite(b1, LOW);
  digitalWrite(b2, speed);
   }
   
void goRight () {
  //Set motor to move the car right
  analogWrite(a1, LOW);
  digitalWrite(a2, speed);
  analogWrite(b1, LOW);
  digitalWrite(b2, speed);
}

void halt() {
  //Set motor to stop the car
  analogWrite(a1, LOW);
  digitalWrite(a2, LOW);
  analogWrite(b1, LOW);
  digitalWrite(b2, LOW);
}

void openGripper() {
   for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    gripper.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the gripper to reach the position
  }
}

void closeGripper() {
 for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    gripper.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the gripper to reach the position
  }
}

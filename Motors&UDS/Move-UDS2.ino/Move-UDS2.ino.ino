#define servoPin 8
#define gripperPin 13
#define servoMinPulse 500
#define servoCenterPulse 1400 // 
#define servoMaxPulse 2400 //in microseconds

#define gripper_open_pulse 1600
#define gripper_close_pulse  971 // 
#define servoPulseRepeat 10 // number of pulse send to servo

#define a1 10 // Pin for motor 1 (Right), direction 1 (clockwise)
#define a2 11 // Pin for motor 1 (Right), direction 2 (anti-clockwise)
#define b1 6 // Pin for motor 2 (left), direction 1 (clockwise)
#define b2 5 // Pin for motor 2 (left), direction 2 (anti-clockwise)
const int speed = 255; //Setting speed to maximum
unsigned long lastScanTime = 0;
int distanceForward, distanceLeft, distanceRight, duration;

// The pins for the ultrasonic sensor
const int trigPin = 9; // ultrasonic trigger pin
const int echoPin = 12; // ultrasonic echo pin 


void setup() {
  Serial.begin(9600);
  // The motor pins as input pins
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);

  // The trigger and echo pins for the ultrasonic sensor as output and input, respectively
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(servoPin, OUTPUT);
  pinMode(gripperPin, OUTPUT);
  digitalWrite(servoPin, LOW);
}


void loop() {
  justLookForward();
  goForward();

  static unsigned long start_time = millis();
  static unsigned long current_time = millis();
  static unsigned long lastCheckTime = 0;

  if (millis() - lastScanTime >= 2000) { // if it's been 2 seconds since the last scan
    halt(); // stop the car
    delay(500);
    lastScanTime = millis(); // record the time of the scan
    scanDistances(); // scan for distances and update global variables
  }

//  if path right, turn right
//  if path forward, go forward
//  else go left.
  
   if (distanceLeft > 30) { // if there is no obstacle on the right
    goLeft();
    delay(500);
    } 
  
  if (distanceForward > 30) { // if there is no obstacle in front
    goForward();
  } else { // if there is no obstacle on the left
    goRight();
    delay(500);
  } 
  
}

void scanDistances() {
  lookForward();
  lookLeft(); // scan left
//  lookRight(); // scan right
  justLookForward(); // return to forward-facing position
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


void openGripper() {
  servo(gripperPin, gripper_open_pulse);
}

void closeGripper(){
  servo(gripperPin, gripper_close_pulse);
}

void goBackwards() {
  goLeft();
  goLeft();
}

void servo(int pin, int length) {
      for (int i=0; i < servoPulseRepeat; i++) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(length);//in microseconds
        digitalWrite(pin, LOW);
        delay(20); 
}
}

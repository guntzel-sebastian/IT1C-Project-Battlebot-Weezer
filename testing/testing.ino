  /****************************
  *          [ io ]           *
  ****************************/ 
  
  ////=====[Digital Pins]=====////
    
    // 0 RX -> HR
    
    // 1 TX -> HT
    
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
  int setPulse1 = 0;
  int setPulse2 = 0;
  boolean turningLeft = false;
  boolean forward = false;
  
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
  
  ////=====[Ultrasonic Distance Sensor]=====////
  
  double distanceForward = 0;
  double distanceLeft = 0; 
  double distanceRight = 0;
  int duration = 0;
  
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
  
  /********
  * LOOP *
  ********/
  
  void loop()
  {
    justLookForward();
  
    static unsigned long start_time = millis();
    static unsigned long current_time = millis();
    static unsigned long interval = 100;
    static unsigned long lastCheckTime = 0;
  
  //  if(millis() >= current_time + 300){
  //        time_now += 100;
  //        Serial.println("Hello");
  //  }
  
//    if (millis() - lastScanTime >= 3000) { // if it's been 2 seconds since the last scan
//      halt(); // stop the car
//      lastScanTime = millis(); // record the time of the scan
//      scanDistances(); // scan for distances and update global variables
//    }
    
  //  adjustPosition();


 
  
  //  if path right, turn right
  //  if path forward, go forward
  //  else go left.
  // 
  
solveIt();
//  delay(1000);

Serial.print("Counter 1: ");
Serial.println(counter1);
Serial.print("Counter 2: ");  
Serial.println(counter2);
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
  uint32_t green = strip.Color(0, 255, 0);
  
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
  
  double lookRight() {
    servo(servoPin, servoMinPulse);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distanceRight = duration / 58.2;
  //  delay(300);
  //  Serial.print("The distance to the Right is: ");
  //  Serial.println(distanceRight);
  }
  
  double lookLeft() {
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
  
  double lookForward() {
    servo(servoPin, servoCenterPulse);
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    int duration = pulseIn(echoPin, HIGH);
    distanceForward = duration / 58.2; // convert duration to distance in cm
  //  delay(300);
  
    Serial.print("The distance Forward is: ");
    Serial.println(distanceForward);
  }
  
  ////=====[Movement Interrupts]=====////
  void halt() {
    //Set motor to stop the car
    analogWrite(a1, LOW);
    digitalWrite(a2, LOW);
    analogWrite(b1, LOW);
    digitalWrite(b2, LOW);
    strip.fill(green);
    strip.show(); 
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
  
  void moveForward(int pulse1, int pulse2){
    setPulse1 = counter1 - pulse1;
    setPulse2 = counter2 - pulse2;
    if (setPulse1 <= pulse1 && setPulse2 <= pulse2){
      analogWrite(a1, 0); //lower speed to drive straight
      analogWrite(a2, 200);
      analogWrite(b1, 215);
      analogWrite(b2, 0);
      strip.fill(white);
      strip.show();  
    }
    else{
      stopped();
      turningLeft = false;
      forward = false;
      delay(1000);
    }
  }
  
  void goBackward(int pulse1, int pulse2){
    setPulse1 = counter1 - pulse1;
    setPulse2 = counter2 - pulse2;
    if (setPulse1 <= pulse1 && setPulse2 <= pulse2){
      analogWrite(a1, 200); //lower speed to drive straight
      analogWrite(a2, 0);
      analogWrite(b1, 0);
      analogWrite(b2, 215);
      strip.fill(white);
      strip.show(); 
    }
    else{
      stopped();
    }
  }
  
  void turnRight(int pulse1, int pulse2){
    setPulse1 = counter1 - pulse1;
    setPulse2 = counter2 - pulse2;
    if (setPulse1 <= pulse1 && setPulse2 <= pulse2){ // L && R
      analogWrite(a1, 0);
      analogWrite(a2, 200);
      analogWrite(b1, 0);
      analogWrite(b2, 215);
      strip.fill(amber);
      strip.show(); 
    }
    else{
      stopped();

    }
  }
  
  void turnLeft(int pulse1, int pulse2){
    setPulse1 = counter1 - pulse1;
    setPulse2 = counter2 - pulse2;
    if (setPulse1 <= pulse1 && setPulse2 <= pulse2){
      analogWrite(a1, 200); //left
      analogWrite(a2, 0); //left
      analogWrite(b1, 215); //right
      analogWrite(b2, 0); //right
      strip.fill(amber);
      strip.show();

    }
    else{
      stopped();
      forward = true;
      delay(1000);
    }
  }
  
  //key distances: 14 on either side, 8 in front
  
  //maze solving logic
  void solveIt()
  {
    if (turningLeft == false && forward == false){
      halt();
      scanDistances();
    }
    solveTheThing();
  }


  
  void solveTheThing() {

     if (distanceLeft > 15 && turningLeft == false) { // if there is no obstacle on the left    
        turningLeft = true;
        delay(1000);
        Serial.print("Boolean: ");
        Serial.println(turningLeft);
     }
     else if(turningLeft == true){
        if (!forward){
          turnLeft(9, 9);
        }
        if (forward) {   
          moveForward(24, 24);
        }
        Serial.print("Boolean: ");
        Serial.println(turningLeft);
     }
     else if(distanceForward > 15) { // if there is no obstacle in front
       forward = true;
       moveForward(24, 24);
     } 
     else { // if there is no obstacle on the left
      turnRight(9, 9);
     } 

  }
  
  
  //self-centering logic:
  void adjustPosition() {         
              if (distanceLeft != 0.00 && distanceLeft <= 6.00 || distanceForward != 0.00 && distanceForward <= 5.00) { //biggest adjustment, stuck on the wall
                goBackward (15, 15);
              }
              else if (distanceLeft > 6.00 && distanceLeft <= 7.00) { //bigger adjustment, too close to a wall, will collide with wall on the next movement, ~45 degrees
                turnRight(6, 6);
              }
              else if (distanceLeft > 7.00 && distanceLeft <= 8.00) { //big adjustment, still too close, might collide with wall in one to two movements ~34 degrees
                turnRight(5, 5);
              }
              else if (distanceLeft > 8.00 && distanceLeft <= 9.00) { //medium adjustment ~30 degrees
                turnRight(4, 4);
              } 
              else if (distanceLeft > 9.00 && distanceLeft <= 10.00) { //small adjustment, ~23 degrees
                turnRight(3, 3);
              }         
              else if (distanceLeft > 10.00 && distanceLeft <= 11.00) { //smaller adjustment, mostly center, in need of slight adjustment, ~19 degree turns
                turnRight(2, 2);
              }     
              else if (distanceLeft > 11.00 && distanceLeft <= 12.00) { //smallest adjustment, nearly perfectly centered/a microadjustment ~13 degrees turns
                turnRight(1, 1);
              }
              else { // if there is no obstacle on the left
                 if (distanceLeft > 15) { // if there is no obstacle on the left
                    Serial.println(counter1, counter2);
                 turnLeft(15, 15);
                    Serial.println(counter1, counter2);
                 moveForward(45, 45); 
                    Serial.println(counter1, counter2);
                 }  
                 else{
                   if (distanceForward > 15) { // if there is no obstacle in front
                   moveForward(45, 45);
                   } 
                   else { // if there is no obstacle on the left
                   turnRight(15, 15);
                   } 
                 }
              } 
  }

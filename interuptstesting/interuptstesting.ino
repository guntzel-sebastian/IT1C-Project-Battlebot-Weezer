//*****INTERRUPTS************************************************

int counter1 = 0;
int counter2 = 0;
const byte interruptPin1 = 2; //R1 is connected to the left rotation sensor
const byte interruptPin2 = 3; //R2 is connected to the right rotation sensor
volatile byte state = LOW;   

//*****MOTORS****************************************************

#define a1 11 //left motors
#define a2 12

#define b1 10 //right motors 
#define b2 7
            
void setup() {
//  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), counterLeft, CHANGE); //counterLeft
  attachInterrupt(digitalPinToInterrupt(interruptPin2), counterRight, CHANGE); //counterRight
  
  pinMode (a1, OUTPUT);
  pinMode (a2, OUTPUT);
  pinMode (b1, OUTPUT);
  pinMode (b2, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  moveForward();
   delay(1000);
  Serial.println(counter1);
 
}

//void blink() {
//  state = !state;
//}

void counterLeft(){
//  noInterrupts();
  counter1++;
//  interrupts();
}

void counterRight(){
//  noInterrupts();
  counter2++;
//  interrupts();
}

void stop(){
    analogWrite(a1, 0); //lower speed to drive straight
    analogWrite(a2, 0);
    analogWrite(b1, 0);
    analogWrite(b2, 0);
    counter1 = 0;
    counter2 = 0;
}

void turnRight(){
  if (counter1 < counter1 + 20 && counter2 < counter2 + 20){
  analogWrite(a1, 0);
  analogWrite(a2, 245);
  analogWrite(b1, 0);
  analogWrite(b2, 255);
  }
  else{
    stop();
  }
}

void turnLeft(){
  while (counter1 < 30 && counter2 < 30){
    analogWrite(a1, 245); //lower speed to drive straight
    analogWrite(a2, 0);
    analogWrite(b1, 255);
    analogWrite(b2, 0);
  }
}

void moveForward(){
  
  if (counter1 < 40 && counter2 < 40){
    analogWrite(a1, 245); //lower speed to drive straight
    analogWrite(a2, 0);
    analogWrite(b1, 0);
    analogWrite(b2, 255); 
  }
  else{
    stop();
  }
}

//drive loop//

//goForward
  //sensor checking if obstacles ahead
    //if nothing within 10cm, keep going forward
    //else (something within 10cm), stop
      //turn sensor right 90 degrees
        //take "snapshot"
          //is there an object within 10 cm?
            //if not, turn robot right 90 degrees
              //return sensor to forwards position
                ////goForward
            //if yes, turn sensor to the left (180 degrees)
              //take "snapshot"
                //is there an object within 10 cm? (there should not be)
                  //if not, turn robot left 90 degrees 
                       //return sensor to forwards position
                          ////goForward

//if IR sensor value is less than 800
  //begin US sensor drive loop
    //servo loop begins, constantly taking measurements of its surroundings
    
      //if left > 10 cm , turn 
      //if right > 10cm, turn 
      //if front < 10cm, stop
        //if left >10, turn left
          //else right >10, turn right

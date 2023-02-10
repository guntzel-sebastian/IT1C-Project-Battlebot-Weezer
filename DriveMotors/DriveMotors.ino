//motor io to digital pin
#define a1 11 //left motors
#define a2 12

#define b1 10 //right motors 
#define b2 7

void setup(){
  pinMode (a1, OUTPUT);
  pinMode (a2, OUTPUT);
  pinMode (b1, OUTPUT);
  pinMode (b2, OUTPUT);
}

void loop(){
  moveForward();
}

void moveForward(){
  analogWrite(a1, 245); //lower speed to drive straight
  analogWrite(a2, 0);
  analogWrite(b1, 0);
  analogWrite(b2, 255);
}

void moveBackward(){
  analogWrite(a1, 0); //lower speed to drive straight
  analogWrite(a2, 245);
  analogWrite(b1, 255);
  analogWrite(b2, 0);
}

//Millis
/*
 
 */

unsigned long previousMillis_1 = 0;
unsigned long previousMillis_2 = 0;

const long interval_1 = 1000;
const long interval_2 = 5;

// Adafruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define BUTTON_PIN 3 //Pin on the board that goes to N1

#define PIXEL_PIN 6 // Digital IO pin connected to the NeoPixels

#define PIXEL_COUNT 4 //Number of neopixels

Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_RGB
+ NEO_KHZ800);

boolean oldState = HIGH;
int     mode     = 0;    // Currently-active animation mode, 0-9

/* First Attempt
#define buttonOne  2 //brake
#define buttonTwo 3 //blink right
#define buttonThree  4 //blink left

int buttonStateOne = 0;
int buttonStateTwo= 0;
int buttonStateThree = 0;
*/

void setup() {
  
  //// Second Attempt
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'

  /* First Attempt
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
  #endif
  pixels.begin();

  pinMode(buttonOne, INPUT);
  pinMode(buttonTwo, INPUT);
  pinMode(buttonThree, INPUT);
  */
}

void loop() {
  //// Second Attempt
  // Get current button state.
  boolean newState = digitalRead(BUTTON_PIN);
  // Check if state changed from high to low (button press).
  if((newState == LOW) && (oldState == HIGH)) {
    // Short delay to debounce button.
    delay(20);
    // Check if button is still low after debounce.
    newState = digitalRead(BUTTON_PIN);
    if(newState == LOW) {      // Yes, still low
      if(++mode > 4) mode = 0; // Advance to next mode, wrap around after #8
      switch (mode){
        case 0 : //stopped
          stopped(strip.Color(255, 0, 0), 50); //red
          break;
        case 1 : //moveForward
          moveForward(strip.Color(255, 255, 255), 50); //white
          break;
        case 2 : //moveBackward
          moveBackward(strip.Color(255, 255, 255), 50); //white
          break;
        case 3 : //turnRight
          turnRight(strip.Color(255, 75, 0), 50); //amber
          break;
        case 4 : //turn left
          turnLeft(strip.Color(255, 75, 0), 50); //amber
          break;
      }
    }
  }
  oldState = newState;
}
void stopped(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void moveForward(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }  
}

void moveBackward(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void turnRight(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void turnLeft(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}
 /*
//moveForward
  pixels.setPixelColor(0, 0, 255, 0, 2);
  pixels.setPixelColor(1, 0, 255, 0, 2);
  pixels.setPixelColor(2, 255, 255, 255, 255);
  pixels.setPixelColor(3, 255, 255, 255, 255);
  
//moveBackward
  pixels.setPixelColor(0, 255, 255, 255, 2);
  pixels.setPixelColor(1, 255, 255, 255, 2);
  pixels.setPixelColor(2, 0, 255, 0, 255);
  pixels.setPixelColor(3, 0, 255, 0, 255);
  
//turnRight
  pixels.setPixelColor(1, 255, 75, 0, 255);
  pixels.setPixelColor(2, 255, 75, 0, 255);
  
//turnLeft
  pixels.setPixelColor(0, 255, 75, 0, 255);
  pixels.setPixelColor(3, 255, 75, 0, 255);
  
//stopped
  pixels.setPixelColor(0, 255, 0, 0, 255);
  pixels.setPixelColor(1, 255, 0, 0, 255);
  pixels.setPixelColor(2, 255, 0, 0, 255);
  pixels.setPixelColor(3, 255, 0, 0, 255);
  */

/* First Attempt

  buttonStateOne = digitalRead(buttonOne);
  buttonStateTwo = digitalRead(buttonTwo);
  buttonStateThree = digitalRead(buttonThree);
  unsigned long currentMillis = millis();
  pixels.clear();
  
  if(buttonStateOne == LOW){
    pixels.setPixelColor(0, 255, 0, 0, 255);
    pixels.setPixelColor(1, 255, 0, 0, 255);
    pixels.setPixelColor(2, 255, 255, 255, 255);
    pixels.setPixelColor(3, 255, 255, 255, 255);
  } 
  else if(buttonStateTwo == LOW){
      if (currentMillis - previousMillis_1 >= interval_1){
      previousMillis_1 = millis();
        pixels.setPixelColor(1, 255, 75, 0, 255);
        pixels.setPixelColor(2, 255, 75, 0, 255);
      }
      if (currentMillis - previousMillis_2 >= interval_2){
      previousMillis_2 = millis();
        pixels.setPixelColor(1, 0, 0, 0, 255);
        pixels.setPixelColor(2, 0, 0, 0, 255);
      }
  }
  else if(buttonStateThree == LOW){
    if (currentMillis - previousMillis_1 >= interval_1){
    previousMillis_1 = millis();
    }
      pixels.setPixelColor(0, 255, 75, 0, 255);
      pixels.setPixelColor(3, 255, 75, 0, 255);
    if (currentMillis - previousMillis_2 >= interval_2){
    previousMillis_2 = millis();
      pixels.setPixelColor(0, 0, 0, 0, 255);
      pixels.setPixelColor(3, 0, 0, 0, 255);
    }
  }
  else{
    pixels.setPixelColor(0, 0, 255, 0, 2);
    pixels.setPixelColor(1, 0, 255, 0, 2);
    pixels.setPixelColor(2, 255, 255, 255, 255);
    pixels.setPixelColor(3, 255, 255, 255, 255);
  }
    
  pixels.show();
*/

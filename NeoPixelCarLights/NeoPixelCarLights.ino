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

#define PIN 6 //Pin on the board that goes to N1
#define NUMPIXELS 4 //Number of numpixels on the board

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

//buttons
#define buttonOne  2 //brake
#define buttonTwo 3 //blink right
#define buttonThree  4 //blink left

int buttonStateOne = 0;
int buttonStateTwo= 0;
int buttonStateThree = 0;

void setup() {
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
  #endif
  pixels.begin();

  pinMode(buttonOne, INPUT);
  pinMode(buttonTwo, INPUT);
  pinMode(buttonThree, INPUT);
}

void loop() {
  buttonStateOne = digitalRead(buttonOne);
  buttonStateTwo = digitalRead(buttonTwo);
  buttonStateThree = digitalRead(buttonThree);
  unsigned long currentMillis = millis();
  pixels.clear();
  
//brake
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
}

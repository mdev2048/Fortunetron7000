#include "FastLED.h"

#define NUM_STRIPS 1
#define NUM_LEDS_PER_STRIP 50
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];


int ledStripPin = 6;


//distance sensor pin
const int pwPin = 7; 
long pulse, inches, cm;
int sum=0;//Create sum variable so it can be averaged
int avgrange=30;//Quantity of values to average (sample size)



int tempCounter = 0;

void blinkLedStrip(int tgtLevel, int tgtMax) {
  //lights up a a percentage of the led strips calculated by tgtLevel/ tgtMax
  for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
  
  if(i%tgtMax< tgtLevel) { 
  leds[0][i] = CRGB(random(0, 255),random(0, 255),random(0, 255));
  //leds[0][i] = CRGB(255,0,0);
  FastLED.show();
  } else {
  leds[0][i] = CRGB::Black;
  FastLED.show();
  }
  }
  
}

void read_sensor (){ //from http://www.maxbotix.com/articles/085-pt3.htm#CTSRSO
  pinMode(pwPin, INPUT);
    //Used to read in the pulse that is being sent by the MaxSonar device.
  //Pulse Width representation with a scale factor of 147 uS per Inch.

  pulse = pulseIn(pwPin, HIGH);
  //147uS per inch
  inches = pulse/147;
  //change inches to centimetres
  cm = inches * 2.54;
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
}

void setup() {

   Serial.begin(9600); 
  // set crystal ball led controller pin
  pinMode(ledStripPin, OUTPUT);     
  FastLED.addLeds<NEOPIXEL, 6>(leds[0], NUM_LEDS_PER_STRIP); //led strip pin needs to be hardcoded or build errors
}

void loop() {

  tempCounter += 1;
  tempCounter = tempCounter % 10;
 
  blinkLedStrip(tempCounter,10);
  delay(1000);               // wait for a second

  read_sensor();

}

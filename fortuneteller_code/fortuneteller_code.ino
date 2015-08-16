#include "FastLED.h"

#define NUM_STRIPS 1
#define NUM_LEDS_PER_STRIP 50
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];
const int ledStripPin = 5;

int crystalBallState = 0;
int crystalBallMax = 8;








/* =============== DISTANCE SENSOR VARIABLES ==============*/
const int pwPin = 7; 



long pulse, inches, cm;
int sum=0;//Create sum variable so it can be averaged
int avgrange=30;//Quantity of values to average (sample size)


int latchPin = 8;   //Pin connected to ST_CP of 74HC595
int dataPin =9;     //Pin connected to DS of 74HC595
int clockPin = 10;  //Pin connected to SH_CP of 74HC595


//holders for infromation you're going to pass to shifting function
byte data;
byte dataArray[10];


/*============ MOTION SENSOR VARIABLES ====================*/
// motion sensor variables
int pirPin = 3;    //the digital pin connected to the PIR sensor's output

//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
int calibrationTime = 30;        

//the time when the sensor outputs a low impulse
long unsigned int lowIn;         

//the amount of milliseconds the sensor has to be low 
//before we assume all motion has stopped
long unsigned int pause = 5000;  

boolean lockLow = true;
boolean takeLowTime;  

int curCrystalBallVal = 0;
int crystalBallThreshold = 880;


// etc
int tempCounter = 0;



void setup() {

   Serial.begin(9600); 
  // set crystal ball led controller pin
  pinMode(ledStripPin, OUTPUT);     
  FastLED.addLeds<NEOPIXEL, ledStripPin>(leds[0], NUM_LEDS_PER_STRIP); //led strip pin needs to be hardcoded or build errors




  pinMode(latchPin, OUTPUT);
  Serial.begin(9600);

  //Binary notation as comment
  dataArray[0] = 0xFF; //0b11111111
  dataArray[1] = 0xFF; //0b11111110
  dataArray[2] = 0xFC; //0b11111100
  dataArray[3] = 0xF8; //0b11111000
  dataArray[4] = 0xF0; //0b11110000
  dataArray[5] = 0xE0; //0b11100000
  dataArray[6] = 0xC0; //0b11000000
  dataArray[7] = 0x80; //0b10000000
  dataArray[8] = 0x00; //0b00000000
  dataArray[9] = 0xE0; //0b11100000

  //function that blinks all the LEDs
  //gets passed the number of blinks and the pause time
  blinkAll_2Bytes(2,500); 
  
}

void blinkAll_2Bytes(int n, int d) {
  digitalWrite(latchPin, 0);
  shiftOut(dataPin, clockPin, 0);
  shiftOut(dataPin, clockPin, 0);
  digitalWrite(latchPin, 1);
  delay(200);
  for (int x = 0; x < n; x++) {
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, 255);
    shiftOut(dataPin, clockPin, 255);
    digitalWrite(latchPin, 1);
    delay(d);
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, 0);
    shiftOut(dataPin, clockPin, 0);
    digitalWrite(latchPin, 1);
    delay(d);
  }
}
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

void calibrate_motion_sensor() {
  //give the sensor some time to calibrate
  Serial.print("calibrating sensor ");
  
  digitalWrite(pirPin, LOW);
  for(int i = 0; i < calibrationTime; i++){
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  delay(50);
  
}
// the heart of the program
void shiftOut(int myDataPin, int myClockPin, byte myDataOut) {
  // This shifts 8 bits out MSB first, 
  //on the rising edge of the clock,
  //clock idles low

  //internal function setup
  int i=0;
  int pinState;
  pinMode(myClockPin, OUTPUT);
  pinMode(myDataPin, OUTPUT);

  //clear everything out just in case to
  //prepare shift register for bit shifting
  digitalWrite(myDataPin, 0);
  digitalWrite(myClockPin, 0);

  //for each bit in the byte myDataOut�
  //NOTICE THAT WE ARE COUNTING DOWN in our for loop
  //This means that %00000001 or "1" will go through such
  //that it will be pin Q0 that lights. 
  for (i=7; i>=0; i--)  {
    digitalWrite(myClockPin, 0);

    //if the value passed to myDataOut and a bitmask result 
    // true then... so if we are at i=6 and our value is
    // %11010100 it would the code compares it to %01000000 
    // and proceeds to set pinState to 1.
    if ( myDataOut & (1<<i) ) {
      pinState= 1;
    }
    else {  
      pinState= 0;
    }

    //Sets the pin to HIGH or LOW depending on pinState
    digitalWrite(myDataPin, pinState);
    //register shifts bits on upstroke of clock pin  
    digitalWrite(myClockPin, 1);
    //zero the data pin after shift to prevent bleed through
    digitalWrite(myDataPin, 0);
  }

  //stop shifting
  digitalWrite(myClockPin, 0);
}



void read_distance_sensor (){ //from http://www.maxbotix.com/articles/085-pt3.htm#CTSRSO
  pinMode(pwPin, INPUT);
    //Used to read in the pulse that is being sent by the MaxSonar device.
  //Pulse Width representation with a scale factor of 147 uS per Inch.

  pulse = pulseIn(pwPin, HIGH);
  //147uS per inch
  inches = pulse/147;
  //change inches to centimetres
  cm = inches * 2.54;
  /*
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  */

  if( cm < 70) {
display_eyes(true);
  } else {
display_eyes(false);
  }
}

void read_motion_sensor() {
  if(digitalRead(pirPin) == HIGH){

    //makes sure we wait for a transition to LOW before any further output is made:
    Serial.println("---");
    Serial.print("motion detected at ");
    Serial.print(millis()/1000);
    Serial.println(" sec"); 
  }
}

int AnalogMaxima (int AnalogCh, int Threshold, int Delay){    
  int check1;                                  //variable to store first reading.
  int check2;                                  //variable to store second reading.

  check1 = analogRead(AnalogCh);               //Assing first reading 
  delay(Delay);                                //wait
  check2 = analogRead (AnalogCh);              //Assing second reading.
  if (check1>check2){                          //If voltage is DECREASING (no maxima)...
    return 1025;                               //end loop and return 1025.
  }
  else{
    while (analogRead(AnalogCh)>Threshold){     //While above threshold and RISING

      check1 = check2;                          //Write previous last reading as current first for comparison
      check2 = analogRead(AnalogCh);            //Assing second reading.
      delay(Delay/2);                           //wait,and loop unless...                
       
      if (check1>check2){                       //voltage drop is observed
        return check1;                          //if so return highest value :)

      }
    }
  }
} 

void display_eyes(bool tgtOutput) {
  if(tgtOutput) {
    for (int j = 0; j < 2; j++) {
    //load the light sequence you want from array
    data = 0xFF;
    //ground latchPin and hold low for as long as you are transmitting
    digitalWrite(latchPin, 0);
    //move 'em out
    shiftOut(dataPin, clockPin, data);
    //return the latch pin high to signal chip that it 
    //no longer needs to listen for information
    digitalWrite(latchPin, 1);
    
  
    }
  } else {
    for (int j = 0; j < 2; j++) {
    //load the light sequence you want from array
    data = 0x00;
    //ground latchPin and hold low for as long as you are transmitting
    digitalWrite(latchPin, 0);
    //move 'em out
    shiftOut(dataPin, clockPin, data);
    //return the latch pin high to signal chip that it 
    //no longer needs to listen for information
    digitalWrite(latchPin, 1);
    
  
    }
  
  }


}

int read_crystal_ball_sensor() {
  int cb_read_avg = 0;
  for(int i=0;i<10;i++) {
    cb_read_avg += analogRead(0);
    delay(10);
    
  }
  return cb_read_avg/10;
}
void loop() {

 

  delay(100);               // wait for a second

  read_distance_sensor();

  curCrystalBallVal = read_crystal_ball_sensor();
  Serial.println(curCrystalBallVal);
  if(curCrystalBallVal < crystalBallThreshold) {
    Serial.print("making it brighter ");
    crystalBallState += 1;
    crystalBallState = min(crystalBallState,crystalBallMax);
    Serial.println(crystalBallState);
  
  } else {

    crystalBallState -= 1;
    crystalBallState = max(crystalBallState,0);
  }
  blinkLedStrip(crystalBallState,crystalBallMax);
  

}

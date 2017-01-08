#include "FastLED.h"
#include <Servo.h> 

#define NUM_LEDS 20
#define DATA_PIN 4
#define CLOCK_PIN 5
#define BRIGHTNESS 255
CRGB leds[NUM_LEDS];

CRGBPalette16 currentPalette;
TBlendType currentBlending;

Servo ESC;  // create servo object to control a ESC

unsigned long RPMCount = 0;
bool trig = false;
unsigned long RPMTimer = 0;
unsigned int RPMInterval = 0;
double Frequency = 0.0;
unsigned int RPM = 0;
unsigned long LEDTimer = 0;
unsigned long MulTimer = 0;

int val;    // variable to read the value from the analog pin 
int MultiplierCount = 20;

#define PROG_SIZE 100
uint8_t pattern[] = {
   1, 99, 99, 99, 99, 99, 99, 99, 99, 99,
  99,  1, 99, 99, 99, 99, 99, 99, 99, 99,
  99, 99,  1, 99, 99, 99, 99, 99, 99, 99,
  99, 99, 99,  1, 99, 99, 99, 99, 99, 99,
  99, 99, 99, 99,  1, 99, 99, 99, 99, 99,
  99, 99, 99, 99, 99,  1, 99, 99, 99, 99,
  99, 99, 99, 99, 99, 99,  1, 99, 99, 99,  
  99, 99, 99, 99, 99, 99, 99,  1, 99, 99,
  99, 99, 99, 99, 99, 99, 99, 99,  1, 99,
  99, 99, 99, 99, 99, 99, 99, 99, 99,  1,      
};

void setup() 
{
  Serial.begin(57600);
  pinMode(14, INPUT);
  pinMode(22, OUTPUT);
  digitalWrite(22,LOW);
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN>(leds, NUM_LEDS);
  FastLED.clear();
  currentPalette = PartyColors_p;
  currentBlending = LINEARBLEND;

  ESC.attach(6);
}

void loop() 
{
  val = analogRead(A9);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 1000, 2000);     // scale it to use it with the servo (value between 0 and 180) 
  ESC.writeMicroseconds(val);                  // sets the servo position according to the scaled value 

//  Serial.println(val);
  
  if(analogRead(A0) > 1000)
  {
    if(trig == false)
    {
      RPMInterval = micros() - RPMTimer;
      Frequency = 1.0/((RPMInterval*3)/1000000.0);
      RPM = Frequency * 60.0;
      RPMCount += 1;
      Serial.print(Frequency);
      Serial.print(", ");
      Serial.println(RPM);
      trig = true;
      RPMTimer = micros();
    }
  }
  else
    trig = false;

  if(micros() - LEDTimer > 800)
  {
    LEDTimer = micros();
    static uint8_t startIndex = 0;
    for (uint8_t i = 0; i < 10; ++i) 
    {
      CRGB c;
      if (pattern[startIndex] == 99) 
      {
        c = CRGB::Black;   
      }
      else 
      {
        c = ColorFromPalette(currentPalette, pattern[startIndex], BRIGHTNESS, currentBlending);
      }
      
      leds[19 - i] = c;
      startIndex += 1;
      if (startIndex == PROG_SIZE) startIndex = 0;
    }
    
    FastLED.show();
  }

  if(millis() - MulTimer > 200)
  {
    MulTimer = millis();
    MultiplierCount += 1;
    if(MultiplierCount > 200)
      MultiplierCount = 20;
  }
}

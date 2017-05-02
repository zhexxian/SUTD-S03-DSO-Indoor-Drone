#include "FastLED.h"
#define NUM_LEDS 60
CRGB leds[NUM_LEDS];

void setup() { 
    FastLED.addLeds<NEOPIXEL, 6>(leds, NUM_LEDS); 
}

void loop() {
    for(int dot = 0; dot < NUM_LEDS; dot++) { 
        leds[dot] = CRGB::Blue;
        FastLED.show();
        // clear this led for the next time around the loop
        leds[dot] = CRGB::Black;
        delay(30);
    }
}
/*
ESP32 POV-LED
Modified from https://github.com/pololu/apa102-arduino/blob/master/APA102.h
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "Arduino.h"

#define clockPinA 22     // GPIO-PIN
#define dataPinA 23     // GPIO-PIN
#define clockPinB 25     // GPIO-PIN
#define dataPinB 26     // GPIO-PIN
#define clockPinC 32     // GPIO-PIN
#define dataPinC 33     // GPIO-PIN
#define numberOfLeds 20   
#define redAdjustedUpperLimit 155   
#define greenAdjustedUpperLimit 255  
#define blueAdjustedUpperLimit 255   


//Writing 1 byte (8 bits) of signal to Arduino
void writing8BitsToArduino(uint8_t b, uint8_t clockPin, uint8_t dataPin)
{
  digitalWrite(dataPin, b >> 7 & 1);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  digitalWrite(dataPin, b >> 6 & 1);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  digitalWrite(dataPin, b >> 5 & 1);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  digitalWrite(dataPin, b >> 4 & 1);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  digitalWrite(dataPin, b >> 3 & 1);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  digitalWrite(dataPin, b >> 2 & 1);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  digitalWrite(dataPin, b >> 1 & 1);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  digitalWrite(dataPin, b >> 0 & 1);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
}

//Start frame: 0*32
void startFrame(uint8_t clockPin, uint8_t dataPin)
{
  init(clockPin, dataPin);
  writing8BitsToArduino(0, clockPin, dataPin);
  writing8BitsToArduino(0, clockPin, dataPin);
  writing8BitsToArduino(0, clockPin, dataPin);
  writing8BitsToArduino(0, clockPin, dataPin);
}

//End frame
void endFrame(uint16_t count, uint8_t clockPin, uint8_t dataPin)
{
  for (uint16_t i = 0; i < (count + 14)/16; i++)
  {
      writing8BitsToArduino(0, clockPin, dataPin);
  }
  init(clockPin, dataPin);
}

uint8_t map_uint8(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void sendColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness, uint8_t clockPin, uint8_t dataPin)
{
  //First 3 bits of brightness need to be 1
  brightness = 0b11100000|brightness;

  red = map_uint8(red, 0, 255, 0, redAdjustedUpperLimit);
  green = map_uint8(green, 0, 255, 0, greenAdjustedUpperLimit);
  blue = map_uint8(blue, 0, 255, 0, blueAdjustedUpperLimit);
  writing8BitsToArduino(brightness, clockPin, dataPin);
  writing8BitsToArduino(blue, clockPin, dataPin);
  writing8BitsToArduino(green, clockPin, dataPin);
  writing8BitsToArduino(red, clockPin, dataPin);
}

void init(uint8_t clockPin, uint8_t dataPin)
{
  digitalWrite(dataPin, LOW);
  pinMode(dataPin, OUTPUT);
  digitalWrite(clockPin, LOW);
  pinMode(clockPin, OUTPUT);
}

void LED_test(uint8_t clockPin, uint8_t dataPin){
  int a;
  startFrame(clockPin, dataPin);

  for(a=0;a<numberOfLeds;a++){

    sendColor(20, 20, 20, 31, clockPin, dataPin);
  }
  
  endFrame(numberOfLeds-1, clockPin, dataPin);
  
}

void setup() {
  LED_test(clockPinA, dataPinA);
  LED_test(clockPinB, dataPinB);
  LED_test(clockPinC, dataPinC);
}

void loop() {
}

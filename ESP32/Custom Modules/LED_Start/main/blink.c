/*
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

#define clockPin 22     // GPIO-PIN
#define dataPin 23     // GPIO-PIN
#define numberOfLeds 20   
#define redAdjustedUpperLimit 155   
#define greenAdjustedUpperLimit 255  
#define blueAdjustedUpperLimit 255   

//Writing 1 byte (8 bits) of signal to Arduino
void writing8BitsToArduino(uint8_t b)
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
void startFrame()
{
	init();
	writing8BitsToArduino(0);
	writing8BitsToArduino(0);
	writing8BitsToArduino(0);
	writing8BitsToArduino(0);
}

//End frame
void endFrame(uint16_t count)
{
	for (uint16_t i = 0; i < (count + 14)/16; i++)
	{
    	writing8BitsToArduino(0);
	}
	init();
}

uint8_t map_uint8(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max)
{
  	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void sendColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
{
	//First 3 bits of brightness need to be 1
	brightness = 0b11100000|brightness;

	red = map_uint8(red, 0, 255, 0, redAdjustedUpperLimit);
	green = map_uint8(green, 0, 255, 0, greenAdjustedUpperLimit);
	blue = map_uint8(blue, 0, 255, 0, blueAdjustedUpperLimit);
	writing8BitsToArduino(brightness);
	writing8BitsToArduino(blue);
	writing8BitsToArduino(green);
	writing8BitsToArduino(red);
}

void init()
{
	digitalWrite(dataPin, LOW);
	pinMode(dataPin, OUTPUT);
	digitalWrite(clockPin, LOW);
	pinMode(clockPin, OUTPUT);
}

void app_main()
{
    nvs_flash_init();
	//initArduino();
	int a;
	startFrame();

	for(a=0;a<numberOfLeds;a++){

		sendColor(20, 20, 20, 31);
	}
	
	endFrame(numberOfLeds-1);

}








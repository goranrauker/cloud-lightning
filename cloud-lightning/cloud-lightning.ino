/*
 * Copyright (c) 2015 Molly Nicholas
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Molly Nicholas nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Adafruit_NeoPixel.h>
#include <BlynkSimpleEsp8266.h>

#if DEBUG
#define BLYNK_PRINT Serial
#endif

const char WIFI_SSID[] = "Willow's Den";
const char WIFI_PSK[] = "1234567890";
const char BLYNK_AUTH[] = "1d24281960104be9832021e96eace915";

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip)

int NUM_LEDS = 150;
int RAINBOW_LED_COUNT = 50;

int LED_PIN = 0;
int BOARD_LED_PIN = 5;

#define ON_OFF_BTN_PIN    V0
#define ZERGBA_PIN        V1
#define RAINBOW_BTN_PIN   V2

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

const int HIGH_STRIKE_LIKELIHOOD = 5;
const int LOW_STRIKE_LIKELIHOOD = 10;
int currentDataPoint = 0;
int chance = LOW_STRIKE_LIKELIHOOD;

// Simple moving average plot
int NUM_Y_VALUES = 17;

float yValues[] = {
  0,
  7,
  10,
  9,
  7.1,
  7.5,
  7.4,
  12,
  15,
  10,
  0,
  3,
  3.5,
  4,
  1,
  7,
  1
};

float simple_moving_average_previous = 0;
float random_moving_average_previous = 0;

float (*functionPtrs[10])(); //the array of function pointers
int NUM_FUNCTIONS = 2;

bool lightning = false;
bool rainbow_mode = false;
int rainbow_start_byte = 0;
int moodLighting = 0x000000;

void setup() {

  Serial.begin(9600);
  pinMode(BOARD_LED_PIN, OUTPUT);
  
  uint8_t led = 0;
  Blynk.begin(BLYNK_AUTH, WIFI_SSID, WIFI_PSK);
  while ( Blynk.connect() == false ) {
    led ^= 0x01;
    digitalWrite(BOARD_LED_PIN, led);
    delay(200);
    Serial.print(".");
  }
  digitalWrite(BOARD_LED_PIN, HIGH);
  Serial.println("connected");
  // Neopixel setup
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // initializes the array of function pointers.
  functionPtrs[0] = simple_moving_average;
  functionPtrs[1] = random_moving_average;
}

void loop() {

  Blynk.run();

  if (lightning && !rainbow_mode) {

    if (random(chance) == 3) {
      int led = random(NUM_LEDS);
      for (int i = 0; i < 10; i++) {
          // Use this line to keep the lightning focused in one LED.
          // lightningStrike(led):
          // Use this line if you want the lightning to spread out among multiple LEDs.
          lightningStrike(random(10, NUM_LEDS));
        }
        // Once there's been one strike, I make it more likely that there will be a second.
        chance = HIGH_STRIKE_LIKELIHOOD;
      } else {
        chance = LOW_STRIKE_LIKELIHOOD;
      }
      delay(500);
  } 
  if (rainbow_mode) {
    rainbow(rainbow_start_byte++);
    rainbow_start_byte = rainbow_start_byte > 10*NUM_LEDS ? 0 : rainbow_start_byte;
    delay(100);
  } else {
    setAllPixelsTo(150, moodLighting);
  }

  strip.show();
}

void setAllPixelsTo(int num_leds, int color) {
  for (int i = 0; i < num_leds; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void turnAllPixelsOff() {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, 0);
  }
  strip.show();
}

void lightningStrike(int pixel) {
  float brightness = callFunction(random(NUM_FUNCTIONS));
  float scaledWhite = abs(brightness*500);
  
  strip.setPixelColor(pixel, strip.Color(scaledWhite, scaledWhite, scaledWhite));
  strip.show();
  delay(random(5, 100));
//  delay(random(30, 100));
  currentDataPoint++;
  currentDataPoint = currentDataPoint%NUM_Y_VALUES;
}

float callFunction(int index) {
  return (*functionPtrs[index])(); //calls the function at the index of `index` in the array
}

// https://en.wikipedia.org/wiki/Moving_average#Simple_moving_average
float simple_moving_average() {
  uint32_t startingValue = currentDataPoint;
  uint32_t endingValue = (currentDataPoint+1)%NUM_Y_VALUES;
  float simple_moving_average_current = simple_moving_average_previous + 
                                  (yValues[startingValue])/NUM_Y_VALUES - 
                                  (yValues[endingValue])/NUM_Y_VALUES;

  simple_moving_average_previous = simple_moving_average_current;
  return simple_moving_average_current;
}


// Same as simple moving average, but with randomly-generated data points.
float random_moving_average() {
  float firstValue = random(1, 10);
  float secondValue = random(1, 10);
  float random_moving_average_current = random_moving_average_previous +
                                  firstValue/NUM_Y_VALUES -
                                  secondValue/NUM_Y_VALUES;
  random_moving_average_previous = random_moving_average_current;

  return random_moving_average_current;
}

BLYNK_WRITE(ON_OFF_BTN_PIN) {
  Serial.println("received blynk");
  if ( param.asInt() == 1) {
      lightning = true;
  } else {
      lightning = false;
  }
  strip.show();  
}

BLYNK_WRITE(RAINBOW_BTN_PIN) {
  Serial.println("received blynk");
  if ( param.asInt() == 1) {
      rainbow_mode = true;
  } else {
      rainbow_mode = false;
  }
}

BLYNK_WRITE(ZERGBA_PIN) {
  unsigned char color_r = param[0].asInt() / 1024.0 * 255 / 10;
  unsigned char color_g = param[1].asInt() / 1024.0 * 255 / 10;
  unsigned char color_b = param[2].asInt() / 1024.0 * 255 / 10;

  Serial.print(color_r); 
  Serial.print(color_g); 
  Serial.print(color_b); 

  moodLighting = (color_b & 0xFF) | ((color_g & 0xFF) << 8) | ((color_r & 0x0F) << 16);
  setAllPixelsTo(NUM_LEDS, moodLighting);
  Serial.println("");
}

void rainbow(byte startPosition) 
{  
  // Need to scale our rainbow. We want a variety of colors, even if there
  // are just 10 or so pixels.
  int rainbowScale = 192 / RAINBOW_LED_COUNT;
  
  // Next we setup each pixel with the right color
  for (int i=0; i < RAINBOW_LED_COUNT; i++)
  {
    // There are 192 total colors we can get out of the rainbowOrder function.
    // It'll return a color between red->orange->green->...->violet for 0-191.
    strip.setPixelColor(i, rainbowOrder((rainbowScale * (i + startPosition)) % 192));
  }
  // Finally, actually turn the LEDs on:
  strip.show();
}

// Input a value 0 to 191 to get a color value.
// The colors are a transition red->yellow->green->aqua->blue->fuchsia->red...
//  Adapted from Wheel function in the Adafruit_NeoPixel library example sketch
uint32_t rainbowOrder(byte position) 
{
  // 6 total zones of color change:
  if (position < 31)  // Red -> Yellow (Red = FF, blue = 0, green goes 00-FF)
  {
    return strip.Color(0xFF, position * 8, 0);
  }
  else if (position < 63)  // Yellow -> Green (Green = FF, blue = 0, red goes FF->00)
  {
    position -= 31;
    return strip.Color(0xFF - position * 8, 0xFF, 0);
  }
  else if (position < 95)  // Green->Aqua (Green = FF, red = 0, blue goes 00->FF)
  {
    position -= 63;
    return strip.Color(0, 0xFF, position * 8);
  }
  else if (position < 127)  // Aqua->Blue (Blue = FF, red = 0, green goes FF->00)
  {
    position -= 95;
    return strip.Color(0, 0xFF - position * 8, 0xFF);
  }
  else if (position < 159)  // Blue->Fuchsia (Blue = FF, green = 0, red goes 00->FF)
  {
    position -= 127;
    return strip.Color(position * 8, 0, 0xFF);
  }
  else  //160 <position< 191   Fuchsia->Red (Red = FF, green = 0, blue goes FF->00)
  {
    position -= 159;
    return strip.Color(0xFF, 0x00, 0xFF - position * 8);
  }
}

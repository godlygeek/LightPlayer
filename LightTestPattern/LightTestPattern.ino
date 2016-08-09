// Program for testing a single strip of lights.
// Cycles through each possible combination of pure colors:
// #000000 #FF0000 #00FF00 #FFFF00 #0000FF #FF00FF #00FFFF #FFFFFF
// For each color, lights up lights one at a time with a slight delay between
// individual lights, then leaves the whole strip lit up for 2 seconds.

#include <FAB_LED.h>

const uint8_t numPixels = 20;

apa106<D, 6> LEDstrip;
rgb pixels[numPixels] = {};

void setup()
{
}

void loop()
{
  static int color;

  for (int i = 0; i < numPixels; ++i) {
    pixels[i].r = !!(color & 1) * 255;
    pixels[i].g = !!(color & 2) * 255;
    pixels[i].b = !!(color & 4) * 255;
    delay(100);
    LEDstrip.sendPixels(numPixels, pixels);
  }

  color = (color + 1) % 8;

  // Display the pixels on the LED strip.
  LEDstrip.sendPixels(numPixels, pixels);
  delay(2000);
}

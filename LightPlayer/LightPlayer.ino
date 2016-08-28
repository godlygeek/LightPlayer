#include <SPI.h>
#include <SdFat.h>
#include <FAB_LED.h>

#define DEBUG 1

// ports used:
// digital 4 - SD card chip select
// digital 6 - strip of 200 APA106 LEDs
// digital 11 - SPI (SD card DI)
// digital 12 - SPI (SD card DO)
// digital 13 - SPI (SD card CLK)
// analog 0 - RF remote button D
// analog 1 - RF remote button C
// analog 2 - RF remote button B
// analog 3 - RF remote button A

namespace Pressed {
  const int none = 0;
  const int a    = 8;
  const int b    = 4;
  const int c    = 2;
  const int d    = 1;
  const int abcd = a|b|c|d;
  const int abc  = a|b|c;
  const int abd  = a|b|d;
  const int ab   = a|b;
  const int acd  = a|c|d;
  const int ac   = a|c;
  const int ad   = a|d;
  const int bcd  = b|c|d;
  const int bc   = b|c;
  const int bd   = b|d;
  const int cd   = c|d;
}

apa106<D, 6>   LEDstrip;
rgb frame[200];
uint8_t brightness = 255;

// Test with reduced SPI speed for breadboards.
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_FULL_SPEED;

#define BRIGHTNESS_STEP 32

//------------------------------------------------------------------------------
// File system object.
SdFat sd;
File infile;

unsigned long startMillis;

// Serial streams
ArduinoOutStream cout(Serial);

// SD card chip select
const int chipSelect = 4;

void setup()
{
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);

  Serial.begin(9600);

  // Wait for USB Serial
  while (!Serial) {
    SysCall::yield();
  }

  cout << F("\nInitializing SD.\n");
  if (!sd.begin(chipSelect, spiSpeed)) {
    if (sd.card()->errorCode()) {
      cout << F("SD initialization failed.\n");
      cout << F("errorCode: ") << hex << showbase;
      cout << int(sd.card()->errorCode());
      cout << F(", errorData: ") << int(sd.card()->errorData());
      cout << dec << noshowbase << endl;
      return;
    }

    cout << F("\nCard successfully initialized.\n");
    if (sd.vol()->fatType() == 0) {
      cout << F("Can't find a valid FAT16/FAT32 partition.\n");
      return;
    }
    if (!sd.vwd()->isOpen()) {
      cout << F("Can't open root directory.\n");
      return;
    }
    cout << F("Can't determine error type\n");
    return;
  }
  cout << F("\nCard successfully initialized.\n");
  cout << endl;

  sd.ls();

  nextFile();
}

void handleRfRemoteButtons()
{
  static int old_button_states;
  int button_states = PINC & B1111;

  if (button_states == old_button_states) {
    return;
  }
  old_button_states = button_states;

#ifdef DEBUG
  Serial.print(F("\nPressed buttons: "));
  switch (button_states) {
    case Pressed::none: Serial.print(F("NONE")); break;
    case Pressed::d:    Serial.print(F("   D")); break;
    case Pressed::c:    Serial.print(F("  C ")); break;
    case Pressed::cd:   Serial.print(F("  CD")); break;
    case Pressed::b:    Serial.print(F(" B  ")); break;
    case Pressed::bd:   Serial.print(F(" B D")); break;
    case Pressed::bc:   Serial.print(F(" BC ")); break;
    case Pressed::bcd:  Serial.print(F(" BCD")); break;
    case Pressed::a:    Serial.print(F("A   ")); break;
    case Pressed::ad:   Serial.print(F("A  D")); break;
    case Pressed::ac:   Serial.print(F("A C ")); break;
    case Pressed::acd:  Serial.print(F("A CD")); break;
    case Pressed::ab:   Serial.print(F("AB  ")); break;
    case Pressed::abd:  Serial.print(F("AB D")); break;
    case Pressed::abc:  Serial.print(F("ABC ")); break;
    case Pressed::abcd: Serial.print(F("ABCD")); break;
  }
#endif

  switch (button_states) {
    case Pressed::a: {
      brightness -= BRIGHTNESS_STEP; // overflow to 255 at zero
#ifdef DEBUG
      Serial.print(F("\nBrightness: "));
      Serial.print(brightness);
#endif
    } break;

    case Pressed::b: {
      nextFile();
    } break;
  }
}

void nextFile()
{
  if (infile.isOpen()) {
    infile.close();
  }

  while (true) {
    infile.openNext(sd.vwd());
    if (infile.isOpen()) {
      if (!infile.isHidden() && !infile.isSystem()) {
#ifdef DEBUG
        char buf[13];
        infile.getName(buf, sizeof(buf));
        cout << F("\nOpened file: ") << buf;
#endif
        startMillis = millis();
        return;
      }
      infile.close();
    } else {
      sd.vwd()->rewind();
    }
  }
}

void loop()
{
  handleRfRemoteButtons();

  int bytes_read = infile.read(frame, sizeof(frame));
  if (bytes_read != sizeof(frame)) {
    nextFile();
    return;
  }

  handleRfRemoteButtons();

#if defined(DEBUG) && DEBUG > 1
  Serial.print(F("\nFirst pixel before brightness scaling:\n"));
  Serial.print(frame[0]);
  Serial.print(F(" "));
  Serial.print(frame[1]);
  Serial.print(F(" "));
  Serial.print(frame[2]);
#endif

  // Adjust frame brightness
  for (int i = 0; i < sizeof(frame) / sizeof(*frame); ++i) {
    frame[i].r = map(frame[i].r, 0, 255, 0, brightness);
    frame[i].g = map(frame[i].g, 0, 255, 0, brightness);
    frame[i].b = map(frame[i].b, 0, 255, 0, brightness);
  }

#if defined(DEBUG) && DEBUG > 1
  Serial.print(F("\nFirst pixel after brightness scaling:\n"));
  Serial.print(frame[0]);
  Serial.print(F(" "));
  Serial.print(frame[1]);
  Serial.print(F(" "));
  Serial.print(frame[2]);
#endif

  while (millis() - startMillis < 49UL) {
      handleRfRemoteButtons();
  }

  while (millis() - startMillis < 50UL) {
    // busy loop until its time to paint the lights
  }
  startMillis += 50UL;

  LEDstrip.sendPixels(sizeof(frame) / sizeof(*frame), frame);
}

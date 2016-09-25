#include <SPI.h>
#include <SdFat.h>
#include <FAB_LED.h>

#define DEBUG

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

enum class Pressed : uint8_t {
  none = 0,
  a    = 8,
  b    = 4,
  c    = 2,
  d    = 1,
  abcd = a|b|c|d,
  abc  = a|b|c,
  abd  = a|b|d,
  ab   = a|b,
  acd  = a|c|d,
  ac   = a|c,
  ad   = a|d,
  bcd  = b|c|d,
  bc   = b|c,
  bd   = b|d,
  cd   = c|d,
};

enum class Mode : uint8_t {
  NORMAL,
  COLOR_CEILING,
  COLOR_FLOOR,
  VIDEO_PLAYBACK,
  MACRO,
  MISC,
} CurrentMode;

struct PendingOperations
{
  uint8_t cycle_brightness;
  uint8_t skip_video;

  uint8_t cycle_ceiling_red;
  uint8_t cycle_ceiling_green;
  uint8_t cycle_ceiling_blue;

  uint8_t cycle_floor_red;
  uint8_t cycle_floor_green;
  uint8_t cycle_floor_blue;

  uint8_t toggle_negative;
  uint8_t reset_settings;
};

volatile PendingOperations GlobalPendingOperations;

apa106<D, 6>   LEDstrip;
rgb frame[200];
uint8_t brightness = 255;
uint8_t negative;

struct ColorIntensity
{
  enum { MIN = 0, MAX = 8 };

  uint8_t floor = MIN;
  uint8_t ceil = MAX;

  void lowerCeil()
  {
    ceil = (ceil == floor ? MAX : ceil - 1);
  }

  void raiseFloor()
  {
    floor = (ceil == floor ? MIN : floor + 1);
  }

  void reset()
  {
    floor = MIN;
    ceil = MAX;
  }

  uint8_t minBrightness()
  {
    return map(floor, MIN, MAX, negative * brightness, !negative * brightness);
  }

  uint8_t maxBrightness()
  {
    return map(ceil, MIN, MAX, negative * brightness, !negative * brightness);
  }
};

ColorIntensity r_intensity;
ColorIntensity g_intensity;
ColorIntensity b_intensity;

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

  drawOwl(frame);
  LEDstrip.sendPixels(sizeof(frame) / sizeof(*frame), frame);

  Serial.begin(9600);

  // Wait for USB Serial
  while (!Serial) {
    SysCall::yield();
  }

  cout << F("\nInitializing SD.\n");
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
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
  startMillis = millis();

  PCMSK1 = B1111; // Enable interrupts for inputs A0, A1, A2, A3
  PCIFR  |= bit(PCIF1);   // clear any outstanding interrupts
  PCICR  |= bit(PCIE1);   // enable pin change interrupts for D8 to D13
}

void printPendingOperations(PendingOperations & ops)
{
  if (ops.cycle_brightness)    Serial.print(F("\ncycle_brightness"));
  if (ops.skip_video)          Serial.print(F("\nskip_video"));
  if (ops.cycle_ceiling_red)   Serial.print(F("\ncycle_ceiling_red"));
  if (ops.cycle_ceiling_green) Serial.print(F("\ncycle_ceiling_green"));
  if (ops.cycle_ceiling_blue)  Serial.print(F("\ncycle_ceiling_blue"));
  if (ops.cycle_floor_red)     Serial.print(F("\ncycle_floor_red"));
  if (ops.cycle_floor_green)   Serial.print(F("\ncycle_floor_green"));
  if (ops.cycle_floor_blue)    Serial.print(F("\ncycle_floor_blue"));
  if (ops.toggle_negative)     Serial.print(F("\ntoggle_negative"));
  if (ops.reset_settings)      Serial.print(F("\nreset_settings"));
}

void printSettings()
{
  Serial.print(F("\nMode: "));
  Serial.print((uint8_t)CurrentMode);
  Serial.print(F(" Brightness: max "));
  Serial.print(brightness);
  Serial.print(F(" r "));
  Serial.print(r_intensity.floor);
  Serial.print(F("-"));
  Serial.print(r_intensity.ceil);
  Serial.print(F(" g "));
  Serial.print(g_intensity.floor);
  Serial.print(F("-"));
  Serial.print(g_intensity.ceil);
  Serial.print(F(" b "));
  Serial.print(b_intensity.floor);
  Serial.print(F("-"));
  Serial.print(b_intensity.ceil);
  Serial.print(F(" neg "));
  Serial.print(negative);
}

void resetDefaultSettings()
{
  r_intensity.reset();
  g_intensity.reset();
  b_intensity.reset();
  brightness = 255;
  negative = 0;
}

void performPendingOperations(PendingOperations & ops)
{
#ifdef DEBUG
  printPendingOperations(ops);
#endif

  if (ops.cycle_brightness)    brightness -= BRIGHTNESS_STEP;
  if (ops.skip_video)          infile.close();
  if (ops.cycle_ceiling_red)   r_intensity.lowerCeil();
  if (ops.cycle_ceiling_green) g_intensity.lowerCeil();
  if (ops.cycle_ceiling_blue)  b_intensity.lowerCeil();
  if (ops.cycle_floor_red)     r_intensity.raiseFloor();
  if (ops.cycle_floor_green)   g_intensity.raiseFloor();
  if (ops.cycle_floor_blue)    b_intensity.raiseFloor();
  if (ops.toggle_negative)     negative = !negative;
  if (ops.reset_settings)      resetDefaultSettings();

#ifdef DEBUG
  if (    ops.cycle_brightness
       || ops.skip_video
       || ops.cycle_ceiling_red
       || ops.cycle_ceiling_green
       || ops.cycle_ceiling_blue
       || ops.cycle_floor_red
       || ops.cycle_floor_green
       || ops.cycle_floor_blue
       || ops.toggle_negative
       || ops.reset_settings)
  {
    printSettings();
  }
#endif
}

ISR(PCINT1_vect) // handle pin change interrupt for A0 to A5
{
  // Atomically read pins A0-A5, mask off A4 and A5, and represent
  // the states of A0, A1, A2, and A3 as a Pressed enum object
  auto button_states = static_cast<Pressed>(PINC & B1111);

  switch(CurrentMode) {
    case Mode::NORMAL: {
      switch (button_states) {
        case Pressed::a: {
          GlobalPendingOperations.cycle_brightness = 1;
        } break;

        case Pressed::b: {
          GlobalPendingOperations.skip_video = 1;
        } break;

        case Pressed::ab: {
          CurrentMode = Mode::COLOR_CEILING;
        } break;

        case Pressed::cd: {
          CurrentMode = Mode::COLOR_FLOOR;
        } break;

        case Pressed::bc: {
          CurrentMode = Mode::MISC;
        } break;
      }
    } break;

    case Mode::COLOR_CEILING: {
      switch (button_states) {
        case Pressed::d: {
          CurrentMode = Mode::NORMAL;
        } break;

        case Pressed::a: {
          GlobalPendingOperations.cycle_ceiling_red = 1;
        } break;

        case Pressed::b: {
          GlobalPendingOperations.cycle_ceiling_green = 1;
        } break;

        case Pressed::c: {
          GlobalPendingOperations.cycle_ceiling_blue = 1;
        } break;

      }
    } break;

    case Mode::COLOR_FLOOR: {
      switch (button_states) {
        case Pressed::d: {
          CurrentMode = Mode::NORMAL;
        } break;

        case Pressed::a: {
          GlobalPendingOperations.cycle_floor_red = 1;
        } break;

        case Pressed::b: {
          GlobalPendingOperations.cycle_floor_green = 1;
        } break;

        case Pressed::c: {
          GlobalPendingOperations.cycle_floor_blue = 1;
        } break;
      }
    } break;

    case Mode::MISC: {
      switch (button_states) {
        case Pressed::d: {
          CurrentMode = Mode::NORMAL;
        } break;

        case Pressed::a: {
          GlobalPendingOperations.toggle_negative = 1;
        } break;

        case Pressed::b: {
          GlobalPendingOperations.reset_settings = 1;
        } break;
      }
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
        return;
      }
      infile.close();
    } else {
      sd.vwd()->rewind();
    }
  }
}

bool readFrame()
{
  return infile.read(frame, sizeof(frame)) == sizeof(frame);
}

void adjustFrameColors()
{
  const uint8_t r_min_brightness = r_intensity.minBrightness();
  const uint8_t g_min_brightness = g_intensity.minBrightness();
  const uint8_t b_min_brightness = b_intensity.minBrightness();

  const uint8_t r_max_brightness = r_intensity.maxBrightness();
  const uint8_t g_max_brightness = g_intensity.maxBrightness();
  const uint8_t b_max_brightness = b_intensity.maxBrightness();

  for (auto & f : frame) {
    f.r = map(f.r, 0, 255, r_min_brightness, r_max_brightness);
    f.g = map(f.g, 0, 255, g_min_brightness, g_max_brightness);
    f.b = map(f.b, 0, 255, b_min_brightness, b_max_brightness);
  }
}

void loop()
{
  if (!readFrame()) {
    nextFile();
    startMillis = millis();
    return;
  }

  PendingOperations pending_operations;
  noInterrupts();
  memcpy(&pending_operations, &GlobalPendingOperations, sizeof(pending_operations));
  memset(&GlobalPendingOperations, 0, sizeof(GlobalPendingOperations));
  interrupts();

  performPendingOperations(pending_operations);

  adjustFrameColors();

  while (millis() - startMillis < 50UL) {
    // busy loop until its time to paint the lights
  }
  startMillis += 50UL;

  LEDstrip.sendPixels(sizeof(frame) / sizeof(*frame), frame);
}

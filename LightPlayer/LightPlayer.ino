#include <SPI.h>
#include <SdFat.h>
#include <FAB_LED.h>

//#define DEBUG

enum {
  SD_CHIP_SELECT = 4,
  APA106_DATA = 6,
  SD_MISO = 11,
  SD_MOSI = 12,
  SD_CLK = 13,
};

apa106<D, 6> LEDstrip; // This actually refers to pin 6 of the AVR D port, but
                       // that happens to map to the Arduino D6 pin.
rgb frame[200];

uint8_t r_lo = 0;
uint8_t r_hi = 255;

uint8_t g_lo = 0;
uint8_t g_hi = 255;

uint8_t b_lo = 0;
uint8_t b_hi = 255;

int8_t frame_skip;
uint8_t frame_stretch;

//------------------------------------------------------------------------------
// File system object.
SdFat sd;

struct SdContext {
  FatFile dir;
  File file;
  uint8_t index;
  uint8_t max_index;
};

SdContext RootContext;
SdContext RainbowContext;
SdContext *Context = &RootContext;

#define infile (&(Context->file))
#define curr_dir (&(Context->dir))
#define curr_index (Context->index)
#define max_index (Context->max_index)

unsigned long startMillis;

// Serial streams
ArduinoOutStream cout(Serial);

void setup()
{
  drawSplashScreen(frame);
  LEDstrip.sendPixels(sizeof(frame) / sizeof(*frame), frame);

  Serial.begin(9600);

  // Wait for USB Serial
  while (!Serial) {
    SysCall::yield();
  }

  cout << F("\nInitializing SD.\n");
  if (!sd.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
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

  RootContext.dir.openRoot(&sd);
  char rainbows_dir_name[9];
  strcpy_P(rainbows_dir_name, PSTR("rainbows"));
  RainbowContext.dir.open(&RootContext.dir, rainbows_dir_name, O_READ);

#ifdef DEBUG
  Serial.println(F("enumerating rainbows:"));
#endif
  Context = &RainbowContext;
  max_index = 0;
  while (curr_index >= max_index) {
    max_index = curr_index;
    nextFile();
  }

#ifdef DEBUG
  Serial.println(F("enumerating root fs:"));
#endif
  Context = &RootContext;
  max_index = 0;
  while (curr_index >= max_index) {
    max_index = curr_index;
    nextFile();
  }

  startMillis = millis();
}

void nextFile()
{
  if (infile->isOpen()) {
    infile->close();
  }

  while (true) {
    infile->openNext(curr_dir);
    if (infile->isOpen()) {
      if (!infile->isDir() && !infile->isHidden() && !infile->isSystem()) {
        ++curr_index;
#ifdef DEBUG
        char buf[13];
        infile->getName(buf, sizeof(buf));
        cout << F("\nnext opened file ") << (int16_t)curr_index << F(": ") << buf;
#endif
        return;
      }
      infile->close();
    } else {
      curr_dir->rewind();
      curr_index = 0;
    }
  }
}

void previousFile()
{
  if (infile->isOpen()) {
    infile->close();
  }

  while (true) {
    // dir size is 32.
    uint16_t index = curr_dir->curPosition()/32;
    if (index < 2) {
      // Advance to past last file of directory.
      dir_t dir;
      while (curr_dir->readDir(&dir) > 0);
      curr_index = max_index + 1;
      continue;
    }
    // position to possible previous file location.
    index -= 2;

    do {
      infile->open(curr_dir, index, O_READ);

      if (infile->isOpen()) {
        if (!infile->isDir() && !infile->isHidden() && !infile->isSystem()) {
          --curr_index;
#ifdef DEBUG
          char buf[13];
          infile->getName(buf, sizeof(buf));
          cout << F("\nprev opened file ") << (int16_t)curr_index << F(": ") << buf;
#endif
          return;
        }
        infile->close();
      }
    } while (index-- > 0);
  }
}

bool readFrame()
{
  int32_t seek_offset = (int32_t)frame_skip * sizeof(frame);
  if (frame_skip < 0 && infile->curPosition() < -seek_offset) {
    // Going backwards, and reached the start of this file.
    previousFile();
    // Successfully opened previous file - start from the end.
    infile->seekEnd();
  }
  if (!infile->seekCur(seek_offset)) {
    return false;
  }
  return infile->read(frame, sizeof(frame)) == sizeof(frame);
}

void adjustFrameColors()
{
  for (auto & f : frame) {
    f.r = map(f.r, 0, 255, r_lo, r_hi);
    f.g = map(f.g, 0, 255, g_lo, g_hi);
    f.b = map(f.b, 0, 255, b_lo, b_hi);
  }
}

void sendFrameToLights()
{
  unsigned long frame_time = 50UL + frame_stretch;
  while (millis() - startMillis < frame_time) {
    // busy loop until its time to paint the lights
  }
  startMillis += frame_time;

  LEDstrip.sendPixels(sizeof(frame) / sizeof(*frame), frame);
}

void loop()
{
  if (!readFrame()) {
    nextFile();
    startMillis = millis();
    return;
  }

  adjustFrameColors();
  sendFrameToLights();
}

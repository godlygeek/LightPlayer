#include <SPI.h>
#include <SdFat.h>
#include <FAB_LED.h>

#define DEBUG 1

apa106<D, 6>   LEDstrip;
rgb frame[200];

// Test with reduced SPI speed for breadboards.
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_FULL_SPEED;

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

void loop()
{
  int bytes_read = infile.read(frame, sizeof(frame));
  if (bytes_read != sizeof(frame)) {
    nextFile();
    startMillis = millis();
    return;
  }

  while (millis() - startMillis < 50UL) {
    // busy loop until its time to paint the lights
  }
  startMillis += 50UL;

  LEDstrip.sendPixels(sizeof(frame) / sizeof(*frame), frame);
}

#include <SPI.h>
#include <SdFat.h>
#include <FAB_LED.h>

apa106<D, 6>   LEDstrip;
rgb frame[200];

// Test with reduced SPI speed for breadboards.
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_FULL_SPEED;

//------------------------------------------------------------------------------
// File system object.
SdFat sd;

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

  if (!sd.exists("FRACTAL1.DAT")) {
    cout << F("FRACTAL1.DAT file not found.\n");
    return;
  }

  File infile = sd.open("FRACTAL1.DAT");
  if (!infile.isOpen()) {
    cout << F("Failed to open FRACTAL1.DAT\n");
    return;
  }

  int bytes_read = infile.read(frame, sizeof(frame));
  unsigned long prev_millis = millis();

  cout << F("\nFrame size in bytes: ") << sizeof(frame);
  cout << F("\nStarting millis: ") << prev_millis;
  int i = 0;
  while (bytes_read == sizeof(frame)) {
    ++i;
    while (millis() - prev_millis < 50UL) {
      // busy loop until its time to paint the lights
    }
    prev_millis += 50UL;
    LEDstrip.sendPixels(sizeof(frame) / sizeof(*frame), frame);
    bytes_read = infile.read(frame, sizeof(frame));
  }
  cout << F("\nFinal millis: ") << prev_millis;
  cout << F("\nNum frames: ") << i;
}

void loop()
{
}

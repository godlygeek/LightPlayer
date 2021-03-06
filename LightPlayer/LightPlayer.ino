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

const long FRAMES_PER_SEC = 20L;

char rxbuf[14];
uint8_t rxbuf_idx;

char status_string[27];

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

uint8_t directory;
uint8_t video;

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
unsigned long lastAdvertisementSecs;

// Serial streams
ArduinoOutStream cout(Serial);

static void setupFrameTimer()
{
  int ms_per_frame = 50 + frame_stretch;

  // Disable any timer interrupts
  TIMSK1 = 0;

  // Set up timer mode
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) // CTC mode - Count to a given value, then set a flag
         | (1 << CS12); // prescaler of 256, for ticks at 62.5 kHz;
  TCCR1C = 0;

  // timer 1 will begin at 0
  TCNT1 = 0;

  // and reset (setting a flag) after this many ticks:
  OCR1A = ms_per_frame * 62500UL / 1000 - 1;

  // Reset any pending flags
  TIFR1 = 0xFF;
}

void setup()
{
  drawSplashScreen(frame);
  LEDstrip.sendPixels(sizeof(frame) / sizeof(*frame), frame);

  Serial.begin(1200);

  if (!sd.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
    if (sd.card()->errorCode()) {
      cout << F("SD initialization failed.\n");
      cout << F("errorCode: ") << hex << showbase;
      cout << int(sd.card()->errorCode());
      cout << F(", errorData: ") << int(sd.card()->errorData());
      cout << dec << noshowbase << endl;
      return;
    }

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

  RootContext.dir.openRoot(&sd);

  char rainbows_dir_name[9];
  strcpy_P(rainbows_dir_name, PSTR("rainbows"));
  RainbowContext.dir.open(&RootContext.dir, rainbows_dir_name, O_READ);

#ifdef DEBUG
  Serial.println(F("enumerating rainbows:"));
#endif
  Context = &RainbowContext;
  max_index = 0;
  curr_dir->rewind();
  while (curr_index >= max_index) {
    max_index = curr_index;
    nextFile();
  }

#ifdef DEBUG
  Serial.println(F("enumerating root fs:"));
#endif
  Context = &RootContext;
  max_index = 0;
  curr_dir->rewind();
  while (curr_index >= max_index) {
    max_index = curr_index;
    nextFile();
  }

  setupFrameTimer();
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

void readNextFrame()
{
  // seekCur will fail if there aren't enough bytes left in the file.
  // Because our files and offsets are always under 2GB and unsigned underflow
  // is defined behavior, this is also true when seeking backwards.
  while (!infile->seekCur((int32_t)frame_skip * sizeof(frame))
          || infile->read(frame, sizeof(frame)) != sizeof(frame))
  {
    // Reached the end of a file (or a read failed); move to the next file.
    if (frame_skip < 0) {
      previousFile();
      infile->seekEnd();
    } else {
      nextFile();
    }
  }
}

void adjustFrameColors()
{
  for (auto & f : frame) {
    f.r = map(f.r, 0, 255, r_lo, r_hi);
    f.g = map(f.g, 0, 255, g_lo, g_hi);
    f.b = map(f.b, 0, 255, b_lo, b_hi);
  }
}

void setDirectory(uint8_t dir)
{
  if (dir == 0) {
    Context = &RootContext;
    directory = 0;
  } else if (dir == 1) {
    if (RainbowContext.dir.isOpen()) {
      Context = &RainbowContext;
      directory = 1;
    }
  }
}

void selectVideo(uint8_t new_video)
{
  if (new_video < max_index) {
    video = new_video;
    while (curr_index - 1 != new_video) {
      nextFile();
    }
  }
}

void seekToSecond(uint16_t second)
{
  int32_t offset = (second * FRAMES_PER_SEC * sizeof(frame));
  infile->seekSet(offset);
}

void handleSerialCommand(int nparams)
{
  uint8_t params[6];

  for (int i = nparams; i > 0; --i) {
    char *p = rxbuf + 2 * i - 1;
    params[i - 1] = strtol(p, 0, 16);
    *p = 0;
  }

  switch (rxbuf[0]) {
    case 'C':
      r_lo = params[0];
      r_hi = params[1];
      g_lo = params[2];
      g_hi = params[3];
      b_lo = params[4];
      b_hi = params[5];
      break;

    case 'S':
      frame_skip = params[0] - 128;
      frame_stretch = params[1];
      setupFrameTimer();
      break;

    case 'V':
      setDirectory(params[0]);
      selectVideo(params[1]);
      seekToSecond((params[2] << 8) + params[3]);
      break;
  }
}

int numParams(char c)
{
  switch (c) {
    case 'C': return 6;
    case 'S': return 2;
    case 'V': return 4;
  }
  return 0;
}

static int isHexChar(char c)
{
  if ('0' <= c && c <= '9') return 1;
  if ('a' <= c && c <= 'f') return 1;
  return 0;
}

void processSerialInput()
{
  while (Serial.available()) {
    int next_byte = Serial.read();
    int numhex = 2 * (rxbuf_idx ? numParams(rxbuf[0]) : numParams(next_byte));

    if (    (rxbuf_idx == 0 && numhex)
         || (1 <= rxbuf_idx && rxbuf_idx <= numhex && isHexChar(next_byte)))
    {
      rxbuf[rxbuf_idx++] = next_byte;
    }
    else if (rxbuf_idx > numhex && next_byte == '\n')
    {
      rxbuf[rxbuf_idx++] = 0;
      handleSerialCommand(numhex / 2);
      rxbuf_idx = 0;
    }
    else
    {
      // invalid character in the current position; reset
      rxbuf_idx = 0;
    }
  }
}

void populateAdvertisement()
{
  uint16_t seconds = infile->curPosition() / sizeof(frame) / FRAMES_PER_SEC;
  sprintf_P(status_string,
            PSTR("%02x%02x%02x%02x%02x%02x%02x%02x%04x%02x%02x%02x"),
            r_lo, r_hi, g_lo, g_hi, b_lo, b_hi,
            frame_skip + 128, frame_stretch,
            seconds,
            directory, curr_index - 1, video);
}

void sendAdvertisement()
{
  int secs = millis() / 1000;
  if (lastAdvertisementSecs != secs) {
    populateAdvertisement();
    Serial.print(status_string);
    Serial.print('\n');
    lastAdvertisementSecs = secs;
  }
}

void sendFrameToLights()
{
  while (!(TIFR1 & (1 << OCF1A))) {
    // busy loop until its time to paint the lights
  }
  TIFR1 = (1 << OCF1A); // clear the flag

  LEDstrip.sendPixels(sizeof(frame) / sizeof(*frame), frame);
}

void loop()
{
  readNextFrame();
  processSerialInput();
  sendAdvertisement();
  adjustFrameColors();
  sendFrameToLights();
}

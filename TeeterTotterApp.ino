
//
// NOTE: This uses i2c_t3 library to access Teensy 3.1's second SPI
// interface on pins 29 and 30. To do this, Adafruit_LIS3DH had to be
// modified to use i2c_t3 library by changing #include <Wire.h> to
// #include <i2c_t3.h> and by changing all instances of Wire to Wire1.
//
// More info here: https://forum.pjrc.com/threads/21680-New-I2C-library-for-Teensy3
//

#include <i2c_t3.h>
#include <SPI.h>
#include <Adafruit_LIS3DH_i2c_t3.h>
#include <Adafruit_Sensor.h>

#include <OctoWS2811.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

// Since we need both I2C Wire and Wire1.
#define I2C_BUS_ENABLE 2

#define BLACK 0x000000
#define BROWN 0x6A332F
#define RED    0xFF0000
#define ORANGE 0xFF6600
#define YELLOW 0xFFFF00
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define VIOLET 0xCC66FF
#define GREY 0x888888
#define WHITE  0xFFFFFF
#define PINK   0xFF1088
#define DGREEN  0x003300
#define DDGREEN  0x000600
#define DPURPLE 0x331133
#define DDPURPLE 0x190A19

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

const int numLedsPerStrip = 29;
const int numStrips = 6;

DMAMEM int displayMemory[numLedsPerStrip * numStrips];
int drawingMemory[numLedsPerStrip * numStrips];

const int config = WS2811_GRB | WS2811_800kHz;

OctoWS2811 leds(numLedsPerStrip, displayMemory, drawingMemory, config);

//////////////////////////////////////////////////////////////////////////////////////////////

#define MODE_TEST -2
#define MODE_IDLE -1
#define MODE_BEAM 0
#define MODE_MARBLE 1
#define MODE_LEVEL 2

const int numActiveModes = 3;
int currMode;

bool isSwitching;
bool isActive;
const int idleDelay = 10000;
const int modeDelay = 20000;
unsigned long idleStartTime;
unsigned long activeStartTime;
unsigned long prevSwitchTime;

float currTilt;
int tiltDirection;
const int maxTiltReadings = 5;
int numTiltReadings;
float tiltReadings[maxTiltReadings];
float prevTiltAverage;
float tiltAverage;
int tiltAverageDirection;
int prevTiltAverageDirection;
int tiltMoveDirection;
int prevTiltMoveDirection;

const int numMarbles = floor(numLedsPerStrip / 3);
int marblePositions[numMarbles];
int marbleSpeeds[numMarbles];
int marbleMaxSpeed;
int marbleMovesRemaining[numMarbles];
const int marbleResolution = 16;
const int numPositions = numLedsPerStrip * marbleResolution;

const int maxBeams = 8;
int numForwardBeams;
int numBackwardBeams;
int forwardBeamPositions[maxBeams];
int backwardBeamPositions[maxBeams];
const int beamWidth = 6;
const int beamSpeed = 1;
int prevBeamCreatedDirection;


void setup(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  leds.begin();
  leds.show();

  currMode = MODE_IDLE;

  Serial.print("numStrips: ");
  Serial.println(numStrips);
  Serial.print("numLedsPerStrip: ");
  Serial.println(numLedsPerStrip);
  Serial.println();

  isSwitching = false;
  isActive = false;
  idleStartTime = 0;
  activeStartTime = 0;
  prevSwitchTime = 0;

  numTiltReadings = 0;
  prevTiltAverage = 0;
  prevTiltAverageDirection = 0;
  prevTiltMoveDirection = 0;

  marbleMaxSpeed = 0;
  for (int i = 0; i < numMarbles; i++) {
    marblePositions[i] = (floor((numLedsPerStrip - numMarbles) / 2) + i) * marbleResolution;
  }

  numForwardBeams = 0;
  numBackwardBeams = 0;
  prevBeamCreatedDirection = 0;
}

void loop() {
  readTilt();

  switch (currMode) {
    case MODE_TEST:
      loopTestMode();
      break;
    case MODE_IDLE:
      loopIdleMode();
      break;
    case MODE_BEAM:
      loopBeamMode();
      break;
    case MODE_MARBLE:
      loopMarbleMode();
      break;
    case MODE_LEVEL:
      loopLevelMode();
      break;
  }

  delay(20);
}

void readTilt() {
  sensors_event_t event;
  lis.getEvent(&event);

  currTilt = event.acceleration.x / 10;
  tiltDirection = abs(currTilt) / currTilt;

  if (numTiltReadings < maxTiltReadings) {
    tiltReadings[numTiltReadings] = tiltReadings[numTiltReadings - 1];
    numTiltReadings++;
  }
  for (int i = numTiltReadings - 1; i > 0; i--) {
    tiltReadings[i] = tiltReadings[i - 1];
  }

  tiltReadings[0] = currTilt;
  if (numTiltReadings <= 0) {
    numTiltReadings = 1;
  }

  prevTiltAverage = tiltAverage;
  prevTiltAverageDirection = tiltAverageDirection;
  tiltAverage = 0;
  for (int i = 0; i < numTiltReadings; i++) {
    tiltAverage += tiltReadings[i];
  }
  tiltAverage /= numTiltReadings;
  tiltAverageDirection = abs(tiltAverage) / tiltAverage;

  float tiltMoveDirectionDelta = tiltReadings[0] - tiltReadings[numTiltReadings - 1];
  if (abs(tiltMoveDirectionDelta) > 0.05) {
    prevTiltMoveDirection = tiltMoveDirection;
    tiltMoveDirection = abs(tiltMoveDirectionDelta) / tiltMoveDirectionDelta;
  }

  unsigned long now = millis();
  isSwitching = tiltMoveDirection != prevTiltMoveDirection
      && abs(tiltAverage) > 0.1
      && prevBeamCreatedDirection != tiltAverageDirection;

  if (isSwitching) {
    prevSwitchTime = now;
    if (!isActive) {
      activeStartTime = now;
      isActive = true;
    }
  }

  if (now - prevSwitchTime > idleDelay && isActive) {
    idleStartTime = now;
    activeStartTime = 0;
    isActive = false;
  }

  if (isActive) {
    currMode = floor(((millis() - activeStartTime) % (numActiveModes * modeDelay)) / modeDelay);
  }
  else {
    currMode = MODE_IDLE;
  }
}

void loopTestMode() {
  for (int stripIndex = 0; stripIndex < numStrips; stripIndex++) {
    for (int ledIndex = 0; ledIndex < numLedsPerStrip; ledIndex++) {
      leds.setPixel(stripIndex * numLedsPerStrip + ledIndex, getTestModeColor(stripIndex, ledIndex));
    }
  }
  leds.show();
}

int getTestModeColor(int strip, int led) {
  float fractionLed = (float)led / numLedsPerStrip;

  if (currTilt < 0) {
    if (fractionLed < -currTilt) {
      return RED;
    }
    else {
      return BLACK;
    }
  }
  else {
    if (fractionLed < currTilt) {
      return GREEN;
    }
    else {
      return BLACK;
    }
  }
}

void loopIdleMode() {
  for (int stripIndex = 0; stripIndex < numStrips; stripIndex++) {
    for (int ledIndex = 0; ledIndex < numLedsPerStrip; ledIndex++) {
      leds.setPixel(stripIndex * numLedsPerStrip + ledIndex, getIdleModeColor(stripIndex, ledIndex));
    }
  }
  leds.show();
}

int getIdleModeColor(int strip, int led) {
  return DDPURPLE;
}

void loopMarbleMode() {
  int minSpeed = 3;
  marbleMaxSpeed = 0;
  for (int i = 0; i < numMarbles; i++) {
    marbleSpeeds[i] = (tiltDirection > 0 ? numMarbles - i - 1 + minSpeed : i + minSpeed)   *   8 * abs(currTilt);
    if (marbleSpeeds[i] > marbleMaxSpeed) {
      marbleMaxSpeed = marbleSpeeds[i];
    }
  }

  bool allMarblesMoved;

  if (abs(currTilt) > 0.05) {
    for (int i = 0; i < numMarbles; i++) {
      marbleMovesRemaining[i] = marbleSpeeds[i];
    }

    // repeat simulation step until all marbles have moved to ensure equal opportunity
    for (int step = 0; step < numMarbles * marbleMaxSpeed; step++) {
      allMarblesMoved = true;

      for (int i = 0; i < numMarbles; i++) {
        if (marbleMovesRemaining[i] > 0 && stepMarbleWalk(i)) {
          marbleMovesRemaining[i]--;
        }

        if (marbleMovesRemaining[i] > 0) {
          allMarblesMoved = false;
        }
      }

      if (allMarblesMoved) {
        break;
      }
    }
  }

  for (int stripIndex = 0; stripIndex < numStrips; stripIndex++) {
    for (int ledIndex = 0; ledIndex < numLedsPerStrip; ledIndex++) {
      leds.setPixel(stripIndex * numLedsPerStrip + ledIndex, getMarbleModeColor(stripIndex, ledIndex));
    }
  }
  leds.show();
}

bool stepMarbleWalk(int i) {
  int currPos = marblePositions[i];
  int targetPos = currPos - tiltDirection;
  if (targetPos >= 0 && targetPos + marbleResolution - 1 < numPositions && isOpenPosition(targetPos, i)) {
    marblePositions[i] = targetPos;
    return true;
  }
  return false;
}

int getMarbleModeColor(int strip, int led) {
  for (int i = 0; i < numMarbles; i++) {
    if (floor(marblePositions[i] / marbleResolution) == led) {
      return DGREEN;
    }
  }
  return BLACK;
}

bool isOpenPosition(int start, int exceptMarbleIndex) {
  int end = start + marbleResolution - 1;
  for (int i = 0; i < numMarbles; i++) {
    if (i == exceptMarbleIndex) continue;
    if ((marblePositions[i] <= start && start < marblePositions[i] + marbleResolution)
        || (marblePositions[i] <= end && end < marblePositions[i] + marbleResolution)) {
      return false;
    }
  }
  return true;
}

void loopBeamMode() {
  stepBeams();
  createBeams();

  if (abs(currTilt) < 0.15) {
    // Allow beam alternation to reset if we make it to halfway.
    prevBeamCreatedDirection = 0;
  }

  for (int stripIndex = 0; stripIndex < numStrips; stripIndex++) {
    for (int ledIndex = 0; ledIndex < numLedsPerStrip; ledIndex++) {
      leds.setPixel(stripIndex * numLedsPerStrip + ledIndex, getBeamModeColor(stripIndex, ledIndex));
    }
  }
  leds.show();
}

void stepBeams() {
  for (int i = numForwardBeams - 1; i >= 0; i--) {
    forwardBeamPositions[i] += beamSpeed;
    if (forwardBeamPositions[i] > numLedsPerStrip) {
      numForwardBeams--;
    }
  }

  for (int i = numBackwardBeams - 1; i >= 0; i--) {
    backwardBeamPositions[i] -= beamSpeed;
    if (backwardBeamPositions[i] < -beamWidth) {
      numBackwardBeams--;
    }
  }
}

void createBeams() {
  if (isSwitching) {
    if (tiltAverageDirection > 0) {
      createForwardBeam();
    }
    else {
      createBackwardBeam();
    }

    prevBeamCreatedDirection = tiltAverageDirection;
  }
}

void createForwardBeam() {
  if (numForwardBeams < maxBeams) {
    forwardBeamPositions[numForwardBeams] = forwardBeamPositions[numForwardBeams - 1];
    numForwardBeams++;
  }
  for (int i = numForwardBeams - 1; i > 0; i--) {
    forwardBeamPositions[i] = forwardBeamPositions[i - 1];
  }

  forwardBeamPositions[0] = -beamWidth;
  if (numForwardBeams <= 0) {
    numForwardBeams = 1;
  }
}

void createBackwardBeam() {
  if (numBackwardBeams < maxBeams) {
    backwardBeamPositions[numBackwardBeams] = backwardBeamPositions[numBackwardBeams - 1];
    numBackwardBeams++;
  }
  for (int i = numBackwardBeams - 1; i > 0; i--) {
    backwardBeamPositions[i] = backwardBeamPositions[i - 1];
  }

  backwardBeamPositions[0] = numLedsPerStrip - 1;
  if (numBackwardBeams <= 0) {
    numBackwardBeams = 1;
  }
}

int getBeamModeColor(int strip, int led) {
  bool hasForwardBeam = false;
  for (int i = 0; i < numForwardBeams; i++) {
    if (forwardBeamPositions[i] <= led && led < forwardBeamPositions[i] + beamWidth) {
      hasForwardBeam = true;
      break;
    }
  }
  for (int i = 0; i < numBackwardBeams; i++) {
    if (backwardBeamPositions[i] <= led && led < backwardBeamPositions[i] + beamWidth) {
      if (hasForwardBeam) {
        return DGREEN;
      }
      else {
        return DDGREEN;
      }
    }
  }

  if (hasForwardBeam) {
    return DDGREEN;
  }

  return BLACK;
}

void loopLevelMode() {
  int color;
  int halfWidth = 3;
  int tiltLedIndex = floor(numLedsPerStrip * (1 + tiltAverage) / 2);

  for (int stripIndex = 0; stripIndex < numStrips; stripIndex++) {
    for (int ledIndex = 0; ledIndex < numLedsPerStrip; ledIndex++) {
      if (tiltLedIndex - halfWidth < ledIndex && ledIndex < tiltLedIndex + halfWidth) {
        color = DDGREEN;
      }
      else {
        color = DDPURPLE;
      }
      leds.setPixel(stripIndex * numLedsPerStrip + ledIndex, color);
    }
  }
  leds.show();
}

///

float modTime(long period) {
  return (float)(millis() % period) / period;
}

float clampTime(float t) {
  while (t >= 1) {
    t -= 1;
  }
  while (t <= 0) {
    t += 1;
  }
  return t;
}

float splitTime(float t) {
  if (t < 0.5) {
    return 2.0 * t;
  }
  else {
    return (1.0 - t) * 2.0;
  }
}

void setPixelColor(int stripIndex, int pixelIndex, int color) {
  leds.setPixel(stripIndex * numLedsPerStrip + pixelIndex, color);
}

const int redMask = 0xFF0000, greenMask = 0xFF00, blueMask = 0xFF;
int lerpColor(int a, int b, float v) {
  int ar = (a & redMask) >> 16;
  int ag = (a & greenMask) >> 8;
  int ab = (a & blueMask);
  int br = (b & redMask) >> 16;
  int bg = (b & greenMask) >> 8;
  int bb = (b & blueMask);

  ar += (br - ar) * v;
  ag += (bg - ag) * v;
  ab += (bb - ab) * v;

  int rgb = (ar << 16) + (ag << 8) + ab;
  return rgb;
}

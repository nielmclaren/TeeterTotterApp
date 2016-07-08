
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

#define MODE_TEST 0
#define MODE_MARBLE_WALK 1

float currTilt;
int tiltDirection;
int currMode;
const int numMarbles = floor(numLedsPerStrip / 3);
int marblePositions[numMarbles];
bool marbleMoved[numMarbles];

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
  
  currMode = MODE_MARBLE_WALK;
  
  Serial.print("numStrips: ");
  Serial.println(numStrips);
  Serial.print("numLedsPerStrip: ");
  Serial.println(numLedsPerStrip);
  Serial.print("numMarbles: ");
  Serial.println(numMarbles);
  Serial.println();
  
  for (int i = 0; i < numMarbles; i++) {
    if (i < numMarbles/2) {
      marblePositions[i] = i;
    }
    else {
      marblePositions[i] = numLedsPerStrip - (numMarbles - i - 1) - 1;
    }
    marbleMoved[i] = false;
  }
}

void loop() {
  sensors_event_t event; 
  lis.getEvent(&event);
  
  stepMarbleWalk();
  
  currTilt = event.acceleration.x / 10;
  tiltDirection = abs(currTilt) / currTilt;
  
  for (int stripIndex = 0; stripIndex < numStrips; stripIndex++) {
    for (int ledIndex = 0; ledIndex < numLedsPerStrip; ledIndex++) {
      leds.setPixel(stripIndex * numLedsPerStrip + ledIndex, getColor(stripIndex, ledIndex));
    }
  }
  leds.show();
  
  delay(50); 
}

void stepMarbleWalk() {
  bool allMarblesMoved;
  
  if (abs(currTilt) > 0.1) {
    // repeat simulation step until all marbles have moved to ensure equal opportunity
    for (int step = 0; step < numMarbles; step++) {
      allMarblesMoved = true;
      for (int i = 0; i < numMarbles; i++) {
        if (step == 0 || !marbleMoved[i]) {
          marbleMoved[i] = stepMarbleWalk(i);
        }
        if (!marbleMoved[i]) {
          allMarblesMoved = false;
        }
      }
      if (allMarblesMoved) {
        break;
      }
    }
  }
}

bool stepMarbleWalk(int i) {
  int currPos = marblePositions[i];
  int targetPos = currPos + tiltDirection;
  if (targetPos >= 0 && targetPos < numLedsPerStrip && isOpenPosition(targetPos)) {
    marblePositions[i] = targetPos;
    return true;
  }
  return false;
}

bool isOpenPosition(int position) {
  for (int i = 0; i < numMarbles; i++) {
    if (marblePositions[i] == position) {
      return false;
    }
  }
  return true;
}

int getColor(int strip, int led) {
  switch (currMode) {
    case MODE_TEST:
      return getColorTestMode(strip, led);
    case MODE_MARBLE_WALK:
      return getColorMarbleWalkMode(strip, led);
  }
}

int getColorTestMode(int strip, int led) {
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

int getColorMarbleWalkMode(int strip, int led) {
  for (int i = 0; i < numMarbles; i++) {
    if (floor(marblePositions[i]) == led) {
      return DGREEN;
    }
  }
  return BLACK;
}


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


// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
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
}

void loop() {
  sensors_event_t event; 
  lis.getEvent(&event);
  
  float fractionX = event.acceleration.x / 10;
  Serial.print(event.acceleration.x);
  Serial.print("    ");
  Serial.println(fractionX);
  for (int stripIndex = 0; stripIndex < numStrips; stripIndex++) {
    for (int ledIndex = 0; ledIndex < numLedsPerStrip; ledIndex++) {
      int color;
      float fractionLed = (float)ledIndex / numLedsPerStrip;
      
      if (fractionX < 0) {
        if (fractionLed < -fractionX) {
          color = RED;
        }
        else {
          color = BLACK;
        }
      }
      else {
        if (fractionLed < fractionX) {
          color = GREEN;
        }
        else {
          color = BLACK;
        }
      }
      
      leds.setPixel(stripIndex * numLedsPerStrip + ledIndex, color);
      leds.show();
    }
  }
  
  delay(50); 
}

void colorWipe(int color, int wait)
{
  for (int i=0; i < leds.numPixels(); i++) {
    leds.setPixel(i, color);
    leds.show();
    delayMicroseconds(wait);
  }
}

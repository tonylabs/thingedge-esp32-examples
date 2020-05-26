#include "SHT3X.h"
#include <FastLED.h>

SHT3X objSHT30(0x44);

#define LED_PIN     17
#define NUM_LEDS    4
#define BRIGHTNESS  200
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB arrayLEDs[NUM_LEDS];

#define UPDATES_PER_SECOND 100

byte getCelsius;

void setPixel(int i, byte red, byte green, byte blue)
{
  arrayLEDs[i].r = red;
  arrayLEDs[i].g = green;
  arrayLEDs[i].b = blue;
}

void setup()
{
  delay(1000); //@Power-up safety delay
  Serial.begin(115200);
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(arrayLEDs, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
}

void loop()
{
  if(objSHT30.get()==0)
  {
    Serial.print("====== TEMPERATURE IN CELSIUS: ");
    getCelsius = objSHT30.cTemp;
    Serial.println(getCelsius);

    getCelsius = map(getCelsius, 0, 30, 10, 255);
    Serial.print("====== MAPPED TO RED COLOR: ");
    Serial.println(getCelsius);
  }
  else
  {
    getCelsius = 0;
  }

  for( uint8_t i = 0; i < NUM_LEDS; i++)
  {
    setPixel(i, getCelsius*10, 0, 0);
  }
  FastLED.show();
  //FastLED.delay(UPDATES_PER_SECOND);
}

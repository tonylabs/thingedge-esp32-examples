#include <FastLED.h>

#define LED_PIN     17
#define NUM_LEDS    4
#define BRIGHTNESS  255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB arrayLEDs[NUM_LEDS];

void initWS2812()
{
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(arrayLEDs, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 50); // FastLED Power management set at 5V, 100mA.
  FastLED.clear();
  FastLED.show();
}
void setAll(byte red, byte green, byte blue)
{
  fill_solid(arrayLEDs, NUM_LEDS, CRGB( red, green, blue));
  FastLED.show();
}

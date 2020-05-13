#include "relay.h"
#include "WS2812.h"
#include "buzzer.h"

void setup()
{
  //delay( 1000 ); // power-up safety delay

  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  initWS2812();
  setAll(255, 255, 255);

  initButtons();
  initRelay();

  //@Buzzer
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(BUZZER_PIN, channel);

  ledcWriteTone(channel, freq);
  delay(500);
  ledcWriteTone(channel, 0);
  delay(500);
  
}

void loop()
{
  objUserButton1.read();
  objUserButton2.read();
}

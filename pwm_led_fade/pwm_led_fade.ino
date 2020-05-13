/*
 * Copyright (c) 2018, TONYLABS CO., LTD.
 * All rights reserved.
/* Generates PWM on Internal LED Pin GPIO 2 of ESP32*/

#define LED 16

int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

//@Setting PWM Properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 10; //Resolution 8, 10, 12, 15

void setup()
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO2 to be controlled
  ledcAttachPin(LED, ledChannel);
}

void loop()
{
  //PWM Value varries from 0 to 1023  
  Serial.println("10 % PWM");
  ledcWrite(ledChannel, 102);
  delay(2000);

  Serial.println("20 % PWM");
  ledcWrite(ledChannel,205);
  delay(2000);

  Serial.println("40 % PWM");
  ledcWrite(ledChannel,410);
  delay(2000);

  Serial.println("70 % PWM");
  ledcWrite(ledChannel,714);
  delay(2000);

  Serial.println("100 % PWM");
  ledcWrite(ledChannel,1024);
  delay(2000);

  //Continuous Fading
  Serial.println("Fadding Started");
  while(1)
  {
    // set the brightness of pin 2:
    ledcWrite(ledChannel, brightness);
  
    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;
  
    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 1023) {
      fadeAmount = -fadeAmount;
    }
    // wait for 30 milliseconds to see the dimming effect
    delay(10);
  }
}
//=======================================================================

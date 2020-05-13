#include <EasyButton.h>

//@Buttons
#define USR_BTN1_PIN 25
#define USR_BTN2_PIN 26

EasyButton objUserButton1(USR_BTN1_PIN);
EasyButton objUserButton2(USR_BTN2_PIN);

//@RELAY PINS
#define RELAY1_PIN 33
#define RELAY2_PIN 32

void relayOneControl()
{
  digitalWrite(RELAY1_PIN, !digitalRead(RELAY1_PIN));
}

void relayTwoControl()
{
  digitalWrite(RELAY2_PIN, !digitalRead(RELAY2_PIN));
}

void initButtons(void)
{
  objUserButton1.begin();
  objUserButton1.onPressed(relayOneControl);
  
  objUserButton2.begin();
  objUserButton2.onPressed(relayTwoControl);
}

void setup()
{
  initButtons();
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
}

void loop()
{
  objUserButton1.read();
  objUserButton2.read();
}

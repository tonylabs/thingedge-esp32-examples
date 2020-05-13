#include <EasyButton.h>

//@Buttons
#define USR_BTN1_PIN 25
#define USR_BTN2_PIN 26
#define USR_BTN3_PIN 27
EasyButton objUserButton1(USR_BTN1_PIN);
EasyButton objUserButton2(USR_BTN2_PIN);
EasyButton objUserButton3(USR_BTN3_PIN);

void onButtonPressed()
{
  Serial.println("====== USER BUTTON PRESSED.");
}

void onPressedForDuration()
{
  Serial.println("====== USER BUTTON PRESSED FOR 5 SECONDS.");
}

void onSequenceMatched()
{
  Serial.println("====== USER BUTTON PRESSED 3 TIMES");
}

void initButtons(void)
{
  objUserButton1.begin();
  objUserButton1.onPressed(onButtonPressed);
  objUserButton1.onPressedFor(5000, onPressedForDuration);
  objUserButton1.onSequence(3 /* number of presses */, 2000 /* timeout */, onSequenceMatched /* callback */);
  
  objUserButton2.begin();
  objUserButton2.onPressed(onButtonPressed);
  objUserButton2.onPressedFor(5000, onPressedForDuration);
  objUserButton2.onSequence(3 /* number of presses */, 2000 /* timeout */, onSequenceMatched /* callback */);

  objUserButton3.begin();
  objUserButton3.onPressed(onButtonPressed);
  objUserButton3.onPressedFor(5000, onPressedForDuration);
  objUserButton3.onSequence(3 /* number of presses */, 2000 /* timeout */, onSequenceMatched /* callback */);
}

void setup()
{
  Serial.begin(115200);
  initButtons();
}

void loop()
{
  objUserButton1.read();
  objUserButton2.read();
  objUserButton3.read();
}

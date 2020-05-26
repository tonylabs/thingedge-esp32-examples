#include "BQ27441.h"
#include <EasyButton.h>

#define USR_BTN1_PIN 25
#define CHARGER_ENABLE_PIN 13

BQ27441 objCharger;
EasyButton objUserButton1(USR_BTN1_PIN);

void toggleCharger(void)
{
  Serial.println("====== TOGGLE CHARGE ENABLE PIN");
  digitalWrite(CHARGER_ENABLE_PIN, !digitalRead(CHARGER_ENABLE_PIN));
}

void initButtons(void)
{
  objUserButton1.begin();
  objUserButton1.onPressed(toggleCharger);
}

// Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 6800; // e.g. 850mAh battery

void initBQ27441(void)
{
  pinMode(CHARGER_ENABLE_PIN, OUTPUT);
  digitalWrite(CHARGER_ENABLE_PIN, HIGH);
  
  // Use objCharger.begin() to initialize the BQ27441-G1A and confirm that it's
  // connected and communicating.
  if (!objCharger.begin()) // begin() will return true if communication is successful
  {
    // If communication fails, print an error message and loop forever.
    Serial.println("Error: Unable to communicate with BQ27441.");
    Serial.println("  Check wiring and try again.");
    Serial.println("  (Battery must be plugged into Battery Babysitter!)");
    while (1) ;
  }
  Serial.println("====== BQ27441 CONNECTED!");
  objCharger.setCapacity(BATTERY_CAPACITY);
}

void printBatteryStats()
{
  // Read battery stats from the BQ27441-G1A
  unsigned int soc = objCharger.soc();  // Read state-of-charge (%)
  unsigned int volts = objCharger.voltage(); // Read battery voltage (mV)
  int current = objCharger.current(AVG); // Read average current (mA)
  unsigned int fullCapacity = objCharger.capacity(FULL); // Read full capacity (mAh)
  unsigned int capacity = objCharger.capacity(REMAIN); // Read remaining capacity (mAh)
  int power = objCharger.power(); // Read average power draw (mW)
  int health = objCharger.soh(); // Read state-of-health (%)

  // Now print out those values:
  String toPrint = String(soc) + "% | ";
  toPrint += String(volts) + " mV | ";
  toPrint += String(current) + " mA | ";
  toPrint += String(capacity) + " / ";
  toPrint += String(fullCapacity) + " mAh | ";
  toPrint += String(power) + " mW | ";
  toPrint += String(health) + "%";
  
  Serial.println(toPrint);
}

void setup()
{
  Serial.begin(115200);
  
  initBQ27441();
  initButtons();
}

void loop() 
{
  printBatteryStats();
  objUserButton1.read();
  delay(500);
}

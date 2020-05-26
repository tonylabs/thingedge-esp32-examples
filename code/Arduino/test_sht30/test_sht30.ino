#include "SHT3X.h"

SHT3X objSHT30(0x44);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  if(objSHT30.get()==0)
  {
    Serial.print("====== Temperature in Celsius : ");
    Serial.println(objSHT30.cTemp);
    Serial.print("====== Temperature in Fahrenheit : ");
    Serial.println(objSHT30.fTemp);
    Serial.print("====== Relative Humidity : ");
    Serial.println(objSHT30.humidity);
    Serial.println();
  }
  else
  {
    Serial.println("Error!");
  }
  delay(500);
}

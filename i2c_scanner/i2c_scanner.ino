#include <Wire.h>


/*
 * I2C Device on Board
 * 0x1C LSM9DS1
 * 0x44 SHT31
 * 0x45 OPT3001
 * 0x4C MMA7660
 * 0x55 BQ27441 Fuel Gauge
 * 0x60 ATECC508A
 * 0x6A LSM9DS1 
 */
 
void setup()
{
  Wire.begin();
  Serial.begin(115200);
}

void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("\nI2C Scanner");
  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("Done");

  delay(5000);           // wait 5 seconds for next scan
}

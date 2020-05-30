
#include <Wire.h>
#include "opt3001.h"

#define USE_USCI_B1
#define SCL_PIN      22
#define SDA_PIN      21

Opt3001 objOPT3001;

void setup()
{
  unsigned int readings = 0;
  
  Serial.begin(115200);
  delay(1000);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  objOPT3001.begin(DEFAULT_CONFIG_100); 
  
  Serial.println("OPT3001 Initialized----------------------------------");
  
  // get manufacturer ID from OPT3001. Default = 0101010001001001
  readings = objOPT3001.readManufacturerId();  
  Serial.print("Manufacturer ID: "); 
  Serial.println(readings, BIN);

  // get device ID from OPT3001. Default = 0011000000000001
  readings = objOPT3001.readDeviceId();  
  Serial.print("Device ID: "); 
  Serial.println(readings, BIN);
  
  // read config register from OPT3001. Default = 1100110000010000
  readings = objOPT3001.readConfigReg();  
  Serial.print("Configuration Register: "); 
  Serial.println(readings, BIN);

  // read Low Limit register from OPT3001. Default = 0000000000000000
  readings = objOPT3001.readLowLimitReg();  
  Serial.print("Low Limit Register: "); 
  Serial.println(readings, BIN);
  
  // read High Limit register from OPT3001. Default = 1011111111111111
  readings = objOPT3001.readHighLimitReg();  
  Serial.print("High Limit Register: "); 
  Serial.println(readings, BIN);    
  
  Serial.println("\nOPT3001 READINGS-------------------------------------");
}

void loop()
{
  uint32_t readings;
  readings = objOPT3001.readResult();  
  Serial.print("LUX Readings = ");
  Serial.println(readings, DEC);
  delay(10);
}

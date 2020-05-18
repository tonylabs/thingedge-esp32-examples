/* Two Wire I2C Scanner
 * Connect a 2.4k resistor between SDA and +3.3V
 * Connect a 2.4k resistor between SCL and +3.3V
*/
 
#include <Wire.h>

#define SDA1 21
#define SCL1 22

#define SDA2 5
#define SCL2 4

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

void scan1()
{
  Serial.println("Scanning I2C Addresses Channel 1");
  uint8_t cnt=0;
  for(uint8_t i=0;i<128;i++)
  {
    I2Cone.beginTransmission(i);
    uint8_t ec=I2Cone.endTransmission(true);
    if(ec==0)
    {
      if(i<16)Serial.print('0');
      Serial.print(i,HEX);
      cnt++;
    }
    else Serial.print("..");
    Serial.print(' ');
    if ((i&0x0f)==0x0f)Serial.println();
  }
  Serial.print("Scan Completed, ");
  Serial.print(cnt);
  Serial.println(" I2C Devices found.");
}

void scan2()
{
  Serial.println("Scanning I2C Addresses Channel 2");
  uint8_t cnt=0;
  for(uint8_t i=0;i<128;i++)
  {
    I2Ctwo.beginTransmission(i);
    uint8_t ec=I2Ctwo.endTransmission(true);
    if(ec==0)
    {
      if(i<16)Serial.print('0');
      Serial.print(i,HEX);
      cnt++;
    }
    else Serial.print("..");
    Serial.print(' ');
    if ((i&0x0f)==0x0f)Serial.println();
  }
  Serial.print("Scan Completed, ");
  Serial.print(cnt);
  Serial.println(" I2C Devices found.");
}

void setup()
{
  Serial.begin(115200);
  I2Cone.begin(SDA1, SCL1, 400000);
  I2Ctwo.begin(SDA2, SCL2, 400000);
}

void loop()
{
  scan1();
  Serial.println();
  delay(100);
  scan2();
  Serial.println();
  delay(5000);
}

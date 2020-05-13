#include <Wire.h>
#include "MMA7660.h"

MMA7660 objAccel;

void setup()
{
  objAccel.init();
  Serial.begin(115200);
}

void loop()
{
  static long cnt     = 0;
  static long cntout  = 0;
  float ax,ay,az;
  int8_t x, y, z;
  
  objAccel.getXYZ(&x,&y,&z);
  
  //Serial.print("x = ");
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(z);
  Serial.println();

  /*
  if(objAccel.getAcceleration(&ax,&ay,&az))
  {
    //Serial.print("get data ok: ");
  }
  else
  {
    //Serial.print("tiem out: ");
  }
  
  //Serial.print("AX ");
  Serial.print(ax);
  //Serial.println(" g");
  Serial.print(" ");
  Serial.print(ay);
  //Serial.println(" g");
  Serial.print(" ");
  Serial.print(az);
  //Serial.println(" g");
  Serial.println();
  */
  delay(50);
}

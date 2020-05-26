#include "ecm.h"

#define ECM_PIN 34

//@ECM
const float TARGET_FREQUENCY = 200;
const int N = 100;
const float ECM_THRESHOLD = 6000;
const float SAMPLING_FREQUENCY = 8900;

ECM objECM = ECM(TARGET_FREQUENCY, N, SAMPLING_FREQUENCY);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  objECM.getSample(ECM_PIN);
  float magnitude = objECM.detect();

  Serial.println(magnitude);
  
  if (magnitude > ECM_THRESHOLD)
  {
  
  }
  delay(100);
}

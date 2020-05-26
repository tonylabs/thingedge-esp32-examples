
#include "ecm.h"

float SAMPLING_FREQUENCY;
float TARGET_FREQUENCY;
int NN;
float COEFF;
float Q1;
float Q2;

int sampleData[MAXN];

ECM::ECM(float TARGET_FREQUENCY, float N)
{
  ECM(TARGET_FREQUENCY, N, 8900.0);
  //EMC(TARGET_FREQUENCY, N, 4400.0);
}

ECM::ECM(float TARGET_FREQUENCY, float N, float SAMPLING_FREQUENCY)
{
  SAMPLING_FREQUENCY=SAMPLING_FREQUENCY;	//on 16mhz, ~8928.57142857143, on 8mhz ~44444
  TARGET_FREQUENCY=TARGET_FREQUENCY; //should be integer of SAMPLING_RATE/N
  if (N > MAXN)
  {
    NN = MAXN;
  }
  else
  {
    NN=N;
  }
  float omega = (2.0 * PI * TARGET_FREQUENCY) / SAMPLING_FREQUENCY;
  COEFF = 2.0 * cos(omega);
  reset();
}

/* Call this routine before every "block" (size=N) of samples. */
void ECM::reset(void)
{
  Q2 = 0;
  Q1 = 0;
}

/* Call this routine for every sample. */
void ECM::process(int sample)
{
  float Q0;
  Q0 = COEFF * Q1 - Q2 + (float) (sample - ADCCENTER);
  Q2 = Q1;
  Q1 = Q0;
}

/* GET SOME SAMPLE */
void ECM::getSample(int sensorPin)
{
  for (int index = 0; index < NN; index++)
  {
    sampleData[index] = analogRead(sensorPin);
  }
}

float ECM::detect()
{
  float	magnitude;
  /* PROCESS THE SAMPLES */
  for (int index = 0; index < NN; index++)
  {
    process(sampleData[index]);
  }
  /* DO THE "STANDARD EMC" PROCESSING*/
  magnitude = sqrt(Q1*Q1 + Q2*Q2 - COEFF*Q1*Q2);
  reset();
  return magnitude;
}

#ifndef _EMC_H_
#define _EMC_H_

#include "Arduino.h"

#define MAXN 200
#define ADCCENTER 512

class ECM
{
  public:
    ECM(float, float, float);
    ECM(float,float);
	void getSample(int);
	float detect();
  private:
	  void process(int);
	  void reset(void);
};

#endif

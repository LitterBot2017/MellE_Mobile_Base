#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Arduino.h"

class IRSensor
{
    public:
        IRSensor(int pin);
        int getBinFullness();
        int getBinFullnessPercentage();
    private:
        int pinNum;
        float binFullnessMaxDistance = 125.0;
        float binFullnessMinDistance = 90.0;
};
#endif


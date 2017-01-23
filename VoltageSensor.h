#ifndef VOLTAGESENSOR_H
#define VOLTAGESENSOR_H

#include "Arduino.h"

class VoltageSensor
{
    public:
        VoltageSensor(int pin);
        float getBatteryVoltage();
        int getBatteryPercentage();
        int getSensorValue();
        int getBatteryIndex();
    private:
        int pinNum;
        int percentFull[21] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
        float voltages[21] = { 24.0, 24.12, 24.24, 24.36, 24.48, 24.60, 24.72, 24.84, 24.96, 25.08, 25.20, 25.32, 25.44, 25.56, 25.68, 25.80, 25.92, 26.04, 26.16, 26.28, 26.40 };
        int sensorValues[21] = { 859, 861, 864, 867, 870, 872, 875, 878, 881, 884, 886, 889, 892, 895, 897, 900, 903, 906, 909, 912, 914 };
};

#endif


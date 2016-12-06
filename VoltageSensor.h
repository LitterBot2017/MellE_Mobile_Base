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
        float voltages[21] = { 24.0, 24.2, 24.4, 24.6, 24.8, 25.0, 25.2, 25.4, 25.6, 25.8, 26.0, 26.2, 26.4, 26.6, 26.8, 27.0, 27.2, 27.4, 27.6, 27.8, 28.0 };
        int sensorValues[21] = { 859, 861, 864, 867, 870, 872, 875, 878, 881, 884, 886, 889, 892, 895, 897, 900, 903, 906, 909, 912, 914 };
};

#endif


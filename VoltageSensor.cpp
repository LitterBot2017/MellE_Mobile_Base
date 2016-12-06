#include "Arduino.h"
#include "VoltageSensor.h"

VoltageSensor::VoltageSensor(int pin) {
  this->pinNum = pin;
};

float VoltageSensor::getBatteryVoltage() {
  return this->voltages[this->getBatteryIndex()];
}

int VoltageSensor::getBatteryPercentage() {
  return this->percentFull[this->getBatteryIndex()];
}

int VoltageSensor::getBatteryIndex() {
  int sensorValue = this->getSensorValue();
  int index = 0;
  while (index < sizeof(this->sensorValues) && this->sensorValues[index] < sensorValue) {
    index++;
  }
  if (index <= 0){
    return 0;
  } else {
    return (index - 1);
  }
}

int VoltageSensor::getSensorValue() {
  return analogRead(this->pinNum);
}


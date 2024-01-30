#include "optical_intake_sensor.h"

OpticalIntakeSensor::OpticalIntakeSensor(int opticalSensorPort){
    this->opticalSensor = new OpticalSensor(opticalSensorPort);
}

bool OpticalIntakeSensor::isHoldingTriball(){
    return opticalSensor->getProximity()==255;
}

int OpticalIntakeSensor::getTriballRollbackPercentage(){
    return (opticalSensor->getProximity());
}
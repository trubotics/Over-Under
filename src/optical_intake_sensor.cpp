#include "optical_intake_sensor.h"

OpticalIntakeSensor::OpticalIntakeSensor(int opticalSensorPort){
    this->opticalSensor = new pros::Optical(opticalSensorPort);
}

bool OpticalIntakeSensor::isHoldingTriball(){
    return opticalSensor->get_proximity()==255;
}

double_t OpticalIntakeSensor::getTriballRollbackPercentage(){
    return ((255 - opticalSensor->get_proximity()) / 25500.0);
}
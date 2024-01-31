#include "distance_intake_sensor.h"

DistanceIntakeSensor::DistanceIntakeSensor(int distanceSensorPort){
    this->distanceSensor = new pros::Distance(distanceSensorPort);
}

bool DistanceIntakeSensor::isHoldingTriball(){
    return distanceSensor->get() < 100;
}

double_t DistanceIntakeSensor::getTriballRollbackPercentage(){
    // Min ~20mm to Max 100mm
    return (distanceSensor->get() - 20) / 80.0 * 100.0;
}
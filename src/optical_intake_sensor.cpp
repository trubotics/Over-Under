#include <optical_intake_sensor.h>

OpticalIntakeSensor::OpticalIntakeSensor(OpticalSensor *opticalSensor){
    this->opticalSensor = opticalSensor;
}

bool OpticalIntakeSensor::isHoldingTriball(){
    return opticalSensor->getProximity()==255;
}

int OpticalIntakeSensor::getTriballRollbackPercentage(){
    return (opticalSensor->getProximity()/255*100);
}
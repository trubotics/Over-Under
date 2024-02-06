#ifndef OPTICAL_INTAKE_SENSOR_H
#define OPTICAL_INTAKE_SENSOR_H

#include "main.h"
#include "intake_sensor.h"

/**
 * @brief the OpticalIntakeSensor class is a class that provides methods to sense if a triball is in possession using optical sensors
 */ 
class OpticalIntakeSensor : public IntakeSensor
{
public:
    /**
     * @brief Constructs an OpticalIntakeSensor object with the specified optical sensor port
     */
    OpticalIntakeSensor(int opticalSensorPort);

    /**
     * @return whether there is a triabll in the intake 
     */
    bool isHoldingTriball();

    /**
     * @return whether the triball is rolling back with a percentage on how far is has rolled back  
     */
    double_t getTriballRollbackPercentage();

private:
    OpticalSensor *opticalSensor;

};
#endif
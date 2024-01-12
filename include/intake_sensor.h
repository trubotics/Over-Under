#ifndef INTAKE_SENSOR_H
#define INTAKE_SENSOR_H

/**
 * @brief the IntakeSensor class is an interface that provides methods to sense if a triball is in possession
 */ 
class IntakeSensor 
{
public:
    /**
     * @return whether there is a triabll in the intake 
     */
    virtual bool isHoldingTriball() = 0;

    /**
     * @return whether the triball is rolling back with a percentage on how far is has rolled back  
     */
    virtual int getTriballRollbackPercentage() = 0;

};
#endif
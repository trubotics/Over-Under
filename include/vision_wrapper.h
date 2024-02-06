#ifndef VISION_H
#define VISION_H

#include "main.h"
#include "flywheel_stick.h"

/**
 * Wrapper class for the Vision sensor.
*/
class VisionWrapper {
public:
    VisionWrapper(int32_t port, FlywheelStick *flywheelStick);

    /**
     * Returns how much the robot should turn to face the triball.
    */
    int32_t getRotationToTriball();

private:
    pros::Vision sensor;
    pros::vision_signature_s_t triballSignature;
    FlywheelStick *flywheelStick;
};

#endif
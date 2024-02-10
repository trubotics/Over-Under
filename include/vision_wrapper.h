#ifndef VISION_H
#define VISION_H

#include "main.h"
#include "flywheel_stick.h"

const int VISION_FOV = 61; // degrees
const int VISION_WIDTH = 316; // pixels
const int CLOSEST_TRIBALL_WIDTH = 190; // pixels
const int MIN_TRIBALL_WIDTH = 15; // pixels

/**
 * Wrapper class for the Vision sensor.
*/
class VisionWrapper {
public:
    VisionWrapper(int32_t port, FlywheelStick *flywheelStick);

    /**
     * Returns how much the robot should turn to face the triball.
     * Returns numeric_limits<int32_t>::min() if no triball is found.
    */
    int32_t getRotationToTriball();

private:
    pros::Vision sensor;
    pros::vision_signature_s_t triballSignature;
    FlywheelStick *flywheelStick;
};

#endif
#include "vision_wrapper.h"

VisionWrapper::VisionWrapper(int32_t port, FlywheelStick *flywheelStick) : sensor(port, pros::E_VISION_ZERO_CENTER) {
    sensor.set_wifi_mode(1); //TODO: Change this to 0 when not testing
    sensor.set_exposure(50);
    triballSignature = sensor.signature_from_utility(1, -6951, -3001, -4976, -6241, -3153, -4697, 1.200, 0);
    sensor.set_signature(1, &triballSignature);

    this->flywheelStick = flywheelStick;
}

int32_t VisionWrapper::getRotationToTriball() {
    FlywheelStickState previousState = flywheelStick->getState();
    flywheelStick->rotateArm(FlywheelStickState::Vision, true);
    if (sensor.get_object_count() <= 0) {
        return numeric_limits<int32_t>::min();
    }
    pros::vision_object_s_t object = sensor.get_by_size(0);
    return object.x_middle_coord;
}

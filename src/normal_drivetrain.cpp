#include "normal_drivetrain.h"

NormalDrivetrain::NormalDrivetrain(
    int leftFrontMotorPort,
    int leftMiddleMotorPort,
    int leftBackMotorPort,
    int rightFrontMotorPort,
    int rightMiddleMotorPort,
    int rightBackMotorPort,
    bool reverseFront,
    bool reverseMiddle,
    bool reverseBack,
    pros::Imu *inertial
    ) : Drivetrain(inertial)
{

}
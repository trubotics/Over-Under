#include "drivetrain.h"

Drivetrain::Drivetrain(pros::Imu *inertial, IntakeSensor *intake)
{
    this->inertial = inertial;
    this->intakeSensor = intake;
}

void Drivetrain::driveToObject(int velocityPercent) {
    while (!intakeSensor->objectDetected()) {
        drive(velocityPercent / 100.0, 0);
        // TODO: Can implement PID controller here to drive more straight
    }
}

void Drivetrain::rotateBy(double angle)
{
    double target = this->inertial->get_rotation() + angle;
    while (fabs(this->inertial->get_rotation() - target) > 1) {
        this->drive(0, (target - this->inertial->get_rotation()) * 0.01);
        pros::delay(5);
    }

    this->stop();
}

void Drivetrain::rotateTo(double heading)
{
    double diff = heading - this->inertial->get_heading();
    
    // Normalize the difference to be in the range of -180 to 180
    if (diff > 180) {
        diff -= 360;
    } else if (diff < -180) {
        diff += 360;
    }
    
    // Determine the direction based on the normalized difference
    int direction = diff > 0 ? 1 : -1;
    int angle = diff > 0 ? diff : -diff;

    rotateBy(angle * direction);
}
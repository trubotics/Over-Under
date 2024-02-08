#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "main.h"

/// @file
/// @brief Contains the declaration of the Drivetrain class.

/// @class Drivetrain
/// @brief A base drivetrain class
class Drivetrain
{
public:
    Drivetrain(pros::Imu *inertial);

    /// @brief Drives the robot with the given parameters
    /// @param velocity The velocity to drive at
    /// @param rotation The rotation to drive at
    virtual void drive(double velocity, double rotation) = 0;

    /// @brief Rotates the robot by the given angle
    void rotateBy(double angle);

    /// @brief Rotates the robot to the given heading
    void rotateTo(double heading);

    /// @brief Stops the robot
    virtual void stop() = 0;

    /// @brief Gets the drivetrain's velocity
    /// @return The drivetrain's velocity
    virtual double getVelocity() = 0;

private:
    pros::Imu *inertial;
};

#endif
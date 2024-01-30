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
    /// @brief Drives the robot with the given parameters
    /// @param velocity The velocity to drive at
    /// @param rotation The rotation to drive at
    virtual void drive(double velocity, double rotation) = 0;

    /// @brief Stops the robot
    virtual void stop() = 0;

    /// @brief Gets the drivetrain's velocity
    /// @return The drivetrain's velocity
    virtual double getVelocity() = 0;
};

#endif
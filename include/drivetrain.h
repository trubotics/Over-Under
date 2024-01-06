#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "main.h"

/// @file
/// @brief Contains the declaration of the Drivetrain class.

/// @class Drivetrain
/// @brief A drivetrain class that wraps around the okapi ChassisController class.
class Drivetrain
{
public:
    /// @brief Creates a new Drivetrain object with the given parameters
    /// @param ratio The gearset ratio pair to use
    /// @param leftFrontMotorPort The port number of the left front motor
    /// @param leftMiddleMotorPort The port number of the left middle motor
    /// @param leftBackMotorPort The port number of the left back motor
    /// @param rightFrontMotorPort The port number of the right front motor
    /// @param rightMiddleMotorPort The port number of the right middle motor
    /// @param rightBackMotorPort The port number of the right back motor
    /// @param reverseFront The direction of the front motors
    /// @param reverseMiddle The direction of the middle motors
    /// @param reverseBack The direction of the back motors
    Drivetrain(
        AbstractMotor::GearsetRatioPair ratio,
        int leftFrontMotorPort,
        int leftMiddleMotorPort,
        int leftBackMotorPort,
        int rightFrontMotorPort,
        int rightMiddleMotorPort,
        int rightBackMotorPort,
        bool reverseFront,
        bool reverseMiddle,
        bool reverseBack);

    /// @brief Drives the robot with the given parameters
    /// @param velocity The velocity to drive at
    /// @param rotation The rotation to drive at
    void drive(double velocity, double rotation);

    /// @brief Stops the robot
    void stop();

    /// @brief Gets the drivetrain's velocity
    /// @return The drivetrain's velocity
    double getVelocity();
private:
    shared_ptr<ChassisController> chassis; ///< The chassis controller used for controlling the drivetrain.
    double velocity; ///< The velocity of the drivetrain.
};

#endif
#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "main.h"
#include "intake_sensor.h"

/// @file
/// @brief Contains the declaration of the Drivetrain class.

const vector<double> DEFAULT_GAINS = {0.01, 0, 0};

/// @class Drivetrain
/// @brief A base drivetrain class
class Drivetrain
{
public:
    Drivetrain(pros::Imu *inertial, IntakeSensor *intake);

    /// @brief Drives the robot with the given parameters
    /// @param velocity The velocity to drive at (0-1)
    /// @param rotation The rotation to drive at (0-1)
    virtual void drive(double velocity, double rotation) = 0;

    /// @brief Drives the robot straight using PID and the inertial sensor
    /// @param velocityPercent The velocity to drive at in % (0-100)
    /// @param gains The gains to use for the PID controller
    /// @return The task that is created to drive straight
    pros::Task driveStraight(int velocityPercent = 25, vector<double> gains = DEFAULT_GAINS);

    /// @brief Drives the robot straight for a set distance
    /// @param distance The distance to drive for in inches
    /// @param velocityPercent The velocity to drive at in % (0-100)
    virtual void driveFor(double distance, int velocityPercent = 25) = 0;

    /// @brief Drives the robot straight until an object is in front of it
    /// @param velocityPercent 
    void driveToObject(int velocityPercent = 25);

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
    IntakeSensor *intakeSensor;
};

#endif
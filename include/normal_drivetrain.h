#ifndef NORMALDRIVETRAIN_H
#define NORMALDRIVETRAIN_H

#include "main.h"
#include "drivetrain.h"
#include "intake_sensor.h"

/**
 * @brief the NormalDrivetrain class is a class that provides methods to control the drivetrain using the V5 motors
 */
class NormalDrivetrain : public Drivetrain
{
public:
    /**
     * @brief Constructs a NormalDrivetrain object with the specified motor ports and gearset
     */
    NormalDrivetrain(
        int leftFrontMotorPort,
        int leftMiddleMotorPort,
        int leftBackMotorPort,
        int rightFrontMotorPort,
        int rightMiddleMotorPort,
        int rightBackMotorPort,
        bool reverseFront,
        bool reverseMiddle,
        bool reverseBack,
        pros::Imu *inertial, IntakeSensor *intake);

    /**
     * @brief Drives the robot with the given parameters
     * @param velocity The velocity to drive at
     * @param rotation The rotation to drive at
     */
    void drive(double velocity, double rotation);

    /**
     * @brief Stops the robot
     */
    void stop();

    /**
     * @brief Gets the drivetrain's velocity
     * @return The drivetrain's velocity
     */
    double getVelocity();
private:
    double velocity; ///< The velocity of the drivetrain.
};

#endif
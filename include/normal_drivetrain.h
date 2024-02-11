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
        pros::motor_gearset_e_t gearset,
        int16_t degreesPerInch,
        int8_t leftFrontMotorPort,
        int8_t leftMiddleMotorPort,
        int8_t leftBackMotorPort,
        int8_t rightFrontMotorPort,
        int8_t rightMiddleMotorPort,
        int8_t rightBackMotorPort,
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
     * @brief Drives the robot with the given parameters using raw voltage values
     * @param voltagePercent The voltage to drive at (-100 => 100)
     * @param rotation The rotation to drive at (-100 => 100)
    */
    void voltageDrive(double voltagePercent, double rotation);

    /**
     * @brief Drives the robot for a set distance
     * @param distance The distance to drive for in inches
     * @param velocityPercent The velocity to drive at in % (0-100)
     */
    void driveFor(double distance, int velocityPercent);

    /**
     * @brief Stops the robot
     */
    void stop();

    /**
     * @brief Gets the drivetrain's velocity
     * @return The drivetrain's velocity
     */
    double getVelocity();

    /**
     * @brief Sets the brake mode of the drivetrain
     * @param mode The brake mode to set
     */
    void setBrakeMode(pros::motor_brake_mode_e_t mode);
private:
    int16_t degreesPerInch; // The number of degrees per inch for the drivetrain.
    double velocity; // The velocity of the drivetrain.
    double maxVelocity; // The maximum velocity of the drivetrain.

    pros::MotorGroup leftMotors; // The left motors of the drivetrain.
    pros::MotorGroup rightMotors; // The right motors of the drivetrain.
};

#endif
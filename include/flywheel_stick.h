#ifndef FLYWHEEL_STICK_H
#define FLYWHEEL_STICK_H

#include "main.h"
#include "drivetrain.h"
#include "intake_sensor.h"

const double ROLLBACK_PROPORTIONALITY = 2;

/**
 * @brief The FlywheelStickState enum represents the possible states of the FlywheelStick object.
 */
enum FlywheelStickState
{
    Intake,
    Flywheel,
    Block
};

/**
 * @brief The flywheelStickStateData struct represents the data for a given state of the FlywheelStick object.
 */
struct flywheelStickStateData
{
    int armMotorPosition;      // The absolute position of the arm motor in degrees for a given state.
    int flywheelMotorVelocity; // The velocity of the flywheel motor for a given state.
};

const int INTAKE_TIMEOUT = 1000;

/**
 * @brief The FlywheelStick class represents a controller for the flywheel on a stick.
 *
 * This class provides methods to control the rotation of the arm motor and the spinning of the flywheel motor.
 */
class FlywheelStick
{
public:
    /**
     * @brief Constructs a FlywheelStick object with the specified arm motor port and flywheel motor port.
     *
     * @param armMotorPort The port number of the arm motor.
     * @param armReversed Whether the arm motor is reversed.
     * @param flywheelMotorPort The port number of the flywheel motor.
     * @param flywheelReversed Whether the flywheel motor is reversed.
     */
    FlywheelStick(uint8_t armMotorPort, bool armReversed, uint8_t flywheelMotorPort, bool flywheelReversed, IntakeSensor *intakeSensor, Drivetrain *drivetrain);

    /**
     * @brief Enables or disables rollback prevention.
     *
     * @param enabled Whether rollback prevention should be enabled.
     */
    void enableRollback(bool enabled);

    /**
     * @brief Toggles rollback prevention.
     */
    void toggleRollback();

    /**
     * @brief Returns the current state of the FlywheelStick object.
     *
     * @return The current state of the FlywheelStick object.
     */
    FlywheelStickState getState();

    /**
     * @brief Rotates the arm motor to the specified state.
     *
     * @param state The target positional state of the FlywheelStick object.
     */
    void rotateArm(FlywheelStickState state);

    /**
     * @brief Spins the flywheel motor at a velocity suitable for the current state.
     *
     * @param reverse Whether the flywheel motor should spin in reverse.
     */
    void spinFlywheel(bool reverse);

    /**
     * @brief Stops the flywheel motor.
     */
    void stopFlywheel();

    /**
     * @brief Intake or eject a triball.
     *
     * This method is blocking and will return when the operation is complete or the timeout is reached.
     * Run this method in a new task to prevent blocking.
     *
     * @return Whether the operation was successful.
     */
    bool intakeOrEject();

private:
    static const std::unordered_map<FlywheelStickState, flywheelStickStateData> stateDataMap; // The data for each state of the FlywheelStick object.

    FlywheelStickState state;          // The current state of the FlywheelStick object.
    Motor *armMotor;                   // The motor used to control the rotation of the arm.
    Motor *flywheelMotor;              // The motor used to control the spinning of the flywheel.
    IntakeSensor *intakeSensor; // The sensor used to detect whether a triball is loaded.
    Drivetrain *drivetrain;            // The drivetrain used for anti-rollback.
    tuple<bool, bool> rollbackEnabled; // The two conditions that must be met for rollback prevention to be enabled. (explicitly set, and whether currently intaking)

    void rollbackPreventionTask(); // The task that prevents the triball from rolling out.
};

#endif // FLYWHEEL_STICK_H

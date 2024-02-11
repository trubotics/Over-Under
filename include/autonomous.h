#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include "main.h"
#include "drivetrain.h"
#include "flywheel_stick.h"
#include "distance_intake_sensor.h"
#include "wings.h"
#include "vision_wrapper.h"

enum class AutonomousScheme
{
    HELLO_WORLD,
    TEST,
    DEFENSE_SIDE,
    OFFSENSE_SIDE,
};

/// @brief A class that figures out what to do during autonomous
class AutonomousManager
{
private:
    Drivetrain *drivetrain;
    FlywheelStick *flywheelStick;
    DistanceIntakeSensor *intakeSensor;
    Wings *wings;
    VisionWrapper *vision;

    AutonomousScheme scheme;
    unordered_map<string, function<void(Drivetrain*, FlywheelStick*, IntakeSensor*, Wings*, VisionWrapper*)>> autonomousSchemes;
public:
    AutonomousManager(Drivetrain *drivetrain, FlywheelStick *flywheelStick, DistanceIntakeSensor *intakeSensor, Wings *wings, VisionWrapper *vision);
    /// @brief Initializes the autonomous manager with a default scheme
    void initialize(AutonomousScheme scheme);

    /// @brief Registers buttons to autonomous schemes
    void registerSchemes();
    
    /// @brief Runs the current autonomous scheme
    void run();

    /// @brief Runs the given autonomous scheme
    /// @param scheme The scheme to run
    void run(AutonomousScheme scheme);
};


#endif
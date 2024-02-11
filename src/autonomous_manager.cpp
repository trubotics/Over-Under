#include "autonomous.h"

AutonomousManager::AutonomousManager(Drivetrain *drivetrain, FlywheelStick *flywheelStick, DistanceIntakeSensor *intakeSensor, Wings *wings, VisionWrapper *vision)
{
    this->drivetrain = drivetrain;
    this->flywheelStick = flywheelStick;
    this->intakeSensor = intakeSensor;
    this->wings = wings;
    this->vision = vision;
}

void AutonomousManager::initialize(AutonomousScheme scheme)
{
    this->scheme = scheme;
}

void AutonomousManager::registerSchemes()
{
    pros::lcd::print(7, "<-- DEFENSE | idk | OFFENSE ->>");

    while (true)
    {
        uint8_t buttonsPressed = pros::lcd::read_buttons();
        switch (buttonsPressed)
        {
        case 100:
            initialize(AutonomousScheme::DEFENSE_SIDE);
            break;

        case 1:
            initialize(AutonomousScheme::OFFSENSE_SIDE);
            break;
        
        default:
            break;
        }

        pros::delay(250);
    }
}

void AutonomousManager::run()
{
    run(this->scheme);
}

void AutonomousManager::run(AutonomousScheme scheme)
{
    switch (scheme)
    {
    case AutonomousScheme::HELLO_WORLD:
        pros::lcd::print(0, "Hello World");
        break;
    case AutonomousScheme::TEST:
        // Just some testing auton code for now
        double triballAngle;
        do
        {
            triballAngle = vision->getRotationToTriball();
        } while (triballAngle == numeric_limits<int32_t>::min());

        drivetrain->rotateBy(triballAngle);

        drivetrain->driveToObject();
        // Just in case we rotated a bit
        do
        {
            triballAngle = vision->getRotationToTriball();
        } while (triballAngle == numeric_limits<int32_t>::min());
        drivetrain->rotateBy(triballAngle);

        // Intake triball
        drivetrain->drive(0.25, 0);
        flywheelStick->intakeOrEject();
        drivetrain->stop();

        // Drive back
        drivetrain->rotateTo(0);
        drivetrain->driveFor(-10);
    case AutonomousScheme::DEFENSE_SIDE:
        if (intakeSensor->isHoldingTriball()) // Started with preload
        {
            // Score preload
            // Drive to goal
            drivetrain->driveFor(12 * 6, 50); // Now beside goal
            drivetrain->rotateTo(270); // Now in front of goal
            // Drive into goal (alignment point)
            pros::Task goalDriveTask = drivetrain->pidDrive();
            pros::delay(1000); // Arbitrary amount
            goalDriveTask.notify();
            goalDriveTask.join(); // Now touching goal with intake wheel
            drivetrain->driveFor(-0.5); // Backup a tiny bit
            // Eject triball
            flywheelStick->intakeOrEject(); // No longer holding triball

            // Grab matchload
            // Backup & rotate
            drivetrain->driveFor(-5); // No longer touching goal
            drivetrain->rotateTo(180); // Now facing "spawn" location
            // Travel to matchload area
            drivetrain->driveToObject(50); // Now in front of "spawn wall"
            // Rotate towards matchload triball
            drivetrain->rotateBy(75); // Now facing in general direction of triball
            drivetrain->rotateBy(vision->getRotationToTriball()); // Now facing directly at triball
            break;
            // Go towards triball
            drivetrain->driveToObject(); // Now in front of triball
            drivetrain->rotateBy(vision->getRotationToTriball()); // Now facing directly at triball (accurately)
            // Intake triball
            pros::Task matchloadDriveTask = drivetrain->pidDrive();
            flywheelStick->intakeOrEject();
            matchloadDriveTask.notify();
            matchloadDriveTask.join(); // Now holding triball
            // Backup
            drivetrain->driveFor(-5); // Triball taked
        } else // Started without preload
        {

        }
        break;
    case AutonomousScheme::OFFSENSE_SIDE:
        if (intakeSensor->isHoldingTriball()) // Started with preload
        {
            // Score preload
            // Drive to goal
            drivetrain->driveFor(12 * 6, 50); // Now beside goal
            drivetrain->rotateTo(90); // Now in front of goal
            // Drive into goal (alignment point)
            pros::Task goalDriveTask = drivetrain->pidDrive();
            pros::delay(1000); // Arbitrary amount
            goalDriveTask.notify();
            goalDriveTask.join(); // Now touching goal with intake wheel
            drivetrain->driveFor(-0.5); // Backup a tiny bit
            // Eject triball
            flywheelStick->intakeOrEject(); // No longer holding triball

            // Grab field triball
            // Backup & rotate to second triball
            drivetrain->driveFor(-5); // No longer touching goal
            drivetrain->rotateTo(220); // Now facing towards another triball
            drivetrain->driveToObject(); // Now in front of triball
            drivetrain->rotateBy(vision->getRotationToTriball()); // Now facing directly at triball (accurately)
            // Intake triball
            pros::Task fieldTriballDriveTask = drivetrain->pidDrive();
            flywheelStick->intakeOrEject();
            fieldTriballDriveTask.notify();
            fieldTriballDriveTask.join(); // Now holding triball

            // Score triball
            // Face goal
            drivetrain->driveFor(-5); // Backup a little
            drivetrain->rotateTo(90); // Now facing goal
            // Drive to goal
            drivetrain->driveFor(12); // Now in front of goal
            goalDriveTask = drivetrain->pidDrive();
            pros::delay(1000); // Arbitrary amount
            goalDriveTask.notify();
            goalDriveTask.join(); // Now touching goal with intake wheel
            drivetrain->driveFor(-0.5); // Backup a tiny bit
            // Eject triball
            flywheelStick->intakeOrEject(); // No longer holding triball
            
            // Grab matchload
            // Backup & rotate
            drivetrain->driveFor(-5); // No longer touching goal
            drivetrain->rotateTo(180); // Now facing "spawn" location
            // Travel to matchload area
            drivetrain->driveToObject(50); // Now in front of "spawn wall"
            // Rotate towards matchload triball
            drivetrain->rotateBy(-75); // Now facing in general direction of triball
            drivetrain->rotateBy(vision->getRotationToTriball()); // Now facing directly at triball
            break;
            // Go towards triball
            drivetrain->driveToObject(); // Now in front of triball
            drivetrain->rotateBy(vision->getRotationToTriball()); // Now facing directly at triball (accurately)
            // Intake triball
            pros::Task matchloadDriveTask = drivetrain->pidDrive();
            flywheelStick->intakeOrEject();
            fieldTriballDriveTask.notify();
            fieldTriballDriveTask.join(); // Now holding triball
        } else // Started without preload
        {

        }
        break;
    }
}

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
         drivetrain->driveFor(-12);
}
}

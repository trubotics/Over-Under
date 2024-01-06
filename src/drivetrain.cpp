#include "drivetrain.h"

Drivetrain::Drivetrain(
    AbstractMotor::GearsetRatioPair ratio,
    int leftFrontMotorPort,
    int leftMiddleMotorPort,
    int leftBackMotorPort,
    int rightFrontMotorPort,
    int rightMiddleMotorPort,
    int rightBackMotorPort,
    bool reverseFront,
    bool reverseMiddle,
    bool reverseBack)
{
    AbstractMotor::gearset gearset = ratio.internalGearset;

    Motor leftFrontMotor = Motor(leftFrontMotorPort, reverseFront, gearset, AbstractMotor::encoderUnits::degrees);
    Motor leftMiddleMotor = Motor(leftMiddleMotorPort, reverseMiddle, gearset, AbstractMotor::encoderUnits::degrees);
    Motor leftBackMotor = Motor(leftBackMotorPort, reverseBack, gearset, AbstractMotor::encoderUnits::degrees);
    Motor rightFrontMotor = Motor(rightFrontMotorPort, !reverseFront, gearset, AbstractMotor::encoderUnits::degrees);
    Motor rightMiddleMotor = Motor(rightMiddleMotorPort, !reverseMiddle, gearset, AbstractMotor::encoderUnits::degrees);
    Motor rightBackMotor = Motor(rightBackMotorPort, !reverseBack, gearset, AbstractMotor::encoderUnits::degrees);

    MotorGroup leftMotors = MotorGroup({leftFrontMotor, leftMiddleMotor, leftBackMotor});
    MotorGroup rightMotors = MotorGroup({rightFrontMotor, rightMiddleMotor, rightBackMotor});

    chassis = ChassisControllerBuilder()
                  .withMotors(leftMotors, rightMotors)
                  .withDimensions(ratio, {{4_in, 12_in}, gearsetToTPR(gearset)})
                  .withMaxVelocity(50)
                  .withLogger(
                      std::make_shared<Logger>(
                          TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
                          "/ser/sout",                                 // Output to the PROS terminal
                          Logger::LogLevel::debug                      // Most verbose log level
                          ))
                  .build();
}

void Drivetrain::drive(double velocity, double rotation)
{
    chassis->getModel()->arcade(velocity, rotation);
}

void Drivetrain::stop()
{
    chassis->getModel()->stop();
}

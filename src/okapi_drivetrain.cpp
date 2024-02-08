#include "okapi_drivetrain.h"
#include "intake_sensor.h"

using namespace okapi;

OkapiDrivetrain::OkapiDrivetrain(
    AbstractMotor::GearsetRatioPair ratio,
    int leftFrontMotorPort,
    int leftMiddleMotorPort,
    int leftBackMotorPort,
    int rightFrontMotorPort,
    int rightMiddleMotorPort,
    int rightBackMotorPort,
    bool reverseFront,
    bool reverseMiddle,
    bool reverseBack, 
    pros::Imu *inertial, IntakeSensor *intake) : Drivetrain(inertial, intake)
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

    velocity = 0;
}

void OkapiDrivetrain::drive(double velocity, double rotation)
{
    chassis->getModel()->arcade(velocity, rotation);
    this->velocity = velocity * chassis->getModel()->getMaxVelocity();
}

void OkapiDrivetrain::driveFor(double distance, int velocityPercent)
{
    double maxVel = chassis->getMaxVelocity();
    chassis->setMaxVelocity(maxVel * velocity / 100);
    chassis->moveDistance(distance * 1_in);
    chassis->setMaxVelocity(maxVel);
}

void OkapiDrivetrain::stop()
{
    chassis->getModel()->stop();
}

double OkapiDrivetrain::getVelocity()
{
    double velocity = this->velocity;
    return velocity;
}
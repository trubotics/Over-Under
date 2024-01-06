#include "flywheel_stick.h"

const std::unordered_map<FlywheelStickState, flywheelStickStateData> FLYWHEEL_STICK_STATE_DATA = {
    {FlywheelStickState::Intake, {.armMotorPosition = 0, .flywheelMotorVelocity = 200}},
    {FlywheelStickState::Flywheel, {.armMotorPosition = 90, .flywheelMotorVelocity = 600}},
    {FlywheelStickState::Block, {.armMotorPosition = 135, .flywheelMotorVelocity = 0}}};

FlywheelStick::FlywheelStick(uint8_t armMotorPort, bool armReversed, uint8_t flywheelMotorPort, bool flywheelReversed, OpticalSensor *opticalSensor)
{
    this->opticalSensor = opticalSensor;
    armMotor = new Motor(armMotorPort, armReversed, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
    flywheelMotor = new Motor(flywheelMotorPort, flywheelReversed, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    rollbackEnabled = make_tuple(false, true);

    armMotor->tarePosition();

    // pros::Task rollbackPrevention(rollbackPreventionTask, "Rollback Prevention");
    pros::Task rbt([this] {this->rollbackPreventionTask();}, "Rollback Prevention");
}

void FlywheelStick::enableRollback(bool enabled)
{
    get<0>(rollbackEnabled) = enabled;
}

void FlywheelStick::rollbackPreventionTask()
{
    while (true)
    {
        if (!get<0>(rollbackEnabled) || !get<1>(rollbackEnabled) || state != FlywheelStickState::Intake)
        {
            pros::delay(100); // SUPER DUPER IMPORTANT
            continue;
        }

        // All conditions met, prevent rollback
        // @todo: Use PID on optical <-> flywheel
        if (opticalSensor->getProximity() > 100 && opticalSensor->getProximity() < 250)
        {
            flywheelMotor->moveVelocity(-200);
        }
        else
        {
            flywheelMotor->moveVelocity(0);
        }
        pros::delay(10);
    }
}

void FlywheelStick::rotateArm(FlywheelStickState state)
{
    flywheelMotor->moveAbsolute(state, 100);
}

bool FlywheelStick::intakeOrEject()
{
    if (state != FlywheelStickState::Intake)
    {
        return false;
    }

    // Block rollback prevention
    get<1>(rollbackEnabled) = false;

    // Get current state
    bool loaded = isLoaded();

    int velocity = stateDataMap.at(state).flywheelMotorVelocity;
    if (loaded)
    {
        velocity = -velocity;
    }

    flywheelMotor->moveVelocity(velocity);
    int timeoutRemaining = INTAKE_TIMEOUT;
    bool success = false;
    while (timeoutRemaining > 0)
    {
        if (loaded && !isLoaded())
        {
            success = true;
            break;
        }
        pros::delay(10);
        timeoutRemaining -= 10;
    }
    flywheelMotor->moveVelocity(0);
    get<1>(rollbackEnabled) = true;

    return success;
}

bool FlywheelStick::isLoaded()
{
    return opticalSensor->getProximity() > 100 && opticalSensor->getProximity() < 250;
}
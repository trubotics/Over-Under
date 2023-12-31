#include "flywheel_stick.h"

const std::unordered_map<FlywheelStickState, flywheelStickStateData> FLYWHEEL_STICK_STATE_DATA = {
    {FlywheelStickState::Intake, {.armMotorPosition = 0, .flywheelMotorVelocity = 200}},
    {FlywheelStickState::Flywheel, {.armMotorPosition = 75*7, .flywheelMotorVelocity = 600}},
    {FlywheelStickState::Block, {.armMotorPosition = 120*7, .flywheelMotorVelocity = 0}}};

FlywheelStick::FlywheelStick(uint8_t armMotorPort, bool armReversed, uint8_t flywheelMotorPort, bool flywheelReversed, OpticalSensor *opticalSensor, Drivetrain *drivetrain)
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

void FlywheelStick::toggleRollback()
{
    enableRollback(!get<0>(rollbackEnabled));
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
        if (opticalSensor->getProximity() > 40)
        {
            // double error = (255 - opticalSensor->getProximity())/255.0;
            // int velocity = -200.0 * error * ROLLBACK_PROPORTIONALITY; // @todo adjust this
            // flywheelMotor->moveVelocity(velocity);
            // pros::lcd::print(0, "Rollback Prevention: %d", velocity);

            // Other approach, match drivetrain's negative/backwards velocity (clamp)
            double currentVel = drivetrain->getVelocity();
            // Flywheel runs on 7:3 ratio, so multiply by 7/3
            double velocity = min(currentVel, 0.0) * 7.0 / 3.0;
            flywheelMotor->moveVelocity(velocity);
        }
        else
        {
            flywheelMotor->moveVelocity(0);
        }
        pros::delay(10);
    }
}

FlywheelStickState FlywheelStick::getState()
{
    return state;
}

void FlywheelStick::rotateArm(FlywheelStickState state)
{
    int rotation = FLYWHEEL_STICK_STATE_DATA.at(state).armMotorPosition;
    armMotor->moveAbsolute(rotation, 50);
    this->state = state;
}

void FlywheelStick::spinFlywheel(bool reverse)
{
    // Stop rollback prevention
    get<1>(rollbackEnabled) = false;
    int velocity = FLYWHEEL_STICK_STATE_DATA.at(state).flywheelMotorVelocity;
    if (reverse)
    {
        velocity = -velocity;
    }
    flywheelMotor->moveVelocity(velocity);
}

void FlywheelStick::stopFlywheel()
{
    flywheelMotor->moveVelocity(0);
    get<1>(rollbackEnabled) = true;
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
    return opticalSensor->getProximity() > 50;
}
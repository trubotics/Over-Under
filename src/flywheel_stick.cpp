#include "flywheel_stick.h"

const std::unordered_map<FlywheelStickState, flywheelStickStateData> FLYWHEEL_STICK_STATE_DATA = {
    {FlywheelStickState::Intake, {.armMotorPosition = 0, .flywheelMotorVelocity = 200}},
    {FlywheelStickState::Eject, {.armMotorPosition = 15*7, .flywheelMotorVelocity = 300}},
    {FlywheelStickState::Vision, {.armMotorPosition = 50*7, .flywheelMotorVelocity = 0}},
    {FlywheelStickState::Flywheel, {.armMotorPosition = 75*7, .flywheelMotorVelocity = 600}},
    {FlywheelStickState::Block, {.armMotorPosition = 120*7, .flywheelMotorVelocity = 0}}};

FlywheelStick::FlywheelStick(uint8_t armMotorPort, bool armReversed, uint8_t flywheelMotorPort, bool flywheelReversed, IntakeSensor *intakeSensor, Drivetrain *drivetrain)
{
    this->intakeSensor = intakeSensor;
    armMotor = new okapi::Motor(armMotorPort, armReversed, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
    flywheelMotor = new okapi::Motor(flywheelMotorPort, flywheelReversed, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
    rollbackEnabled = make_tuple(false, true);
    state = FlywheelStickState::Intake;

    armMotor->tarePosition();

    // pros::Task rollbackPrevention(rollbackPreventionTask, "Rollback Prevention");
    // pros::Task rbt([this] {this->rollbackPreventionTask();}, "Rollback Prevention");
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
        if (intakeSensor->getTriballRollbackPercentage() > 16)
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

void FlywheelStick::rotateArm(FlywheelStickState state, bool blocking)
{
    int rotation = FLYWHEEL_STICK_STATE_DATA.at(state).armMotorPosition;
    armMotor->moveAbsolute(rotation, 50);

    if (blocking)
    {
        while (armMotor->getPosition() < rotation - 5 || armMotor->getPosition() > rotation + 5)
        {
            pros::delay(10);
        }
    }
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
    bool loaded = intakeSensor->isHoldingTriball();

    if (loaded)
    {
        this->rotateArm(FlywheelStickState::Eject, true);
    }
    else
    {
        this->rotateArm(FlywheelStickState::Intake, true);
    }
    int velocity = FLYWHEEL_STICK_STATE_DATA.at(state).flywheelMotorVelocity * (loaded ? 1 : -1);

    flywheelMotor->moveVelocity(velocity);
    int timeoutRemaining = INTAKE_TIMEOUT;
    bool success = false;
    while (timeoutRemaining > 0)
    {
        if (loaded && !intakeSensor->isHoldingTriball())
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
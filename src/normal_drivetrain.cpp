#include "normal_drivetrain.h"

NormalDrivetrain::NormalDrivetrain(
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
    pros::Imu *inertial,
    IntakeSensor *intake) : Drivetrain(inertial, intake),
                            leftMotors({static_cast<int8_t>(leftFrontMotorPort * (!reverseFront * 2 - 1)),
                                        static_cast<int8_t>(leftMiddleMotorPort * (!reverseMiddle * 2 - 1)),
                                        static_cast<int8_t>(leftBackMotorPort * (!reverseBack * 2 - 1))}
                                       ),
                            rightMotors({static_cast<int8_t>(rightFrontMotorPort * (reverseFront * 2 - 1)),
                                         static_cast<int8_t>(rightMiddleMotorPort * (reverseMiddle * 2 - 1)),
                                         static_cast<int8_t>(rightBackMotorPort * (reverseBack * 2 - 1))}
                                        )
{
    this->degreesPerInch = degreesPerInch;
    this->velocity = 0;

    leftMotors.set_gearing(gearset);
    rightMotors.set_gearing(gearset);

    switch (gearset)
    {
    case pros::E_MOTOR_GEARSET_18:
        maxVelocity = 200;
        break;
    case pros::E_MOTOR_GEARSET_06:
        maxVelocity = 600;
        break;
    case pros::E_MOTOR_GEARSET_36:
        maxVelocity = 100;
        break;
    default:
        maxVelocity = 200;
        break;
    }
}

void NormalDrivetrain::drive(double velocity, double rotation)
{
    double actualVelocity = velocity * maxVelocity;
    leftMotors.move_velocity(actualVelocity + rotation);
    rightMotors.move_velocity(actualVelocity - rotation);
    this->velocity = actualVelocity;
}

void NormalDrivetrain::voltageDrive(double voltagePercent, double rotation)
{
    double voltage = voltagePercent * 12000 / 100;
    double rotationVoltage = rotation * 12000 / 100;
    leftMotors.move_voltage(voltage + rotationVoltage);
    rightMotors.move_voltage(voltage - rotationVoltage);
    this->velocity = voltagePercent * maxVelocity / 100.0;
}

void NormalDrivetrain::driveFor(double distance, int velocityPercent)
{
    // Use encoders to drive for a set distance
    leftMotors.tare_position();
    rightMotors.tare_position();

    if (distance < 0) velocityPercent *= -1;

    pros::Task driveTask = pidDrive(velocityPercent);
    double inchesTravelled = 0;

    do {
        // Get each motor position
        vector<double> leftPositions = leftMotors.get_positions();
        vector<double> rightPositions = rightMotors.get_positions();

        vector<double> positions;
        positions.insert(positions.end(), leftPositions.begin(), leftPositions.end());
        positions.insert(positions.end(), rightPositions.begin(), rightPositions.end());

        // Check for outliars via standard deviation
        vector<int> outliarIndexes = {};
        double sum = 0;
        for (int i = 0; i < positions.size(); i++)
        {
            sum += positions[i];
        }
        double mean = sum / positions.size();

        double sumOfSquares = 0;
        for (int i = 0; i < positions.size(); i++)
        {
            sumOfSquares += pow(positions[i] - mean, 2);
        }
        double variance = sumOfSquares / positions.size();
        double standardDeviation = sqrt(variance);

        for (int i = 0; i < positions.size(); i++)
        {
            if (abs(positions[i] - mean) > 2 * standardDeviation)
            {
                outliarIndexes.push_back(i);
            }
        }

        // Remove outliars
        positions.erase(std::remove_if(positions.begin(), positions.end(), [&](int i) {
            return std::find(outliarIndexes.begin(), outliarIndexes.end(), i) != outliarIndexes.end();
        }));

        // Calculate the new mean
        sum = 0;
        for (int i = 0; i < positions.size(); i++)
        {
            sum += positions[i];
        }
        mean = sum / positions.size();

        // Calculate inches based on degree rotation
        inchesTravelled = mean / degreesPerInch;
    } while (inchesTravelled < distance);

    driveTask.notify();
    driveTask.join();
}

void NormalDrivetrain::stop()
{
    leftMotors.brake();
    rightMotors.brake();
    this->velocity = 0;
}

double NormalDrivetrain::getVelocity()
{
    return this->velocity;
}

void NormalDrivetrain::setBrakeMode(pros::motor_brake_mode_e_t mode)
{
    leftMotors.set_brake_modes(mode);
    rightMotors.set_brake_modes(mode);
}
#include "drivetrain.h"

Drivetrain::Drivetrain(pros::Imu *inertial, IntakeSensor *intake)
{
    this->inertial = inertial;
    this->intakeSensor = intake;
}

pros::Task Drivetrain::pidDrive(int velocityPercent, double deltaRotation, vector<double> gains)
{
    double target = this->inertial->get_rotation() + deltaRotation;
    pros::lcd::print(0, "Target: %f", target);
    pros::Task task([this, target, velocityPercent, gains] {
        pros::Task task = pros::Task::current();
        double integralTotal = 0;
        double prevError = 0;
        double prevTime = pros::millis();
        while (!task.notify_take(true, 5))
        {
            double deltaTime = pros::millis() - prevTime;
            pros::lcd::print(1, "Delta Time: %f", deltaTime);

            // P
            double error = target - this->inertial->get_rotation();
            pros::lcd::print(2, "Error: %f", error);
            double p = error * gains[0];
            pros::lcd::print(3, "P: %f", p);

            // I
            double i = error * deltaTime * gains[1] / 1000.0;
            integralTotal += i;
            pros::lcd::print(4, "I: %f", integralTotal);

            double derivative = ((error - prevError) / deltaTime) * gains[2] * 1000.0;
            pros::lcd::print(5, "D: %f", derivative);

            prevError = error;
            prevTime = pros::millis();

            double output = p + integralTotal + derivative;
            pros::lcd::print(6, "Output: %f", output);
            pros::lcd::print(7, "Velocity: %f", velocityPercent / 1000.0);
            this->voltageDrive(velocityPercent, output);
        }
        this->stop();
    });
    return task;
}

void Drivetrain::driveToObject(int velocityPercent) {
    pros::Task driveTask = pidDrive(velocityPercent);
    while (!intakeSensor->objectDetected()) {
        pros::delay(20);
    }

    driveTask.notify();
    driveTask.join();
}

void Drivetrain::rotateBy(double angle)
{
    pros::delay(100);
    double target = this->inertial->get_rotation() + angle;
    vector<bool> onTarget; // Keeps track of whether the robot is at the target angle at given intervals
    pros::Task driveTask = pidDrive(0, angle);
    do {
        pros::delay(50);
        onTarget.push_back(abs(this->inertial->get_rotation() - target) < 1);

        // Track for 1 second
        if (onTarget.size() > 20) {
            onTarget.erase(onTarget.begin());
        }
    } while (std::find(onTarget.begin(), onTarget.end(), false) != onTarget.end());
    
    driveTask.notify();
    driveTask.join();
}

void Drivetrain::rotateTo(double heading)
{
    double diff = heading - this->inertial->get_heading();
    
    // Normalize the difference to be in the range of -180 to 180
    if (diff > 180) {
        diff -= 360;
    } else if (diff < -180) {
        diff += 360;
    }
    
    // Determine the direction based on the normalized difference
    int direction = diff > 0 ? 1 : -1;
    int angle = diff > 0 ? diff : -diff;

    rotateBy(angle * direction);
}
#include "main.h"
#include "normal_drivetrain.h"
#include "okapi_drivetrain.h"
#include "flywheel_stick.h"
#include "distance_intake_sensor.h"
#include "wings.h"
#include "vision_wrapper.h"
#include "autonomous.h"

#define AUTONOMOUT_SCHEME AutonomousScheme::DEFENSE_SIDE

pros::Imu inertial(14);
DistanceIntakeSensor intakeSensor(15);

NormalDrivetrain drivetrain(
	MOTOR_GEARSET_6, 
	360/27,
	13, 12, 11, 18, 19, 20,
	true, false, false,
	&inertial, &intakeSensor
);
FlywheelStick flywheelStick(
	17, false, 16, false,
	&intakeSensor,
	&drivetrain
);
Wings wings('A', 'B');
VisionWrapper vision(10, &flywheelStick);

AutonomousManager autonManager(&drivetrain, &flywheelStick, &intakeSensor, &wings, &vision);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_background_color(0, 0, 0);
	pros::lcd::set_text_color(255, 255, 255);

	autonManager.initialize(AUTONOMOUT_SCHEME);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	autonManager.registerSchemes();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	drivetrain.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	autonManager.run();
	drivetrain.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(CONTROLLER_MASTER);
	drivetrain.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	flywheelStick.rotateArm(FlywheelStickState::Intake);

	while (true) {
		while (master.get_digital(DIGITAL_B)) {
			drivetrain.stop();
		}

		// ---COMMENT OUT DURING COMPETITION---
		if (master.get_digital(DIGITAL_UP) && master.get_digital(DIGITAL_DOWN)) {
			pidTuner(&master);
		}
		if (master.get_digital(DIGITAL_UP) && master.get_digital_new_press(DIGITAL_A)) {
			pros::Task autonTask(autonomous);

			while (master.get_digital(DIGITAL_A)) {
				pros::delay(100);
			}

			autonTask.remove();
		}
		// ---COMMENT OUT DURING COMPETITION---

		double driveVel = master.get_analog(ANALOG_LEFT_Y);
		double rotateAmount = master.get_analog(ANALOG_RIGHT_X) * 0.75;
		drivetrain.drive(driveVel, rotateAmount);

		if (master.get_digital_new_press(DIGITAL_Y)) {
			flywheelStick.toggleRollback();
		}

		if (master.get_digital_new_press(DIGITAL_X)) {
			wings.toggle();
		}

		if (master.get_digital_new_press(DIGITAL_L1)) {
			FlywheelStickState state = flywheelStick.getState();
			if (state == FlywheelStickState::Intake) {
				flywheelStick.rotateArm(FlywheelStickState::Flywheel);
			} else if (state == FlywheelStickState::Flywheel) {
				flywheelStick.rotateArm(FlywheelStickState::Block);
			}
		}
		if (master.get_digital_new_press(DIGITAL_L2)) {
			FlywheelStickState state = flywheelStick.getState();
			if (state == FlywheelStickState::Block) {
				flywheelStick.rotateArm(FlywheelStickState::Flywheel);
			} else if (state == FlywheelStickState::Flywheel) {
				flywheelStick.rotateArm(FlywheelStickState::Intake);
			}
		}

		if (master.get_digital(DIGITAL_R1)) {
			flywheelStick.spinFlywheel(false);
		} else if (master.get_digital(DIGITAL_R2)) {
			flywheelStick.spinFlywheel(true);
		} else if (!intakeSensor.isHoldingTriball()) { // Don't interfere with rollback prevention
			flywheelStick.stopFlywheel();
		}

		// std::string controllerStr = std::to_string(vision.getRotationToTriball());
		// master.set_text(0, 0, controllerStr);

		pros::delay(20);
	}
}

void pidTuner(pros::Controller *controller) {
	vector<double> gains = DEFAULT_GAINS;
	optional<pros::Task> driveTask;
	int selectedIndex = 0;

	while (true) {

		if (controller->get_digital_new_press(DIGITAL_A)) {
			if (driveTask) {
				driveTask->notify();
				driveTask->join();
				driveTask.reset();
			} else {
				driveTask = drivetrain.pidDrive(15, 0, gains);
			}
		}
		if (controller->get_digital_new_press(DIGITAL_B)) {
			if (driveTask) {
				driveTask->notify();
				driveTask->join();
				driveTask.reset();
			} else {
				driveTask = drivetrain.pidDrive(0, 90, gains);
			}
		}
		if (controller->get_digital_new_press(DIGITAL_X)) {
			if (driveTask) {
				driveTask->notify();
				driveTask->join();
				driveTask.reset();
			} else {
				driveTask = drivetrain.pidDrive(0, 180, gains);
			}
		}
		if (controller->get_digital_new_press(DIGITAL_Y)) {
			if (driveTask) {
				driveTask->notify();
				driveTask->join();
				driveTask.reset();
			} else {
				driveTask = drivetrain.pidDrive(0, 15, gains);
			}
		}

		if (controller->get_digital_new_press(DIGITAL_LEFT)) {
			selectedIndex = (selectedIndex - 1) % 3;
		}
		if (controller->get_digital_new_press(DIGITAL_RIGHT)) {
			selectedIndex = (selectedIndex + 1) % 3;
		}

		if (controller->get_digital_new_press(DIGITAL_UP)) {
			gains[selectedIndex] += controller->get_digital(DIGITAL_L1) ? 0.1 : 0.01;
		}
		if (controller->get_digital_new_press(DIGITAL_DOWN)) {
			gains[selectedIndex] -= controller->get_digital(DIGITAL_L1) ? 0.1 : 0.01;
		}

		string gainsStr = "Gains: ";
		switch (selectedIndex) {
			case 0:
				gainsStr += "P: ";
				break;
			case 1:
				gainsStr += "I: ";
				break;
			case 2:
				gainsStr += "D: ";
				break;
		}
		gainsStr += to_string(gains[selectedIndex]);
		controller->set_text(0, 0, gainsStr);

		pros::delay(20);
	}
}
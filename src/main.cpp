#include "main.h"
#include "drivetrain.h"
#include "flywheel_stick.h"
#include "wings.h"
#include "optical_intake_sensor.h"

Drivetrain drivetrain(
	AbstractMotor::GearsetRatioPair(AbstractMotor::gearset::blue, 1),
	13, 12, 11, 18, 19, 20,
	true, false, false
);
OpticalIntakeSensor intakeSensor(15);
FlywheelStick flywheelStick(
	17, false, 16, false,
	&intakeSensor,
	&drivetrain
);
Wings wings('A', 'B');

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
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
void competition_initialize() {}

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
void autonomous() {}

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
	Controller master;

	while (true) {
		while (master.getDigital(ControllerDigital::B)) {
			drivetrain.stop();
		}

		drivetrain.drive(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightX));

		if (master[ControllerDigital::Y].changedToPressed()) {
			flywheelStick.toggleRollback();
		}

		if (master[ControllerDigital::X].changedToPressed()) {
			wings.toggle();
		}

		if (master[ControllerDigital::L1].changedToPressed()) {
			FlywheelStickState state = flywheelStick.getState();
			if (state == FlywheelStickState::Intake) {
				flywheelStick.rotateArm(FlywheelStickState::Flywheel);
			} else if (state == FlywheelStickState::Flywheel) {
				flywheelStick.rotateArm(FlywheelStickState::Block);
			}
		}
		if (master[ControllerDigital::L2].changedToPressed()) {
			FlywheelStickState state = flywheelStick.getState();
			if (state == FlywheelStickState::Block) {
				flywheelStick.rotateArm(FlywheelStickState::Flywheel);
			} else if (state == FlywheelStickState::Flywheel) {
				flywheelStick.rotateArm(FlywheelStickState::Intake);
			}
		}

		if (master.getDigital(ControllerDigital::R1)) {
			flywheelStick.spinFlywheel(false);
		} else if (master.getDigital(ControllerDigital::R2)) {
			flywheelStick.spinFlywheel(true);
		} else if (!intakeSensor.isHoldingTriball()) { // Don't interfere with rollback prevention
			flywheelStick.stopFlywheel();
		}

		std::string velocityStr = std::to_string(intakeSensor.isHoldingTriball());
		master.setText(0, 0, velocityStr);

		pros::delay(20);
	}
}

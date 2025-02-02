#include "main.h"
#include "config.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();

	auto intake_motors = std::make_shared<pros::MotorGroup>(config::PORT_INTAKE);
	mechanism::Intake::initialize(intake_motors);

	auto arm_motors = std::make_shared<pros::Motor>(config::PORT_ARM);
	auto arm_rotation = std::make_shared<pros::Rotation>(config::PORT_ARM_ROTATION);

	mechanism::Arm::initialize(
		arm_motors,
		arm_rotation,
		config::ARM_PID,
		config::ARM_TARGET_CONFIG,
		config::ARM_kG);
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
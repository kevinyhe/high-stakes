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

	auto left_motors = std::make_shared<pros::MotorGroup>(config::PORT_LEFT_DRIVE);
	auto right_motors = std::make_shared<pros::MotorGroup>(config::PORT_RIGHT_DRIVE);
	
	// input curve for throttle input during driver control
	lemlib::ExpoDriveCurve throttle_curve(3,	// joystick deadband out of 127
										  10,	// minimum output where drivetrain will move out of 127
										  1.019 // expo curve gain
	);

	// input curve for steer input during driver control
	lemlib::ExpoDriveCurve steer_curve(3,	 // joystick deadband out of 127
									   10,	 // minimum output where drivetrain will move out of 127
									   1.019 // expo curve gain
	);

	drivetrain = std::make_shared<lemlib::Drivetrain>(left_motors, right_motors, config::DRIVE_WHEEL_TRACK, config::DRIVE_WHEEL_DIAMETER, config::DRIVE_RPM, config::DRIVE_HORIZONTAL_DRIFT);

	lemlib::OdomSensors sensors(
		new lemlib::TrackingWheel(&pros::Rotation(config::PORT_VERTICAL_ROTATION), config::VERTICAL_TRACKING_WHEEL_DIAMETER, config::VERTICAL_TRACKING_WHEEL_DISTANCE),
		nullptr,
		new lemlib::TrackingWheel(&pros::Rotation(config::PORT_LATERAL_ROTATION), config::HORIZONTAL_TRACKING_WHEEL_DIAMETER, config::HORIZONTAL_TRACKING_WHEEL_DISTANCE),
		nullptr,
		new pros::Imu(config::PORT_IMU));

	lemlib::ControllerSettings linearSettings(config::LINEAR_KP, config::LINEAR_KI, config::LINEAR_KD, config::LINEAR_WINDUP, config::LINEAR_SMALL_ERROR, config::LINEAR_SMALL_ERROR_TIMEOUT, config::LINEAR_LARGE_ERROR, config::LINEAR_LARGE_ERROR_TIMEOUT, config::LINEAR_SLEW);
	lemlib::ControllerSettings angularSettings(config::ANGULAR_KP, config::ANGULAR_KI, config::ANGULAR_KD, config::ANGULAR_WINDUP, config::ANGULAR_SMALL_ERROR, config::ANGULAR_SMALL_ERROR_TIMEOUT, config::ANGULAR_LARGE_ERROR, config::ANGULAR_LARGE_ERROR_TIMEOUT, config::ANGULAR_SLEW);

	chassis = std::make_shared<lemlib::Chassis>(*drivetrain, linearSettings, angularSettings, sensors);
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
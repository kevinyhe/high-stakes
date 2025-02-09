#include "main.h"
#include "config.hpp"
#include "mcl/mcl.hpp"

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

	arm_motors->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	mechanism::Arm::initialize(
		arm_motors,
		arm_rotation,
		config::ARM_PID,
		config::ARM_TARGET_CONFIG,
		config::ARM_kG);

	clamp.retract();
	doinker.retract();
	// arm_rotation->reset();
	arm_rotation->set_position(2000);

	chassis->calibrate(); // calibrate sensors

	pros::Task screenTask([&]()
						  {
        while (true) {
            // print robot location to the brain screen	
            pros::lcd::print(0, "X: %f", chassis->getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis->getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis->getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis->getPose());
            // delay to save resources
            pros::delay(50);
        } });

	// busted
	MCLOdom mcl_odom = MCLOdom({.particle_count = 2000,
								.uniform_random_percent = 0.1, // this randomness prevents the system from getting stuck in a bad guess
								.tracker_odom_sd = 0.05},
							   chassis,
							   {{
									// front
									.port = 1,
									.x_offset = -5.5,
									.y_offset = 7,
									.theta_offset = 0,
								},
								{
									// back
									.port = 1,
									.x_offset = -5.5,
									.y_offset = -5.5,
									.theta_offset = M_PI,
								},
								{
									// left
									.port = 1,
									.x_offset = -5.25,
									.y_offset = -3,
									.theta_offset = M_PI / 2.0,
								},
								{
									// right
									.port = 1,
									.x_offset = 7,
									.y_offset = -5,
									.theta_offset = -M_PI / 2.0,
								}});
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
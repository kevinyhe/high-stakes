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
	auto intake_optical = std::make_shared<pros::Optical>(config::PORT_RING_SORT_OPTICAL);
	auto intake_distance = std::make_shared<pros::Distance>(config::PORT_RING_SORT_DISTANCE);

	mechanism::Intake::initialize(intake_motors, intake_optical, intake_distance, config::SORT_DISTANCE, config::RED_BOUND, config::BLUE_BOUND);

	auto arm_motors = std::make_shared<pros::Motor>(config::PORT_ARM);
	auto arm_rotation = std::make_shared<pros::Rotation>(config::PORT_ARM_ROTATION);

	arm_motors->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	mechanism::Arm::initialize(
		arm_motors,
		arm_rotation,
		config::ARM_PID,
		config::ARM_TARGET_CONFIG,
		config::ARM_kG);

	mechanism::Clamp::initialize(
		std::make_shared<Pneumatic>(config::PORT_CLAMP),
		std::make_shared<pros::Distance>(config::PORT_CLAMP_DISTANCE),
		config::AUTOCLAMP_THRESHOLD);

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
// 	MCLOdom mcl_odom = MCLOdom({.particle_count = 2000,
// 								.uniform_random_percent = 0.1, // this randomness prevents the system from getting stuck in a bad guess
// 								.tracker_odom_sd = 0.05},
// 							   chassis,
// 							   {{
// 									// front
// 									.port = 1,
// 									.x_offset = -5.5,
// 									.y_offset = 7,
// 									.theta_offset = 0,
// 								},
// 								{
// 									// back
// 									.port = 1,
// 									.x_offset = -5.5,
// 									.y_offset = -5.5,
// 									.theta_offset = M_PI,
// 								},
// 								{
// 									// left
// 									.port = 1,
// 									.x_offset = -5.25,
// 									.y_offset = -3,
// 									.theta_offset = M_PI / 2.0,
// 								},
// 								{
// 									// right
// 									.port = 1,
// 									.x_offset = 7,
// 									.y_offset = -5,
// 									.theta_offset = -M_PI / 2.0,
// 								}});
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
int prev_pot_range = 0;

void competition_initialize() {
	while (pros::competition::is_disabled()) {
		// get the potentiometer value
		int pot_value = potentiometer.get_value();
		// depending on which auton is selected, run the corresponding function
		// get one of 8 ranges from potentiometer
		int pot_range = std::floor(pot_value / 45);
		int prev_pot_range = pot_range;
	
		if (prev_pot_range != pot_range) {
			controller.rumble(".");
		}
	
		switch (pot_range)
		{
		case 0:
			controller.print(0, 0, "Red Mogo");
			break;
		case 1:
			controller.print(0, 0, "Blue Mogo");
			break;
		case 2:
			controller.print(0, 0, "Red Ring");
			break;
		case 3:
			controller.print(0, 0, "Blue Ring");
			break;
		case 4:
			controller.print(0, 0, "Prog Skills");
			break;
		case 5:
			controller.print(0, 0, "Default");
			break;
		default:
			break;
		}
	}
}
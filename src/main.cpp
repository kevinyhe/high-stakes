#include "main.h"
#include "autonomous.hpp"
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

	auto intake_f_motor = std::make_shared<pros::Motor>(config::PORT_INTAKE_FIRST);
	auto intake_s_motor = std::make_shared<pros::Motor>(config::PORT_INTAKE_SECOND);
	auto intake_optical = std::make_shared<pros::Optical>(config::PORT_RING_SORT_OPTICAL);
	auto intake_distance = std::make_shared<pros::Distance>(config::PORT_RING_SORT_DISTANCE);

	mechanism::Intake::initialize(intake_f_motor, intake_s_motor, intake_optical, intake_distance, config::SORT_DISTANCE, config::RED_BOUND, config::BLUE_BOUND);

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
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	
}

void find_tracking_center(float turnVoltage, uint32_t time)
{
	chassis->setPose(0, 0, 0);
	unsigned long n = 0;
	float heading;

	std::cout << std::fixed << "\033[1mCopy this:\033[0m\n\\left[";
	chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis->tank(-turnVoltage, turnVoltage);

	std::ostringstream out;

	auto end_time = time + pros::millis();

	std::vector<float> thetas;

	int i = 0;

	while (pros::millis() < end_time && i++ < 12000)
	{
		std::cout << "\\left(" << chassis->getPose().x << "," << chassis->getPose().y << "\\right),";
		thetas.emplace_back(chassis->getPose(true).theta);

		/*if (i % 250 == 0) {
		  std::cout << "\\right]\n\\left[" ;
		} */
		if (i % 50 == 0)
		{
			std::cout.flush();
		}
		pros::delay(20);
	}
	chassis->cancelAllMotions();
	std::cout << "\b\\right]" << std::endl;

	pros::delay(1000);
	std::cout << "\\theta_{t}=\\left[";
	i = 0;

	for (auto &theta : thetas)
	{
		i++;
		std::cout << theta << ",";
		if (i % 50 == 0)
			std::cout.flush();
		pros::delay(20);
	}
	std::cout << "\b\\right]" << std::endl;

	std::cout << "Go to https://www.desmos.com/calculator/rxdoxxil1j to solve for offsets." << std::endl;
}

void autonomous()
{
	find_tracking_center(70, 12000);

	auto &auton_selector = AutonSelector::get_instance();
	auton_selector.run_selected_routine();
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
	int prev_pot_range = 0;

	while (pros::competition::is_disabled())
	{
		// get the potentiometer value
		int pot_value = potentiometer.get_value();
		// depending on which auton is selected, run the corresponding function
		// get one of 8 ranges from potentiometer
		int pot_range = std::floor(pot_value / 45);

		if (prev_pot_range != pot_range)
		{
			controller.rumble(".");
			prev_pot_range = pot_range;
		}

		auto &auton_selector = AutonSelector::get_instance();

		switch (pot_range)
		{
		case 0:
			controller.print(0, 0, "Red Mogo   ");
			auton_selector.set_auton_routine(AutonSelector::AutonRoutine::RED_MOGO);
			break;
		case 1:
			controller.print(0, 0, "Blue Mogo  ");
			auton_selector.set_auton_routine(AutonSelector::AutonRoutine::BLUE_MOGO);
			break;
		case 2:
			controller.print(0, 0, "Red Ring   ");
			auton_selector.set_auton_routine(AutonSelector::AutonRoutine::RED_RING);
			break;
		case 3:
			controller.print(0, 0, "Blue Ring  ");
			auton_selector.set_auton_routine(AutonSelector::AutonRoutine::BLUE_RING);
			break;
		case 4:
			controller.print(0, 0, "Prog Skills");
			auton_selector.set_auton_routine(AutonSelector::AutonRoutine::PROG_SKILLS);
			break;
		case 5:
			controller.print(0, 0, "Default    ");
			auton_selector.set_auton_routine(AutonSelector::AutonRoutine::DEFAULT);
			break;
		default:
			break;
		}
		pros::delay(50);
	}
}
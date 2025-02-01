#include "main.h"
#include "config.h"
#include "mechanism/arm.hpp"
#include "mechanism/intake.hpp"

// wont use pointers bc lemlib
pros::MotorGroup left_drive(config::PORT_LEFT_DRIVE);
pros::MotorGroup right_drive(config::PORT_RIGHT_DRIVE);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_drive,
                              &right_drive,
                              config::DRIVE_WHEEL_TRACK,
                              config::DRIVE_WHEEL_DIAMETER,
                              config::DRIVE_GEAR_RATIO * 600,
                              0);

std::shared_ptr<mechanism::Intake> intake = std::make_shared<mechanism::Intake>(std::make_shared<pros::MotorGroup>(config::PORT_INTAKE));
std::shared_ptr<mechanism::Arm> arm = std::make_shared<mechanism::Arm>(
    std::make_shared<pros::Motor>(config::PORT_ARM),
    std::make_shared<pros::Rotation>(config::PORT_ARM_ROTATION),
    std::make_shared<mechanism::Intake>(std::make_shared<pros::MotorGroup>(config::PORT_INTAKE)),
    std::make_shared<PID>(config::PARAMS_ARM_PID),
    config::ARM_TARGET_CONFIG,
    config::ARM_GEAR_RATIO);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

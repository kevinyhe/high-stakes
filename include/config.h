#pragma once

#include <initializer_list>
#include "main.h"
#include "mechanism/arm.hpp"
#include "mechanism/intake.hpp"

namespace config
{
    // dt
    constexpr std::initializer_list<int8_t> PORT_LEFT_DRIVE = {-1, -2, -3};
    constexpr std::initializer_list<int8_t> PORT_RIGHT_DRIVE = {4, 5, 6};

    constexpr double DRIVE_WHEEL_DIAMETER = 3.25;
    constexpr double DRIVE_WHEEL_TRACK = 13;         // TODO:
    constexpr double DRIVE_GEAR_RATIO = 36.0 / 48.0; // TODO:

    // subsystems
    constexpr std::initializer_list<int8_t> PORT_INTAKE = {9, -11};
    constexpr int8_t PORT_ARM = 10;
    constexpr int8_t PORT_ARM_ROTATION = 0; // TODO:

    constexpr double ARM_GEAR_RATIO = 1.0 / 3.0; // TODO:

    constexpr mechanism::ArmTargetConfig ARM_TARGET_CONFIG = {
        load : -44.0,
        alliance_stake : 10.0,
        ladder_touch : 25.0,
        neutral_stake : 39.0
    };
    constexpr PIDParameters PARAMS_ARM_PID = { // TODO: tune
        kP : 5.0,
        kI : 0.0,
        kD : 1.0
    };

    // sensors
    constexpr int8_t PORT_IMU = 0; // TODO:
    constexpr int8_t PORT_VERTICAL_ROTATION = 0; // TODO:
    constexpr int8_t PORT_LATERAL_ROTATION = 0;  // TODO:
}

extern pros::MotorGroup left_drive;
extern pros::MotorGroup right_drive;

extern lemlib::Drivetrain drivetrain;

extern std::shared_ptr<mechanism::Arm> arm;
extern std::shared_ptr<mechanism::Intake> intake;

extern pros::Controller controller;

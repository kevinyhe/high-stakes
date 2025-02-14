#pragma once

#include "main.h"
#include "controller/pid.hpp"
#include "mechanism/arm.hpp"
#include "mechanism/intake.hpp"
#include "mechanism/clamp.hpp"
#include "device/pneumatic.hpp"
#include "math/math.hpp"
#include "math/vector.hpp"
#include "math/angle.hpp"
#include "math/pose.hpp"

namespace config
{
    // dt
    inline const std::initializer_list<int8_t> PORT_LEFT_DRIVE = {-1, -2, -3};
    inline const std::initializer_list<int8_t> PORT_RIGHT_DRIVE = {4, 5, 6};

    inline const double DRIVE_WHEEL_DIAMETER = 3.25;
    inline const double DRIVE_WHEEL_TRACK = 15;      
    inline const double DRIVE_GEAR_RATIO = 36.0 / 48.0; 
    inline const int DRIVE_RPM = 450;                 
    inline const int DRIVE_HORIZONTAL_DRIFT = 2; // coefficient of friction


    // subsystems
    inline const std::initializer_list<int8_t> PORT_INTAKE = {9, -11};

    inline const int8_t PORT_ARM = 10;
    inline const int8_t PORT_ARM_ROTATION = -18;

    inline const char PORT_DOINKER = 'E';
    inline const char PORT_INTAKE_LIFT = 'F'; 
    inline const char PORT_CLAMP = 'G'; 

    inline const mechanism::ArmTargetConfig ARM_TARGET_CONFIG = {
        load : 54.5,
        idle : 20.0,
        prime : 90.0,
        neutral_stake : 175.0
    };
    inline const PIDParameters PARAMS_ARM_PID = {
        kP : 5.2,
        kI : 0.0,
        kD : 15.0,
    };
    inline const double ARM_kG = 9.8;
    inline const std::shared_ptr<PID> ARM_PID = std::make_shared<PID>(PARAMS_ARM_PID);

    // sensors
    inline const int8_t PORT_IMU = 19;      
    inline const int8_t PORT_VERTICAL_ROTATION = 21; 
    inline const int8_t PORT_LATERAL_ROTATION = -8; 
    
    inline const double VERTICAL_TRACKING_WHEEL_DIAMETER = 2;
    inline const double VERTICAL_TRACKING_WHEEL_DISTANCE = 0.5; // TODO:
    inline const double HORIZONTAL_TRACKING_WHEEL_DIAMETER = 2;
    inline const double HORIZONTAL_TRACKING_WHEEL_DISTANCE = -0.5; // TODO:

    // PID
    inline const double LINEAR_KP = 7.0; 
    inline const double LINEAR_KI = 0.0; 
    inline const double LINEAR_KD = 10.0; 

    inline const double LINEAR_WINDUP = 0.0; 
    inline const double LINEAR_SMALL_ERROR = 0.5;
    inline const double LINEAR_SMALL_ERROR_TIMEOUT = 100;
    inline const double LINEAR_LARGE_ERROR = 2.5; 
    inline const double LINEAR_LARGE_ERROR_TIMEOUT = 500; 
    inline const double LINEAR_SLEW = 0.0; 

    inline double ANGULAR_KP = 2.8; 
    inline double ANGULAR_KI = 0.0; 
    inline double ANGULAR_KD = 9.0; 

    inline const double ANGULAR_WINDUP = 0.0; 
    inline const double ANGULAR_SMALL_ERROR = 1.2; 
    inline const double ANGULAR_SMALL_ERROR_TIMEOUT = 200; 
    inline const double ANGULAR_LARGE_ERROR = 2.5; 
    inline const double ANGULAR_LARGE_ERROR_TIMEOUT = 1000; 
    inline const double ANGULAR_SLEW = 0.0; 

    inline const int8_t PORT_RING_SORT_OPTICAL = 20;
    inline const int8_t PORT_RING_SORT_DISTANCE = 7;

    inline const int8_t PORT_CLAMP_DISTANCE = 17;
    inline const double AUTOCLAMP_THRESHOLD = 32;

    inline const double SORT_DISTANCE = 30;
    inline const double RED_BOUND = 35;
    inline const double BLUE_BOUND = 160;
} // namespace config

inline pros::MotorGroup left_motors = pros::MotorGroup(config::PORT_LEFT_DRIVE, pros::MotorGearset::blue);
inline pros::MotorGroup right_motors = pros::MotorGroup(config::PORT_RIGHT_DRIVE, pros::MotorGearset::blue);

// input curve for throttle input during driver control
inline lemlib::ExpoDriveCurve throttle_curve(3,    // joystick deadband out of 127
                                      10,   // minimum output where drivetrain will move out of 127
                                      1.019 // expo curve gain
);

// input curve for steer input during driver control
inline lemlib::ExpoDriveCurve steer_curve(3,    // joystick deadband out of 127
                                   10,   // minimum output where drivetrain will move out of 127
                                   1.019 // expo curve gain
);

inline lemlib::Drivetrain drivetrain(&left_motors, &right_motors, config::DRIVE_WHEEL_TRACK, config::DRIVE_WHEEL_DIAMETER, config::DRIVE_RPM, config::DRIVE_HORIZONTAL_DRIFT);

inline pros::Rotation vertical_rotation(config::PORT_VERTICAL_ROTATION);
inline pros::Rotation lateral_rotation(config::PORT_LATERAL_ROTATION);
inline pros::Imu imu(config::PORT_IMU);

inline Pneumatic doinker = Pneumatic(config::PORT_DOINKER);
inline Pneumatic intake_lift = Pneumatic(config::PORT_INTAKE_LIFT);

inline lemlib::OdomSensors sensors(
    new lemlib::TrackingWheel(&vertical_rotation, config::VERTICAL_TRACKING_WHEEL_DIAMETER, config::VERTICAL_TRACKING_WHEEL_DISTANCE),
    nullptr,
    new lemlib::TrackingWheel(&lateral_rotation, config::HORIZONTAL_TRACKING_WHEEL_DIAMETER, config::HORIZONTAL_TRACKING_WHEEL_DISTANCE),
    nullptr,
    &imu);

inline lemlib::ControllerSettings linearSettings(config::LINEAR_KP, config::LINEAR_KI, config::LINEAR_KD, config::LINEAR_WINDUP, config::LINEAR_SMALL_ERROR, config::LINEAR_SMALL_ERROR_TIMEOUT, config::LINEAR_LARGE_ERROR, config::LINEAR_LARGE_ERROR_TIMEOUT, config::LINEAR_SLEW);
inline lemlib::ControllerSettings angularSettings(config::ANGULAR_KP, config::ANGULAR_KI, config::ANGULAR_KD, config::ANGULAR_WINDUP, config::ANGULAR_SMALL_ERROR, config::ANGULAR_SMALL_ERROR_TIMEOUT, config::ANGULAR_LARGE_ERROR, config::ANGULAR_LARGE_ERROR_TIMEOUT, config::ANGULAR_SLEW);

inline std::shared_ptr<lemlib::Chassis> chassis = std::make_shared<lemlib::Chassis>(drivetrain, linearSettings, angularSettings, sensors);

// 358.29
// 357.80
// 358.29
// 358.97

// 1.0046394809
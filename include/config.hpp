#include "main.h"
#include "controller/pid.hpp"
#include "mechanism/arm.hpp"
#include "mechanism/intake.hpp"

namespace config
{
    // dt
    inline const std::initializer_list<int8_t> PORT_LEFT_DRIVE = {-1, -2, -3};
    inline const std::initializer_list<int8_t> PORT_RIGHT_DRIVE = {4, 5, 6};

    inline const double DRIVE_WHEEL_DIAMETER = 3.25;
    inline const double DRIVE_WHEEL_TRACK = 13;         // TODO:
    inline const double DRIVE_GEAR_RATIO = 36.0 / 48.0; // TODO:
    inline const int DRIVE_RPM = 360;                   // TODO:
    inline const int DRIVE_HORIZONTAL_DRIFT = 0;


    // subsystems
    inline const std::initializer_list<int8_t> PORT_INTAKE = {9, -11};

    inline const int8_t PORT_ARM = 10;
    inline const int8_t PORT_ARM_ROTATION = 0; // TODO:

    inline const double ARM_GEAR_RATIO = 1.0 / 3.0; // TODO:

    inline const mechanism::ArmTargetConfig ARM_TARGET_CONFIG = {
        load : -44.0,
        alliance_stake : 10.0,
        ladder_touch : 25.0,
        neutral_stake : 39.0
    };
    inline const PIDParameters PARAMS_ARM_PID = { // TODO: tune
        kP : 5.0,
        kI : 0.0,
        kD : 1.0
    };
    inline const double ARM_kG = 10.0; // TODO:
    inline const std::shared_ptr<PID> ARM_PID = std::make_shared<PID>(PARAMS_ARM_PID);

    // sensors
    inline const int8_t PORT_IMU = 0;               // TODO:
    inline const int8_t PORT_VERTICAL_ROTATION = 0; // TODO:
    inline const int8_t PORT_LATERAL_ROTATION = 0;  // TODO:
    
    inline const double VERTICAL_TRACKING_WHEEL_DIAMETER = 2.0; // TODO:
    inline const double VERTICAL_TRACKING_WHEEL_DISTANCE = 0;     // TODO:
    inline const double HORIZONTAL_TRACKING_WHEEL_DIAMETER = 2.0; // TODO:
    inline const double HORIZONTAL_TRACKING_WHEEL_DISTANCE = 0;     // TODO:
    
    // PID
    inline const double LINEAR_KP = 0.0; 
    inline const double LINEAR_KI = 0.0; 
    inline const double LINEAR_KD = 0.0; 

    inline const double LINEAR_WINDUP = 0.0; 
    inline const double LINEAR_SMALL_ERROR = 0.0; 
    inline const double LINEAR_SMALL_ERROR_TIMEOUT = 0.0; 
    inline const double LINEAR_LARGE_ERROR = 0.0; 
    inline const double LINEAR_LARGE_ERROR_TIMEOUT = 0.0; 
    inline const double LINEAR_SLEW = 0.0; 

    inline const double ANGULAR_KP = 0.0; 
    inline const double ANGULAR_KI = 0.0; 
    inline const double ANGULAR_KD = 0.0; 

    inline const double ANGULAR_WINDUP = 0.0; 
    inline const double ANGULAR_SMALL_ERROR = 0.0; 
    inline const double ANGULAR_SMALL_ERROR_TIMEOUT = 0.0; 
    inline const double ANGULAR_LARGE_ERROR = 0.0; 
    inline const double ANGULAR_LARGE_ERROR_TIMEOUT = 0.0; 
    inline const double ANGULAR_SLEW = 0.0; 
} // namespace config

extern std::shared_ptr<lemlib::Drivetrain> drivetrain;
extern std::shared_ptr<lemlib::Chassis> chassis;

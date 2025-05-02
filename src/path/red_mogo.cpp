#include "autonomous.hpp"
#include "config.hpp"
#include "main.h"

void red_mogo()
{
    chassis->setPose(-49.381, -37.6, 101.2);
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::BLUE);

    arm.set_rotation_value(65.7);
    chassis->moveToPoint(-29.215, -43.969, 2000, {.minSpeed = 127, .earlyExitRange = 8});
    arm.set_state(mechanism::ArmState::PRIME);
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPoint(-12.769, -45.849, 2000, {.minSpeed = 127, .earlyExitRange = 6});
    intake.enable_stop_next_ring();
    chassis->waitUntil(2.7);
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    // chassis->swingToHeading(-135, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 127, .earlyExitRange = 20});
    chassis->waitUntilDone();
    chassis->tank(40, -127);
    pros::delay(900);
    intake.disable_stop_next_ring();
    arm.set_state(mechanism::ArmState::IDLE);
    chassis->moveToPoint(-24, -24, 2000, {.forwards = false});
}
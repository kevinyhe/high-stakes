#include "autonomous.hpp"
#include "config.hpp"
#include "main.h"

void red_mogo()
{
    chassis->setPose(-48.406, -36.651, 100.8);
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::BLUE);

    chassis->moveToPoint(-12.627, -44.294, 3000, {.minSpeed = 127, .earlyExitRange = 8});
    doinker.extend();
    arm.set_state(mechanism::ArmState::LOAD);
    chassis->waitUntil(4);
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->waitUntil(22);
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    chassis->waitUntilDone();
    chassis->swingToPoint(-24, -24, lemlib::DriveSide::RIGHT, 3000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 6});
    chassis->waitUntilDone();
    arm.set_state(mechanism::ArmState::PRIME);
    chassis->moveToPoint(-24, -24, 3000, {.forwards = false});
    clamp.set_autoclamp(true);
}
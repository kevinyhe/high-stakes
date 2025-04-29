#include "autonomous.hpp"
#include "config.hpp"
#include "main.h"

void red_awp_safe()
{
    chassis->setPose(-61.5, 13.6, 212.0);
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::BLUE);

    arm.set_rotation_value(65.7);
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    pros::delay(350);
    chassis->moveToPoint(-54.423, 24.824, 2000, {.forwards = false, .minSpeed = 127});
    chassis->waitUntil(1);
    chassis->cancelMotion();
    chassis->moveToPoint(-36.835, 20.867, 3000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 5});
    arm.set_state(mechanism::ArmState::PRIME);
    chassis->moveToPoint(-30.504, 24.089, 3000, {.forwards = false, .maxSpeed = 85});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {

        pros::delay(10);
    }
    chassis->cancelMotion();
    clamp.extend();


}